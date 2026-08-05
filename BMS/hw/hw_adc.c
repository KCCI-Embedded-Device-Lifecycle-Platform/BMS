/**
 * @file    hw_adc.c
 * @brief   ADC1 + DMA2_Stream0 (Circular, Scan, Continuous) 드라이버
 *
 * @details 동작 원리
 *   1) ADC 가 Scan/Continuous 모드로 8개 랭크를 쉬지 않고 순환 변환한다.
 *   2) DMA 가 Circular 모드로 결과를 s_dma_buf[] 에 자동 갱신한다.
 *      -> CPU 개입 없이 데이터가 항상 최신으로 유지된다.
 *   3) 한 스캔이 끝날 때마다 TC 인터럽트가 발생하고,
 *      ISR 은 "덧셈 8회"만 수행하여 누적기에 더한다. (ISR 최소 연산 원칙)
 *   4) CFG_ADC_OVERSAMPLE_N(16) 회 누적되면 평균을 확정한다.
 *      STM32F4 는 하드웨어 오버샘플링이 없으므로 소프트웨어로 구현.
 *      16회 평균 -> 랜덤 노이즈가 1/4 로 감소 (SNR +12dB, 유효 2bit 증가)
 *
 * @note    변환 시간 계산
 *          ADCCLK = PCLK2(84MHz) / 8 = 10.5MHz
 *          채널당 = (480 sampling + 12 conversion) / 10.5MHz = 46.9us
 *          1 스캔  = 46.9us x 8 = 375us  -> ISR 주기 약 2.7kHz
 *          평균 갱신 주기 = 375us x 16 = 6ms (100ms 태스크에 충분)
 */
#include "hw_adc.h"
#include "dbg.h"

extern ADC_HandleTypeDef hadc1;         /* CubeMX 생성 (adc.c) */

/* --- DMA 대상 버퍼 : 하드웨어가 임의 시점에 갱신하므로 volatile 필수 --- */
static volatile uint16_t s_dma_buf[ADC_IDX_MAX];

/* --- ISR 누적기 --- */
static volatile uint32_t s_acc[ADC_IDX_MAX];
static volatile uint16_t s_avg[ADC_IDX_MAX];
static volatile uint8_t  s_acc_cnt;
static volatile bool     s_ready;

bool hw_adc_init(void)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)ADC_IDX_MAX; i++) {
        s_acc[i] = 0;
        s_avg[i] = 0;
    }
    s_acc_cnt = 0;
    s_ready   = false;

    /* 내부 참조전압 채널은 ADC_CCR 의 TSVREFE 비트로 별도 인에이블이 필요하다.
     * CubeMX 에서 Vrefint 를 랭크에 추가하면 자동으로 켜지지만,
     * 명시적으로 한 번 더 보장한다. */
    ADC->CCR |= ADC_CCR_TSVREFE;

    /* --- DDS(DMA Disable Selection) 강제 --- *
     * CR2 의 DDS 가 0 이면 "시퀀스 마지막 변환 이후로는 DMA 요청을 내지
     * 않는다". Circular DMA 라도 첫 8채널 스캔 한 번만 전송되고 멈춰서
     * TC 콜백이 딱 1회 뜨고 끝난다. -> s_acc_cnt 가 16 에 도달하지 못해
     * hw_adc_is_ready() 가 영원히 false 가 된다.
     *
     * CubeMX 의 ADC1 설정에 "DMA Continuous Requests = Enabled" 를 켜면
     * adc.c 가 올바르게 생성되지만, 그 체크박스가 꺼진 채로 재생성되면
     * 조용히 다시 깨진다. 재생성에 영향받지 않도록 여기서 못을 박는다.
     * (핸들과 레지스터를 함께 맞춰 둬야 이후 HAL_ADC_Init 재호출에도 안전) */
    hadc1.Init.DMAContinuousRequests = ENABLE;
    SET_BIT(hadc1.Instance->CR2, ADC_CR2_DDS);

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_dma_buf, (uint32_t)ADC_IDX_MAX) != HAL_OK) {
        DBG_E("ADC DMA start failed");
        return false;
    }
    DBG_I("ADC1 + DMA started (%d ch, oversample x%d)", ADC_IDX_MAX, CFG_ADC_OVERSAMPLE_N);
    return true;
}

bool hw_adc_is_ready(void)
{
    return s_ready;
}

void hw_adc_snapshot(uint16_t *p_out)
{
    uint8_t i;

    /* ISR 이 s_avg[] 를 갱신하는 도중에 읽으면 채널 간 시점이 어긋난다(tearing).
     * 복사 구간만 인터럽트를 막아 일관된 스냅샷을 보장한다. (약 1us) */
    CRITICAL_ENTER();
    for (i = 0; i < (uint8_t)ADC_IDX_MAX; i++) {
        p_out[i] = s_avg[i];
    }
    CRITICAL_EXIT();
}

uint16_t hw_adc_get_raw(adc_idx_t idx)
{
    if (idx >= ADC_IDX_MAX) {
        return 0;
    }
    return s_avg[idx];
}

/**
 * @brief  Vrefint 를 이용해 실제 VDDA 를 역산한다.
 * @note   VDDA = 3300mV * VREFINT_CAL / VREFINT_DATA
 *         VREFINT_CAL 은 공장에서 VDDA=3.3V, 30도 조건으로 측정해
 *         0x1FFF7A2A 에 저장해 둔 값이다.
 *         USB 전원은 4.6~5.2V 로 흔들리고 LDO 출력도 함께 변하므로,
 *         고정 3300mV 를 쓰면 셀 전압에 수십 mV 오차가 그대로 실린다.
 */
int32_t hw_adc_get_vdda_mv(void)
{
    uint16_t cal = *CFG_VREFINT_CAL_ADDR;
    uint16_t now = s_avg[ADC_IDX_VREFINT];

    if ((now == 0U) || (cal == 0U)) {
        return CFG_VREFINT_CAL_VDDA_MV;     /* 아직 준비 전이면 공칭값 */
    }
    return (int32_t)DIV_ROUND((int32_t)CFG_VREFINT_CAL_VDDA_MV * (int32_t)cal, (int32_t)now);
}

int32_t hw_adc_raw_to_uv(uint16_t raw)
{
    int32_t vdda_mv = hw_adc_get_vdda_mv();
    int32_t num, q, r;

    /* uV 로 계산해야 이후 분압 복원(x6)에서 절삭 오차가 누적되지 않는다.
     *
     * raw * vdda_mv * 1000 을 한 번에 곱하면 int32 가 넘친다.
     *   4095 * 3300 * 1000 = 13,513,500,000  >  2,147,483,647
     * (raw 가 약 650 만 넘어도 오버플로우 -> 음수/엉뚱한 uV 가 나온다)
     * 그래서 몫/나머지로 쪼개 32bit 안에서 정확히 계산한다.
     *   num = raw * vdda_mv        <= 4095 * 3600 = 14.7M   (안전)
     *   q*1000                     <= 3600 * 1000 = 3.6M    (안전)
     *   r*1000                     <  4095 * 1000 = 4.1M    (안전)
     */
    num = (int32_t)raw * vdda_mv;
    q   = num / CFG_ADC_FULL_SCALE;
    r   = num % CFG_ADC_FULL_SCALE;

    return (q * 1000L) + (int32_t)DIV_ROUND(r * 1000L, CFG_ADC_FULL_SCALE);
}

/* ==================================================================
 *  DMA 전송완료 콜백 (ISR 컨텍스트)
 *  - 여기서는 덧셈만 한다. 나눗셈/조건분기/로그 금지.
 * ================================================================== */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    uint8_t i;

    if (hadc->Instance != ADC1) {
        return;
    }

    for (i = 0; i < (uint8_t)ADC_IDX_MAX; i++) {
        s_acc[i] += s_dma_buf[i];
    }

    s_acc_cnt++;
    if (s_acc_cnt >= CFG_ADC_OVERSAMPLE_N) {
        for (i = 0; i < (uint8_t)ADC_IDX_MAX; i++) {
            /* N 이 2의 거듭제곱이므로 컴파일러가 시프트로 최적화한다 */
            s_avg[i] = (uint16_t)(s_acc[i] / CFG_ADC_OVERSAMPLE_N);
            s_acc[i] = 0;
        }
        s_acc_cnt = 0;
        s_ready   = true;
    }
}

/* Circular DMA 는 절반 시점에도 콜백이 뜬다. 스캔 중간이므로 무시한다. */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED_ARG(hadc);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        DBG_E("ADC error 0x%lX", (unsigned long)HAL_ADC_GetError(hadc));
    }
}
