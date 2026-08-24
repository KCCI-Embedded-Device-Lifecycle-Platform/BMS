/**
 * @file    hw_adc.c
 * @brief   ADC1 + DMA2_Stream0 (Circular, Scan, Continuous) 드라이버
 *
 * ADC 가 Scan/Continuous 로 10랭크를 계속 돌고 DMA(Circular)가 s_dma_buf[] 를 자동 갱신한다.
 * 스캔이 끝날 때마다 TC 인터럽트가 뜨고, ISR 은 덧셈 10회만 해서 누적기에 더한다
 * (ISR 최소 연산 원칙). CFG_ADC_OVERSAMPLE_N(16)회 누적되면 평균을 확정한다 —
 * STM32F4 에는 하드웨어 오버샘플링이 없어 소프트웨어로 구현한 것이고,
 * 16회 평균이면 랜덤 노이즈가 1/4 로 줄어든다 (유효 2bit 증가).
 *
 * @note    타이밍: ADCCLK = PCLK2(84MHz)/8 = 10.5MHz
 *          채널당 (480+12)/10.5MHz = 46.9us -> 1스캔 469us -> 평균 갱신 7.5ms.
 *          100ms 태스크에 충분하다.
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
static volatile uint32_t s_last_avg_ms;

bool hw_adc_init(void)
{
    uint8_t i;

    for (i = 0; i < (uint8_t)ADC_IDX_MAX; i++) {
        s_acc[i] = 0;
        s_avg[i] = 0;
    }
    s_acc_cnt = 0;
    s_ready   = false;
    s_last_avg_ms = 0U;

    /* Vrefint 채널은 ADC_CCR 의 TSVREFE 로 별도 인에이블이 필요하다.
     * CubeMX 가 켜 주지만 명시적으로 한 번 더 보장한다. */
    ADC->CCR |= ADC_CCR_TSVREFE;

    /* --- DDS 강제 ---
     * DDS 가 0 이면 시퀀스 마지막 변환 이후 DMA 요청을 내지 않아, Circular 라도 첫 스캔
     * 한 번만 전송되고 멈춘다 -> TC 콜백이 1회만 떠서 hw_adc_is_ready() 가 영원히 false.
     * CubeMX 의 "DMA Continuous Requests" 체크박스가 꺼진 채로 재생성되면 조용히 다시
     * 깨지므로, 재생성에 영향받지 않도록 여기서 핸들과 레지스터를 함께 못 박는다. */
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

bool hw_adc_is_fresh(uint32_t max_age_ms)
{
    return s_ready && ((uint32_t)(HAL_GetTick() - s_last_avg_ms) <= max_age_ms);
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
 * @note   VDDA = 3300mV * VREFINT_CAL / VREFINT_DATA (CAL 은 공장 측정값).
 *         USB 전원이 4.6~5.2V 로 흔들리면 LDO 출력도 같이 변하므로, 고정 3300mV 를 쓰면
 *         셀 전압에 수십 mV 오차가 그대로 실린다.
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
     * 다만 raw * vdda_mv * 1000 을 한 번에 곱하면 int32 를 넘는다
     * (4095*3300*1000 = 135억). 그래서 몫/나머지로 쪼개 32bit 안에서 계산한다. */
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
        s_last_avg_ms = HAL_GetTick();
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
        s_ready = false;
        DBG_E("ADC error 0x%lX", (unsigned long)HAL_ADC_GetError(hadc));
    }
}
