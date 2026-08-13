/**
 * @file    acs712.c
 * @brief   ACS712-05B 홀 효과 전류 센서 (팩 센서 크로스체크용)
 *
 * 전류를 두 번 재는 것은 중복이 아니다. 팩 센서는 션트(전기적), ACS712 는 홀(자기적)로
 * 원리가 달라서, 둘의 차이가 SENSOR_ERR 의 유일한 근거다. 센서가 하나뿐이면 0A 보고가
 * 진짜 0A 인지 고장인지 구분할 수 없다.
 *
 * @warning 레벨 시프트 필수. ACS712 는 5V 구동이라 출력이 최대 약 3.4V 까지 올라가고,
 *      ADC 핀에 직결하면 보호 다이오드로 전류가 흘러 핀이 손상된다.
 *      10k+20k 분압으로 x2/3 감쇠 후 입력 -> 복원 배율 3/2, 실효 감도 123.3mV/A.
 */
#include "acs712.h"
#include "hw_adc.h"
#include "dbg.h"

static int32_t s_offset_uv;     /* 무전류 시 ADC 핀 전압 (이상적으로 VCC/2 x 2/3) */
static int32_t s_current_ma;
static bool    s_calibrated;

void acs712_init(void)
{
    s_offset_uv  = 0;
    s_current_ma = 0;
    s_calibrated = false;
}

bool acs712_calibrate(void)
{
    uint32_t sum = 0;
    uint8_t  i;

    if (!hw_adc_is_ready()) {
        DBG_W("acs712 cal skipped (adc not ready)");
        return false;
    }

    for (i = 0; i < CFG_ACS712_CAL_SAMPLES; i++) {
        sum += hw_adc_get_raw(ADC_IDX_ACS712);
        HAL_Delay(2);
    }
    s_offset_uv  = hw_adc_raw_to_uv((uint16_t)(sum / CFG_ACS712_CAL_SAMPLES));
    s_calibrated = true;

    DBG_I("acs712 offset = %ld mV (expect ~1650mV at pin)", (long)(s_offset_uv / 1000));
    return true;
}

void acs712_update(void)
{
    int32_t pin_uv, d_uv, sens_uv;

    if (!s_calibrated) {
        s_current_ma = 0;
        return;
    }

    pin_uv = hw_adc_raw_to_uv(hw_adc_get_raw(ADC_IDX_ACS712));
    d_uv   = pin_uv - s_offset_uv;

    /* 센서 원래 전압으로 복원 : d_sensor = d_pin * 3 / 2 */
    sens_uv = (d_uv * CFG_ACS712_DIV_NUM) / CFG_ACS712_DIV_DEN;

    /* I[mA] = dV[uV] / 감도[uV/mA].
     * (sens_uv * 1000) / 185000 으로 쓰면 int32 오버플로가 난다 — 정상 +-5A 구간은
     * 괜찮지만 ACS712 미연결/무전원이면 핀이 레일에 붙어 sens_uv 가 -2,500,000 까지 가고,
     * 곱셈이 넘쳐 엉뚱한 양수가 나온다. 하필 SENSOR_ERR 이 잡아야 할 고장 모드가
     * 조용히 정상처럼 보이는 것이다. uV/A 를 1000 으로 나눠 두면 곱셈 자체가 없어진다. */
    s_current_ma = DIV_ROUND(sens_uv, CFG_ACS712_SENS_UV_PER_A / 1000L);
}

int32_t acs712_get_ma(void)
{
    return s_current_ma;
}

int32_t acs712_get_offset_uv(void)
{
    return s_offset_uv;
}
