/**
 * @file    acs712.c
 * @brief   ACS712-05B 홀 효과 전류 센서 (INA219 크로스체크용)
 *
 * @details 왜 전류 센서가 두 개인가?
 *      INA219 는 션트(저항) 방식, ACS712 는 홀 효과 방식으로
 *      물리 원리가 완전히 다르다. 두 값이 크게 어긋나면
 *      "둘 중 하나가 고장났다"는 것을 알 수 있다 (Sensor Error).
 *      단일 센서만 있으면 센서가 0A 를 계속 보고해도
 *      정말 0A 인지 고장인지 구분할 수 없다.
 *      안전 시스템에서 센서 다중화는 기본 요구사항이다.
 *
 * @warning 레벨 시프트 필수
 *      ACS712 는 5V 구동이고 출력이 최대 약 3.4V 까지 올라간다.
 *      STM32 ADC 핀에 직결하면 VDDA+0.3V 를 넘어 보호 다이오드로
 *      전류가 흘러 핀이 손상된다.
 *      10k(상단) + 20k(하단) 분압으로 x2/3 감쇠 후 입력한다.
 *      -> 복원 배율 3/2, 실효 감도 = 185mV/A x 2/3 = 123.3mV/A
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

    /* I[mA] = dV[uV] / 감도[uV/A] * 1000 */
    s_current_ma = DIV_ROUND(sens_uv * 1000L, CFG_ACS712_SENS_UV_PER_A);
}

int32_t acs712_get_ma(void)
{
    return s_current_ma;
}

int32_t acs712_get_offset_uv(void)
{
    return s_offset_uv;
}
