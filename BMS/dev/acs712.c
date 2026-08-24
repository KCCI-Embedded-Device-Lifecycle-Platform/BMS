/**
 * @file    acs712.c
 * @brief   ACS712-05B 홀 효과 전류 센서 (팩 센서 크로스체크용)
 *
 * 전류를 두 번 재는 것은 중복이 아니다. 팩 센서는 션트(전기적), ACS712 는 홀(자기적)로
 * 원리가 달라서, 둘의 차이가 SENSOR_ERR 의 유일한 근거다. 센서가 하나뿐이면 0A 보고가
 * 진짜 0A 인지 고장인지 구분할 수 없다.
 *
 * @warning **실물은 OUT 을 PC2 에 직결했다 (레벨시프트 없음).** 원 설계는 10k+20k 로
 *      x2/3 감쇠 후 입력하는 것이었으나 분압기 없이 조립이 끝나 되돌릴 수 없다.
 *      -> 복원 배율은 CFG_ACS712_DIV_NUM/DEN = 1/1 이고 실효 감도는 185mV/A 그대로다.
 *      감쇠가 없는데 3/2 로 복원하면 전류가 1.5배로 읽히고, 그 오차가
 *      |acs_ma - pack_ma| 에 실려 멀쩡한 상태에서 SENSOR_ERR 을 띄운다.
 *      배율은 실물을 바꾸는 날에만 같이 바꿀 것 — 근거는 bms_cfg.h 의 정의부에 있다.
 */
#include "acs712.h"
#include "hw_adc.h"
#include "dbg.h"

static int32_t s_offset_uv;     /* 무전류 시 ADC 핀 전압 (직결이므로 이상적으로 VCC/2) */
static int32_t s_current_ma;
static bool    s_calibrated;
static bool    s_ok;

void acs712_init(void)
{
    s_offset_uv  = 0;
    s_current_ma = 0;
    s_calibrated = false;
    s_ok         = false;
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
    s_offset_uv = hw_adc_raw_to_uv((uint16_t)(sum / CFG_ACS712_CAL_SAMPLES));

    if (ABS_DIFF(s_offset_uv / 1000L, CFG_ACS712_OFFSET_EXPECT_MV) >
        CFG_ACS712_OFFSET_TOL_MV) {
        s_calibrated = false;
        s_ok         = false;
        DBG_E("acs712 offset %ldmV out of range (expect %ld+-%ldmV)",
              (long)(s_offset_uv / 1000L), (long)CFG_ACS712_OFFSET_EXPECT_MV,
              (long)CFG_ACS712_OFFSET_TOL_MV);
        return false;
    }

    s_calibrated = true;
    s_ok         = true;

    DBG_I("acs712 offset = %ld mV (expect ~%ldmV at pin, VCC/2 직결)",
          (long)(s_offset_uv / 1000), (long)CFG_ACS712_OFFSET_EXPECT_MV);
    return true;
}

void acs712_update(void)
{
    int32_t pin_uv, d_uv, sens_uv;
    uint16_t raw;

    if (!s_calibrated) {
        s_current_ma = 0;
        s_ok = false;
        return;
    }

    raw = hw_adc_get_raw(ADC_IDX_ACS712);
    if ((raw <= 4U) || (raw >= (uint16_t)(CFG_ADC_FULL_SCALE - 4L))) {
        s_current_ma = 0;
        s_ok = false;
        return;
    }

    pin_uv = hw_adc_raw_to_uv(raw);
    d_uv   = pin_uv - s_offset_uv;

    /* 센서 원래 전압으로 복원. 직결이라 지금은 1/1 = 항등이지만 식을 남겨 둔다 —
     * 분압기를 다는 날 고칠 곳이 bms_cfg.h 한 곳으로 유지되는 것이 이 나눗셈의 값이다. */
    sens_uv = (d_uv * CFG_ACS712_DIV_NUM) / CFG_ACS712_DIV_DEN;

    /* I[mA] = dV[uV] / 감도[uV/mA].
     * (sens_uv * 1000) / 185000 으로 쓰면 int32 오버플로가 난다 — 정상 +-5A 구간은
     * 괜찮지만 ACS712 미연결/무전원이면 핀이 레일에 붙어 sens_uv 가 -2,500,000 까지 가고,
     * 곱셈이 넘쳐 엉뚱한 양수가 나온다. 하필 SENSOR_ERR 이 잡아야 할 고장 모드가
     * 조용히 정상처럼 보이는 것이다. uV/A 를 1000 으로 나눠 두면 곱셈 자체가 없어진다. */
    s_current_ma = DIV_ROUND(sens_uv, CFG_ACS712_SENS_UV_PER_A / 1000L);
    s_ok = true;
}

int32_t acs712_get_ma(void)
{
    return s_current_ma;
}

int32_t acs712_get_offset_uv(void)
{
    return s_offset_uv;
}

bool acs712_is_ok(void)
{
    return s_ok;
}
