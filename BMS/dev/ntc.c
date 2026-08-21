/**
 * @file    ntc.c
 * @brief   NTC 10K B3950 온도 변환 (LUT + 선형보간, 부동소수점 미사용)
 *
 *      3.3V ---[10k 풀업]---+---[NTC]--- GND      R_ntc = R_pull * V / (VDDA - V)
 *                           |
 *                          ADC
 *
 * @note    베타식 대신 LUT 를 쓰는 이유: 1/T = 1/T0 + (1/B)ln(R/R0) 은 logf() 가 필요한데
 *      newlib 의 logf 는 수 KB 플래시를 잡아먹고 연산도 무겁다. 온도는 급변하지 않고
 *      0.1도 분해능이면 충분하므로 5도 간격 LUT + 선형보간으로 충분하다
 *      (오차 0.2도 이내, 연산은 비교/뺄셈/곱셈뿐).
 */
#include "ntc.h"
#include "hw_adc.h"
#include "dbg.h"

#define NTC_CH_COUNT        4U
#define NTC_LUT_SIZE        19U
#define NTC_LUT_START_C10   (-100)      /* -10.0 C */
#define NTC_LUT_STEP_C10    50          /*   5.0 C */

/* 저항[ohm] : -10C 부터 +80C 까지 5도 간격 (B=3950, R25=10k) */
static const uint16_t s_lut_ohm[NTC_LUT_SIZE] = {
    58280, 44034, 33626, 25928, 20177,  /* -10 ~ 10 */
    15840, 12537, 10000,  8037,  6507,  /*  15 ~ 35 */
     5302,  4349,  3588,  2979,  2486,  /*  40 ~ 60 */
     2086,  1760,  1492,  1270           /*  65 ~ 80 */
};

static const adc_idx_t s_ch[NTC_CH_COUNT] = {
    ADC_IDX_NTC1, ADC_IDX_NTC2, ADC_IDX_NTC3, ADC_IDX_NTC4
};
static int32_t s_temp_c10[NTC_CH_COUNT];
static int32_t s_res_ohm[NTC_CH_COUNT];

/**
 * @brief 저항 -> 온도(0.1C) 변환. LUT 는 저항 내림차순이다.
 */
static int32_t ntc_res_to_c10(int32_t r_ohm)
{
    uint8_t i;
    int32_t r_hi, r_lo, t_lo;

    if (r_ohm >= (int32_t)s_lut_ohm[0]) {
        return NTC_LUT_START_C10;                       /* 범위 하한 클램프 */
    }
    if (r_ohm <= (int32_t)s_lut_ohm[NTC_LUT_SIZE - 1]) {
        return NTC_LUT_START_C10 + (int32_t)(NTC_LUT_SIZE - 1) * NTC_LUT_STEP_C10;
    }

    for (i = 1; i < NTC_LUT_SIZE; i++) {
        if (r_ohm > (int32_t)s_lut_ohm[i]) {
            r_hi = (int32_t)s_lut_ohm[i - 1];           /* 저항 큰 쪽 = 온도 낮은 쪽 */
            r_lo = (int32_t)s_lut_ohm[i];
            t_lo = NTC_LUT_START_C10 + (int32_t)(i - 1) * NTC_LUT_STEP_C10;

            /* 선형보간 : 저항이 r_hi->r_lo 로 줄어드는 만큼 온도가 5도 오른다 */
            return t_lo + DIV_ROUND((r_hi - r_ohm) * NTC_LUT_STEP_C10, (r_hi - r_lo));
        }
    }
    return NTC_TEMP_INVALID;
}

void ntc_init(void)
{
    uint8_t i;
    for (i = 0; i < NTC_CH_COUNT; i++) {
        s_temp_c10[i] = NTC_TEMP_INVALID;
        s_res_ohm[i]  = 0;
    }
    DBG_I("ntc init (%uch, 10k pullup, B3950 LUT)", (unsigned)NTC_CH_COUNT);
}

void ntc_update(void)
{
    uint8_t i;
    int32_t v_mv, vdda_mv, r;

    vdda_mv = hw_adc_get_vdda_mv();

    for (i = 0; i < NTC_CH_COUNT; i++) {
        v_mv = DIV_ROUND(hw_adc_raw_to_uv(hw_adc_get_raw(s_ch[i])), 1000L);

        /* 개방(NTC 미연결) -> v ~ VDDA, 단락 -> v ~ 0 : 둘 다 오류로 처리 */
        if ((v_mv <= 20) || (v_mv >= (vdda_mv - 20))) {
            s_res_ohm[i]  = 0;
            s_temp_c10[i] = NTC_TEMP_INVALID;
            continue;
        }

        /* R_ntc = R_pull * V / (VDDA - V) */
        r = DIV_ROUND(CFG_NTC_PULLUP_OHM * v_mv, (vdda_mv - v_mv));

        s_res_ohm[i]  = r;
        s_temp_c10[i] = ntc_res_to_c10(r);
    }
}

int32_t ntc_get_temp_c10(uint8_t idx)
{
    return (idx < NTC_CH_COUNT) ? s_temp_c10[idx] : NTC_TEMP_INVALID;
}

int32_t ntc_get_res_ohm(uint8_t idx)
{
    return (idx < NTC_CH_COUNT) ? s_res_ohm[idx] : 0;
}

/**
 * @brief  유효 채널의 최댓값(가장 뜨거운 곳)을 돌려준다.
 *
 * @note   블랙보드 bms_data_t.temp_c10 이 하나뿐이라 4채널을 하나로 줄여야 한다.
 *     평균이 아니라 최댓값인 이유: 평균은 한 셀만 뜨거운 국부 발열을 희석시켜
 *     과온 임계를 못 넘게 만든다. 보호는 항상 최악값 기준이어야 한다.
 *     단선/단락 채널은 건너뛰므로, 센서 하나가 빠져도 나머지로 보호가 계속된다.
 */
int32_t ntc_get_temp_max_c10(void)
{
    uint8_t i;
    int32_t max = NTC_TEMP_INVALID;

    for (i = 0; i < NTC_CH_COUNT; i++) {
        if (s_temp_c10[i] == NTC_TEMP_INVALID) {
            continue;
        }
        if ((max == NTC_TEMP_INVALID) || (s_temp_c10[i] > max)) {
            max = s_temp_c10[i];
        }
    }
    return max;
}

uint8_t ntc_get_valid_count(void)
{
    uint8_t i, n = 0;

    for (i = 0; i < NTC_CH_COUNT; i++) {
        if (s_temp_c10[i] != NTC_TEMP_INVALID) {
            n++;
        }
    }
    return n;
}
