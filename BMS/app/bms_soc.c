/**
 * @file    bms_soc.c
 * @brief   OCV-LUT 기반 SOC 추정 + 1차 IIR 평활
 *
 * 정확도만 보면 쿨롱 카운팅(전류 적분)이 낫지만 적분 오차가 누적되고 초기 SOC 를 모른다.
 * 실제 BMS 는 "쿨롱 카운팅 + 무부하 시 OCV 재보정" 하이브리드를 쓰는데,
 * 여기서는 구조만 잡아 두고 OCV 기반으로 구현한다 (적분은 2단계에 추가).
 *
 * 전압 기반의 한계: 충·방전 중에는 내부저항 강하(I x R) 때문에 단자 전압이 OCV 와 달라
 * SOC 가 출렁인다. -> IIR(1/8)로 평활하고 1초 주기로만 갱신한다.
 */
#include "bms_soc.h"
#include "dbg.h"

#define SOC_LUT_SIZE        11U
#define SOC_IIR_SHIFT       3U      /* 나누기 8 */

/* 셀 평균 전압[mV] -> SOC[%] (Li-ion 대표 곡선, 10% 간격) */
static const uint16_t s_ocv_mv[SOC_LUT_SIZE] = {
    3000, 3300, 3500, 3600, 3680, 3740, 3800, 3880, 3960, 4060, 4200
};

static int32_t s_soc_x100;          /* 내부는 0.01% 해상도로 보관 */
static bool    s_first;

void bms_soc_init(void)
{
    s_soc_x100 = 0;
    s_first    = true;
}

static uint8_t soc_from_ocv(int32_t cell_mv)
{
    uint8_t i;
    int32_t lo, hi;

    if (cell_mv <= (int32_t)s_ocv_mv[0]) {
        return 0;
    }
    if (cell_mv >= (int32_t)s_ocv_mv[SOC_LUT_SIZE - 1]) {
        return 100;
    }

    for (i = 1; i < SOC_LUT_SIZE; i++) {
        if (cell_mv < (int32_t)s_ocv_mv[i]) {
            lo = (int32_t)s_ocv_mv[i - 1];
            hi = (int32_t)s_ocv_mv[i];
            /* 구간당 10% 를 선형 배분 */
            return (uint8_t)(((int32_t)(i - 1) * 10) +
                             DIV_ROUND((cell_mv - lo) * 10, (hi - lo)));
        }
    }
    return 100;
}

void bms_soc_update(bms_data_t *p_d)
{
    int32_t sum = 0;
    uint8_t i;
    int32_t raw_x100;

    for (i = 0; i < BMS_CELL_COUNT; i++) {
        sum += p_d->cell_mv[i];
    }
    raw_x100 = (int32_t)soc_from_ocv(sum / (int32_t)BMS_CELL_COUNT) * 100;

    if (s_first) {
        s_soc_x100 = raw_x100;      /* 첫 샘플은 그대로 (필터 수렴 대기 방지) */
        s_first    = false;
    } else {
        s_soc_x100 += (raw_x100 - s_soc_x100) >> SOC_IIR_SHIFT;
    }

    p_d->soc = (uint8_t)CLAMP(DIV_ROUND(s_soc_x100, 100), 0, 100);
}

uint8_t bms_soc_get(void)
{
    return (uint8_t)CLAMP(DIV_ROUND(s_soc_x100, 100), 0, 100);
}
