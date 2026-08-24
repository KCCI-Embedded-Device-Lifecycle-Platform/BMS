/**
 * @file    bms_soc.c
 * @brief   OCV 초기값 + 전류 적분(쿨롱 카운팅) 기반 SOC 추정
 *
 * 부팅 직후에는 셀 평균 OCV 로 초기 SOC 를 잡는다. 이후 전류가 흐를 때는
 * 팩 전류(+충전/-방전)를 적분하므로 충전 중 단자 전압 변동 때문에 SOC 가
 * 반대로 움직이지 않는다. 무부하에서는 OCV 쪽으로 천천히 재보정한다.
 */
#include "bms_soc.h"
#include "dbg.h"

#define SOC_LUT_SIZE        11U
#define SOC_IIR_SHIFT       3U      /* 나누기 8 */
#define SOC_X100_MAX        10000L
#define SECONDS_PER_HOUR    3600LL

/* 셀 평균 전압[mV] -> SOC[%] (Li-ion 대표 곡선, 10% 간격) */
static const uint16_t s_ocv_mv[SOC_LUT_SIZE] = {
    3000, 3300, 3500, 3600, 3680, 3740, 3800, 3880, 3960, 4060, 4200
};

static int32_t s_soc_x100;          /* 내부는 0.01% 해상도로 보관 */
static int64_t s_coulomb_rem;       /* 0.01% 미만 적분 나머지 */
static bool    s_first;

_Static_assert(CFG_BATTERY_CAPACITY_MAH > 0L,
               "CFG_BATTERY_CAPACITY_MAH must be positive");

void bms_soc_init(void)
{
    s_soc_x100 = 0;
    s_coulomb_rem = 0;
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
    int32_t ocv_x100;

    for (i = 0; i < BMS_CELL_COUNT; i++) {
        sum += p_d->cell_mv[i];
    }
    ocv_x100 = (int32_t)soc_from_ocv(sum / (int32_t)BMS_CELL_COUNT) * 100;

    if (s_first) {
        s_soc_x100 = ocv_x100;      /* 첫 샘플은 OCV 그대로 (초기값 대기 방지) */
        s_first    = false;
    } else if ((p_d->pack_ma > CFG_SOC_REST_CURRENT_MA) ||
               (p_d->pack_ma < -CFG_SOC_REST_CURRENT_MA)) {
        /* bms_soc_update() 주기는 1초다. SOC 0.01% 단위 변화량은
         * I[mA] * 1s * 10000 / (capacity[mAh] * 3600s) 이다.
         * 정수 나눗셈에서 버려지는 몫은 다음 주기에 이어서 누적한다. */
        const int64_t denominator =
            (int64_t)CFG_BATTERY_CAPACITY_MAH * SECONDS_PER_HOUR;
        const int64_t numerator =
            s_coulomb_rem + ((int64_t)p_d->pack_ma * 10000LL);
        const int32_t delta_x100 = (int32_t)(numerator / denominator);

        s_coulomb_rem = numerator - ((int64_t)delta_x100 * denominator);
        s_soc_x100 += delta_x100;
        if (s_soc_x100 <= 0) {
            s_soc_x100 = 0;
            s_coulomb_rem = 0;
        } else if (s_soc_x100 >= SOC_X100_MAX) {
            s_soc_x100 = SOC_X100_MAX;
            s_coulomb_rem = 0;
        }
    } else {
        s_coulomb_rem = 0;
        s_soc_x100 += (ocv_x100 - s_soc_x100) >> SOC_IIR_SHIFT;
    }

    p_d->soc = (uint8_t)CLAMP(DIV_ROUND(s_soc_x100, 100), 0, 100);
}
