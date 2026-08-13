/**
 * @file    cell_adc.c
 * @brief   저항 분압 기반 셀 전압 측정
 *
 *      B+ (16.8V) ---[100k]---+---[20k]--- GND
 *                             |
 *                            ADC (max 2.80V)
 *      노드 전압 = ADC 전압 x (100k+20k)/20k = x6.0,  셀 전압 = 인접 노드의 차분
 *
 * @warning 차분이 오차를 증폭한다. Cell4 = 16.8V - 12.6V 처럼 큰 값끼리 빼서 작은 값을
 *      얻으므로, 노드 1% 오차(16.8V x 1% = 168mV)가 4.2V 짜리 셀에 그대로 실린다.
 *      그 상태로는 OV 임계 4.20 vs 4.25V(50mV)를 구분할 수 없다.
 *      -> 0.1% 저항 + 채널별 캘리브레이션(gain_q16 / off_mv)이 전제다.
 */
#include "cell_adc.h"
#include "hw_adc.h"
#include "dbg.h"

typedef struct {
    int32_t node_mv[BMS_CELL_COUNT];
    int32_t cell_mv[BMS_CELL_COUNT];
    int32_t gain_q16[BMS_CELL_COUNT];
    int32_t off_mv[BMS_CELL_COUNT];
} cell_adc_ctx_t;

static cell_adc_ctx_t s_ctx;

/* ADC 랭크 인덱스 매핑 (하드웨어 배선이 바뀌면 여기만 수정) */
static const adc_idx_t s_node_ch[BMS_CELL_COUNT] = {
    ADC_IDX_NODE_B1, ADC_IDX_NODE_B2, ADC_IDX_NODE_B3, ADC_IDX_NODE_BP
};

void cell_adc_init(void)
{
    uint8_t i;

    memset(&s_ctx, 0, sizeof(s_ctx));
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        s_ctx.gain_q16[i] = 65536;      /* x1.000 : 캘리브레이션 전 기본값 */
        s_ctx.off_mv[i]   = 0;
    }
    DBG_I("cell_adc init (divider x%ld.%03ld)",
          (long)(CFG_DIV_SCALE_X1000 / 1000), (long)(CFG_DIV_SCALE_X1000 % 1000));
}

void cell_adc_update(void)
{
    uint8_t i;
    int32_t pin_uv;
    int32_t node_uv;

    for (i = 0; i < BMS_CELL_COUNT; i++) {
        /* 1) ADC raw -> 핀 전압 [uV] (VDDA 실측 보정 포함) */
        pin_uv = hw_adc_raw_to_uv(hw_adc_get_raw(s_node_ch[i]));

        /* 2) 분압 복원. 먼저 나누면 1mV 절삭 오차가 x6 으로 증폭되므로 "곱한 뒤 나눈다".
         *    중간값 3,300,000uV x 6000 = 1.98e10 이라 int32 를 넘는다 -> int64 승격. */
        node_uv = (int32_t)(((int64_t)pin_uv * CFG_DIV_SCALE_X1000) / 1000L);

        /* 3) gain(Q16) 적용 후 offset 보정 (int64 로 중간 오버플로 차단) */
        node_uv = (int32_t)(((int64_t)node_uv * s_ctx.gain_q16[i]) >> 16);

        s_ctx.node_mv[i] = DIV_ROUND(node_uv, 1000L) + s_ctx.off_mv[i];
    }

    /* 4) 차분으로 셀 전압 산출 */
    s_ctx.cell_mv[0] = s_ctx.node_mv[0];
    for (i = 1; i < BMS_CELL_COUNT; i++) {
        s_ctx.cell_mv[i] = s_ctx.node_mv[i] - s_ctx.node_mv[i - 1];
    }
}

int32_t cell_adc_get_node_mv(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_ctx.node_mv[idx] : 0;
}

int32_t cell_adc_get_cell_mv(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_ctx.cell_mv[idx] : 0;
}

int32_t cell_adc_get_pack_mv(void)
{
    return s_ctx.node_mv[BMS_CELL_COUNT - 1];
}

void cell_adc_set_gain(uint8_t idx, int32_t gain_q16)
{
    if (idx < BMS_CELL_COUNT) {
        s_ctx.gain_q16[idx] = gain_q16;
    }
}

void cell_adc_set_offset(uint8_t idx, int32_t off_mv)
{
    if (idx < BMS_CELL_COUNT) {
        s_ctx.off_mv[idx] = off_mv;
    }
}

int32_t cell_adc_get_gain(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_ctx.gain_q16[idx] : 65536;
}

/* --- 캘리브레이션 가드레일 ---
 * 저항 공차와 ADC 오차를 다 합쳐도 실제 보정폭은 수 % 안쪽이다. ±25% 밖은 보정이 아니라
 * 노드를 잘못 짚었거나 DMM 레인지가 틀렸거나 배선이 안 붙은 것이다.
 * 그대로 받으면 "보정했더니 더 틀어졌다" 가 되어 원인 추적이 어려워진다. */
#define CAL_GAIN_MIN_Q16    49152L      /* x0.75 */
#define CAL_GAIN_MAX_Q16    81920L      /* x1.25 */
#define CAL_MIN_MV          500L        /* 0 근처에서 비율을 잡으면 게인이 발산한다 */

bool cell_adc_calibrate_node(uint8_t idx, int32_t actual_mv)
{
    int32_t measured;
    int64_t g;

    if (idx >= BMS_CELL_COUNT) {
        return false;
    }

    measured = s_ctx.node_mv[idx];
    if ((measured < CAL_MIN_MV) || (actual_mv < CAL_MIN_MV)) {
        return false;
    }

    /* 덮어쓰지 않고 현재 게인에 비율을 곱한다 -> 여러 번 눌러도 누적이 아니라 수렴한다. */
    g = ((int64_t)s_ctx.gain_q16[idx] * (int64_t)actual_mv) / (int64_t)measured;

    if ((g < CAL_GAIN_MIN_Q16) || (g > CAL_GAIN_MAX_Q16)) {
        return false;
    }

    s_ctx.gain_q16[idx] = (int32_t)g;
    return true;
}

void cell_adc_reset_cal(void)
{
    uint8_t i;
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        s_ctx.gain_q16[i] = 65536;
        s_ctx.off_mv[i]   = 0;
    }
}
