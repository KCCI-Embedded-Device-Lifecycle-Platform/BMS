/**
 * @file    cell_adc.c
 * @brief   저항 분압 기반 셀 전압 측정
 *
 * @details 측정 원리
 *      B+ (16.8V) ---[100k]---+---[20k]--- GND
 *                             |
 *                            ADC (max 2.80V)
 *
 *      노드 전압 = ADC 전압 x (100k + 20k) / 20k = x6.0
 *      셀 전압   = 인접 노드의 차분
 *
 * @warning 차분 오차 증폭
 *      Cell4 = Node4 - Node3 = 16.8V - 12.6V 처럼
 *      "큰 값에서 큰 값을 빼서 작은 값을 얻는" 연산이다.
 *      각 노드에 1% 오차가 있으면 16.8V x 1% = 168mV 오차가
 *      그대로 4.2V 짜리 셀 전압에 실린다 (상대오차 4%).
 *      과전압 임계(4.20 vs 4.25V, 50mV 차이)를 판별할 수 없다.
 *
 *      대응 : (1) 0.1% 저항 사용  (2) 채널별 2점 캘리브레이션
 *      -> gain_q16 / offset_mv 를 두어 런타임 보정이 가능하도록 설계
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

        /* 2) 분압 복원
         *    먼저 나누면(pin_uv/1000) 1mV 절삭 오차가 x6 으로 증폭되어 6mV 가 된다.
         *    반드시 "곱한 뒤 나눈다". 중간값이 int32(21억)를 넘으므로 int64 로 승격.
         *    최대 검산: 3,300,000uV x 6000 = 1.98e10  ->  int64 필요 */
        node_uv = (int32_t)(((int64_t)pin_uv * CFG_DIV_SCALE_X1000) / 1000L);

        /* 3) 캘리브레이션 : gain(Q16) 적용 후 offset 보정
         *    (int64 로 승격해 중간 오버플로 차단) */
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
