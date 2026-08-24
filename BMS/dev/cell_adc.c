/**
 * @file    cell_adc.c
 * @brief   저항 분압 기반 셀 전압 측정
 *
 *      Node_n ---[R_top]---+---[R_bot]--- GND
 *                          |
 *                         ADC        노드 전압 = ADC 전압 x (R_top+R_bot)/R_bot
 *                                    셀 전압   = 인접 노드의 차분
 *
 *      분압비는 노드마다 따로 갖는다 (CFG_DIV_SCALE_X1000_LIST). 4채널이 같은 저항으로
 *      꽂혀 있으리라는 보장이 없고, 한 노드만 달라도 증상이 그 노드에서 끝나지 않기
 *      때문이다 — 아래 @warning 참조.
 *
 * @warning 차분이 오차를 증폭한다. Cell4 = 16.8V - 12.6V 처럼 큰 값끼리 빼서 작은 값을
 *      얻으므로, 노드 1% 오차(16.8V x 1% = 168mV)가 4.2V 짜리 셀에 그대로 실린다.
 *      그 상태로는 OV 임계 4.20 vs 4.25V(50mV)를 구분할 수 없다.
 *      -> 0.1% 저항 + 채널별 게인 캘리브레이션(gain_q16)이 전제다.
 *
 * @warning 노드 하나가 틀리면 셀 두 개가 반대 부호로 틀린다. node3 만 배율이 어긋났을 때
 *      cell3 = +10.4V / cell4 = -2.3V 로 나온 적이 있다 (합은 맞으므로 pack 은 멀쩡해 보인다).
 *      셀 값이 이상하면 셀이 아니라 **노드**를 먼저 볼 것 — 'v' 덤프의 pin uV 가 그 창구다.
 *
 * @note 오프셋 보정은 일부러 두지 않는다. 이 회로의 지배 오차는 분압 저항 공차(곱셈성)라
 *      16.8V 에서 168mV 인데, ADC 오프셋 기여분은 x6 을 먹어도 20mV 수준이다.
 *      2점 보정을 하려면 0V 근처 두 번째 점이 필요하고 스택 분압에서 그건 노드를
 *      단락시켜야 해서 벤치에서 위험하다. 지배 오차만 정확히 잡는 쪽이 낫다.
 */
#include "cell_adc.h"
#include "hw_adc.h"
#include "dbg.h"

typedef struct {
    uint16_t raw[BMS_CELL_COUNT];       /* ADC 원값 (포화 판정에 필요하다) */
    int32_t pin_uv[BMS_CELL_COUNT];     /* 분압 복원 전 = ADC 핀 실측 [uV] */
    int32_t node_mv[BMS_CELL_COUNT];
    int32_t cell_mv[BMS_CELL_COUNT];
    int32_t gain_q16[BMS_CELL_COUNT];
} cell_adc_ctx_t;

static cell_adc_ctx_t s_ctx;

/* ADC 랭크 인덱스 매핑 (하드웨어 배선이 바뀌면 여기만 수정) */
static const adc_idx_t s_node_ch[BMS_CELL_COUNT] = {
    ADC_IDX_NODE_B1, ADC_IDX_NODE_B2, ADC_IDX_NODE_B3, ADC_IDX_NODE_BP
};

/* 노드별 분압 복원 배율 (x1000). 실제로 꽂힌 저항을 적는 자리이고,
 * 그 뒤에 남는 수 % 는 gain_q16 이 맡는다. 두 축을 섞지 않는다. */
static const int32_t s_div_x1000[BMS_CELL_COUNT] = CFG_DIV_SCALE_X1000_LIST;

void cell_adc_init(void)
{
    uint8_t i;

    memset(&s_ctx, 0, sizeof(s_ctx));
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        s_ctx.gain_q16[i] = 65536;      /* x1.000 : 캘리브레이션 전 기본값 */
    }
    /* 노드마다 다를 수 있으므로 4개를 다 찍는다. 배선을 고친 뒤 펌웨어를 안 고쳤을 때
     * 부팅 로그 한 줄로 바로 드러나야 한다. */
    DBG_I("cell_adc init (divider x%ld.%03ld / x%ld.%03ld / x%ld.%03ld / x%ld.%03ld)",
          (long)(s_div_x1000[0] / 1000), (long)(s_div_x1000[0] % 1000),
          (long)(s_div_x1000[1] / 1000), (long)(s_div_x1000[1] % 1000),
          (long)(s_div_x1000[2] / 1000), (long)(s_div_x1000[2] % 1000),
          (long)(s_div_x1000[3] / 1000), (long)(s_div_x1000[3] % 1000));
}

void cell_adc_update(void)
{
    uint8_t i;
    int32_t node_uv;

    for (i = 0; i < BMS_CELL_COUNT; i++) {
        /* 1) ADC raw -> 핀 전압 [uV] (VDDA 실측 보정 포함).
         *    이 값을 남겨 두는 이유: 노드 전압이 이상할 때 "ADC 가 이상한가" 와
         *    "분압비 설정이 실물과 다른가" 를 가르는 유일한 지점이다. */
        s_ctx.raw[i]    = hw_adc_get_raw(s_node_ch[i]);
        s_ctx.pin_uv[i] = hw_adc_raw_to_uv(s_ctx.raw[i]);

        /* 2) 분압 복원. 먼저 나누면 1mV 절삭 오차가 x6 으로 증폭되므로 "곱한 뒤 나눈다".
         *    중간값 3,300,000uV x 6000 = 1.98e10 이라 int32 를 넘는다 -> int64 승격. */
        node_uv = (int32_t)(((int64_t)s_ctx.pin_uv[i] * s_div_x1000[i]) / 1000L);

        /* 3) 채널 게인(Q16) 적용 (int64 로 중간 오버플로 차단) */
        node_uv = (int32_t)(((int64_t)node_uv * s_ctx.gain_q16[i]) >> 16);

        s_ctx.node_mv[i] = DIV_ROUND(node_uv, 1000L);
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

int32_t cell_adc_get_pin_uv(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_ctx.pin_uv[idx] : 0;
}

/* 16회 평균이라 완전 포화라도 정확히 4095 가 아닐 수 있어 몇 LSB 여유를 둔다.
 * 포화는 "값이 조금 틀리다" 가 아니라 "이 채널은 아무 정보도 없다" 이므로,
 * 분압비 역산도 캘리브레이션도 여기서는 전부 의미가 없어진다. */
#define RAIL_MARGIN_LSB     4U

bool cell_adc_is_railed(uint8_t idx)
{
    if (idx >= BMS_CELL_COUNT) {
        return false;
    }
    return (s_ctx.raw[idx] >= (uint16_t)(CFG_ADC_FULL_SCALE - (int32_t)RAIL_MARGIN_LSB));
}

uint16_t cell_adc_get_raw(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_ctx.raw[idx] : 0U;
}

int32_t cell_adc_get_div_x1000(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_div_x1000[idx] : 0;
}

void cell_adc_set_gain(uint8_t idx, int32_t gain_q16)
{
    if (idx < BMS_CELL_COUNT) {
        s_ctx.gain_q16[idx] = gain_q16;
    }
}


int32_t cell_adc_get_gain(uint8_t idx)
{
    return (idx < BMS_CELL_COUNT) ? s_ctx.gain_q16[idx] : 65536;
}

/* --- 캘리브레이션 가드레일 ---
 * 저항 공차와 ADC 오차를 다 합쳐도 실제 보정폭은 수 % 안쪽이다. ±25% 밖은 보정이 아니라
 * 노드를 잘못 짚었거나 DMM 레인지가 틀렸거나 배선/부품이 설정과 다른 것이다.
 * 그대로 받으면 "보정했더니 더 틀어졌다" 가 되어 원인 추적이 어려워진다.
 * -> 거부하는 대신 cell_adc_measure_div_x1000() 으로 "그럼 실제 분압비는 얼마인가" 를
 *    돌려준다. 거부가 막다른 길이 되면 사용자는 결국 가드레일을 넓히려 든다. */
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

int32_t cell_adc_measure_div_x1000(uint8_t idx, int32_t actual_mv)
{
    int32_t pin_uv;

    if (idx >= BMS_CELL_COUNT) {
        return 0;
    }
    if (cell_adc_is_railed(idx)) {
        /* 포화 상태에서는 pin_uv 가 VDDA 에 고정되므로, 여기서 나오는 배율은
         * "실제 분압비" 가 아니라 "VDDA 대비 몇 배인가" 라는 무의미한 수다.
         * 그 수를 설정에 옮겨 적으면 오차가 굳어 버리므로 계산 자체를 거절한다. */
        return 0;
    }
    pin_uv = s_ctx.pin_uv[idx];
    if ((pin_uv <= 0) || (actual_mv < CAL_MIN_MV)) {
        return 0;                       /* 핀이 0V 면 분압비가 아니라 결선 문제다 */
    }

    /* 배율 = 노드[V] / 핀[V] = (actual_mv/1e3) / (pin_uv/1e6).
     * x1000 스케일로 = actual_mv * 1e6 / pin_uv. 12150 * 1e6 = 1.2e10 이라 int64 필수. */
    return (int32_t)(((int64_t)actual_mv * 1000000LL) / (int64_t)pin_uv);
}

void cell_adc_reset_cal(void)
{
    uint8_t i;
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        s_ctx.gain_q16[i] = 65536;
    }
}
