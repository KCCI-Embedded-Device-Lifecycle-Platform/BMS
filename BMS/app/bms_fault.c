/**
 * @file    bms_fault.c
 * @brief   Fault 판단 — 진입은 빠르게, 해제는 느리게 (비대칭)
 *
 * 원칙: 안전이 확실할 때만 열고, 불확실하면 닫는다(default closed).
 *   진입 : 임계 초과가 CONFIRM_CNT(3회=300ms) 연속되어야 확정. ADC 노이즈 1회로
 *          충전이 끊기는 것을 막는다.
 *   해제 : 히스테리시스 밴드 아래로 내려간 뒤 CLEAR_HOLD_MS(3초) 유지되어야 해제.
 *          임계 근처에서 on/off 를 반복하는 채터링을 막는다.
 *          (예: OV 진입 4200 / 해제 4150 -> 4195~4205 를 오가도 Fault 는 유지)
 */
#include "bms_fault.h"
#include "hw_tick.h"
#include "dbg.h"

#define FAULT_CONFIRM_CNT   CFG_FAULT_CONFIRM_CNT   /* bms_cfg.h : 기본 3 = 300ms */

typedef struct {
    uint8_t  cnt;                   /* 연속 초과 횟수 */
    uint32_t clear_start;           /* 해제 조건 만족 시작 시각 */
    bool     clear_wait;
} fault_ctx_t;

static fault_ctx_t s_ctx[8];        /* 비트 위치별 컨텍스트 */
static uint32_t    s_link_last_ms;
static uint16_t    s_prev_fault;

/* 이번 판정에서 임계를 넘긴 첫 셀 인덱스 (0xFF = 없음). 로그 전용. */
static uint8_t     s_ov_idx = 0xFFU;
static uint8_t     s_uv_idx = 0xFFU;

/* 과온 임계만 런타임 변경 대상(CAN 0x205)이라 RAM 으로 승격한다.
 * 나머지 임계값은 여전히 컴파일 타임 상수다. */
static int32_t     s_th_over_temp_c10     = CFG_OVER_TEMP_C10;
static int32_t     s_th_over_temp_clr_c10 = CFG_OVER_TEMP_CLR_C10;

/**
 * @brief  개별 Fault 상태 갱신
 * @param  bit_pos    : s_ctx 인덱스
 * @param  set_cond   : 진입 조건 (true 면 이상)
 * @param  clear_cond : 해제 조건 (true 면 정상 복귀 가능)
 * @param  hold_ms    : 해제 조건을 유지해야 하는 시간
 * @param  p_flt      : Fault 비트마스크 (in/out)
 * @param  mask       : 해당 Fault 비트
 */
static void fault_eval(uint8_t bit_pos, bool set_cond, bool clear_cond,
                       uint32_t hold_ms, uint16_t *p_flt, uint16_t mask)
{
    fault_ctx_t *c = &s_ctx[bit_pos];

    if (FLAG_IS_SET(*p_flt, mask)) {
        /* --- Fault 중 : 해제 판단 --- */
        if (clear_cond) {
            if (!c->clear_wait) {
                c->clear_wait  = true;
                c->clear_start = hw_tick_ms();
            } else if (hw_tick_elapsed(c->clear_start, hold_ms)) {
                FLAG_CLR(*p_flt, mask);
                c->clear_wait = false;
                c->cnt        = 0;
            }
        } else {
            c->clear_wait = false;      /* 다시 초과 -> 대기 리셋 */
        }
    } else {
        /* --- 정상 : 진입 판단 --- */
        if (set_cond) {
            if (c->cnt < FAULT_CONFIRM_CNT) {
                c->cnt++;
            }
            if (c->cnt >= FAULT_CONFIRM_CNT) {
                FLAG_SET(*p_flt, mask);
                c->clear_wait = false;
            }
        } else {
            c->cnt = 0;
        }
    }
}

void bms_fault_init(void)
{
    memset(s_ctx, 0, sizeof(s_ctx));
    s_link_last_ms = hw_tick_ms();
    s_prev_fault   = BMS_FLT_NONE;
}

void bms_fault_notify_link(void)
{
    s_link_last_ms = hw_tick_ms();
}

int32_t bms_fault_get_ot_threshold(void)
{
    return s_th_over_temp_c10;
}

/**
 * @brief  과온 임계 원격 변경 (CAN 0x205)
 * @param  c10 : 요청 임계값 [0.1C]
 * @retval 실제로 적용된 값 [0.1C]
 * @note   원격 명령이 배터리 보호를 무력화하지 못하도록 안전 범위로 강제 클램프한다.
 *         "적용된 값" 을 돌려주므로 상위 노드는 응답만 보고 클램프를 알 수 있다.
 */
int32_t bms_fault_set_ot_threshold(int32_t c10)
{
    s_th_over_temp_c10     = CLAMP(c10, CFG_OT_LIMIT_MIN_C10, CFG_OT_LIMIT_MAX_C10);
    s_th_over_temp_clr_c10 = s_th_over_temp_c10 - CFG_OT_HYSTERESIS_C10;

    DBG_W("OT threshold -> %s C (req %s)",
          dbg_temp(s_th_over_temp_c10), dbg_temp(c10));
    return s_th_over_temp_c10;
}

void bms_fault_check(bms_data_t *p_d)
{
    uint8_t  i;
    bool     ov_set = false, ov_clr = true;
    bool     uv_set = false, uv_clr = true;
    int32_t  diff;
    uint16_t flt = p_d->fault;

    /* ---------- 1) 셀 과전압 / 저전압 ---------- */
    /* 어느 셀이 넘겼는지를 같이 남긴다. "CELL_OV" 라는 이름만으로는 4개 중 무엇인지
     * 알 수 없고, 셀1 은 차분이 아니라 node1 그 자체라 원인 계통이 아예 다르다
     * (셀1 만 넘으면 기준전위 이동, 셀2~4 면 실제 셀 전압). */
    s_ov_idx = 0xFFU;
    s_uv_idx = 0xFFU;
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        if (p_d->cell_mv[i] > CFG_CELL_OV_MV)      { ov_set = true;
                                                     if (s_ov_idx == 0xFFU) { s_ov_idx = i; } }
        if (p_d->cell_mv[i] >= CFG_CELL_OV_CLR_MV) { ov_clr = false; }
        if (p_d->cell_mv[i] < CFG_CELL_UV_MV)      { uv_set = true;
                                                     if (s_uv_idx == 0xFFU) { s_uv_idx = i; } }
        if (p_d->cell_mv[i] <= CFG_CELL_UV_CLR_MV) { uv_clr = false; }
    }
    fault_eval(0, ov_set, ov_clr, CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_CELL_OV);
    fault_eval(1, uv_set, uv_clr, CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_CELL_UV);

    /* ---------- 2) 팩 과전압 ---------- */
    fault_eval(2, (p_d->pack_mv > CFG_PACK_OV_MV),
                  (p_d->pack_mv < CFG_PACK_OV_CLR_MV),
                  CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_PACK_OV);

    /* ---------- 3) 과전류 (절대값 기준) ---------- */
    {
        int32_t ia = (p_d->pack_ma < 0) ? -p_d->pack_ma : p_d->pack_ma;
        fault_eval(3, (ia > CFG_OVER_CURRENT_MA),
                      (ia < (CFG_OVER_CURRENT_MA - 100)),
                      CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_OVER_CURRENT);
    }

    /* ---------- 4) 과온 (매크로가 아닌 변수 -> 0x205 로 런타임 변경 가능) ---------- */
    if (p_d->temp_c10 != BMS_TEMP_INVALID) {
        fault_eval(4, (p_d->temp_c10 > s_th_over_temp_c10),
                      (p_d->temp_c10 < s_th_over_temp_clr_c10),
                      CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_OVER_TEMP);
    }

    /* ---------- 5) 센서 크로스체크 (션트 vs 홀) ---------- */
    diff = ABS_DIFF(p_d->pack_ma, p_d->acs_ma);
    /* INA226 포화 구간에서는 pack_ma 자체가 ACS712로 대체되므로 두 센서의
     * 불일치 비교는 의미가 없다. 포화는 정상적인 역할 전환이고 과전류는 위에서 잡는다. */
    fault_eval(5, (!p_d->sensor_ready) ||
                  (!p_d->pack_current_saturated && (diff > CFG_SENSOR_DIFF_MA)),
                  (p_d->sensor_ready) &&
                  (p_d->pack_current_saturated || (diff < (CFG_SENSOR_DIFF_MA / 2))),
                  CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_SENSOR_ERR);

    /* ---------- 6) 통신 두절 ----------
     * 해제 대기는 전용 상수를 쓴다. 하트비트 1회의 유효기간이 1초뿐이라 공용 3초를 쓰면
     * 해제 조건이 3초 연속 성립할 일이 없어 LINK_TIMEOUT 이 영구히 남는다. */
    {
        bool to = hw_tick_elapsed(s_link_last_ms, CFG_LINK_TIMEOUT_MS);
        fault_eval(6, to, !to, CFG_LINK_CLEAR_HOLD_MS, &flt, BMS_FLT_LINK_TIMEOUT);
    }

    /* ---------- 7) 셀 불균형 (Warning, 즉시 반영) ---------- */
    if (p_d->imbalance_mv > CFG_IMBALANCE_MV) {
        FLAG_SET(flt, BMS_FLT_IMBALANCE);
    } else {
        FLAG_CLR(flt, BMS_FLT_IMBALANCE);
    }

    /* ---------- 8) 충전 허가 결정 ---------- */
    p_d->fault         = flt;
    p_d->charge_permit = ((flt & BMS_FLT_CRITICAL_MASK) == 0U);

    if (flt != s_prev_fault) {
        /* 최상위 1개만 찍으면 방금 추가된 비트가 우선순위에 밀려 안 보인다.
         * 그래서 새로 걸린 것(set)과 풀린 것(clr)을 나눠서 찍는다. */
        uint16_t rise = (uint16_t)(flt & (uint16_t)~s_prev_fault);
        uint16_t fall = (uint16_t)(s_prev_fault & (uint16_t)~flt);

        DBG_W("FAULT 0x%04X -> 0x%04X  set:%s clr:%s permit=%d",
              s_prev_fault, flt,
              bms_fault_name(rise), bms_fault_name(fall),
              (int)p_d->charge_permit);

        /* --- 셀 Fault 가 새로 걸리면 판정에 쓴 원값을 그 자리에서 남긴다 ---
         * 1초 요약 로그로는 이걸 절대 못 본다. 트립하면 FSM 이 같은 슬롯에서 릴레이를
         * 다시 열고, 열리면 값이 원래대로 돌아와서 요약에는 "임계 이하" 만 남는다.
         * ("CELL_OV 인데 로그의 셀 전압은 4200 미만" 으로 보이는 이유가 이것이다)
         *
         * node 를 같이 찍는 이유: cell1 = node1 그 자체(GND 기준)이고 cell2~4 만
         * 차분이다. 기준전위가 밀리면 셀1 에만 통째로 실리고 나머지는 상쇄된다. */
        if (FLAG_IS_SET(rise, (uint16_t)(BMS_FLT_CELL_OV | BMS_FLT_CELL_UV))) {
            uint8_t idx = (s_ov_idx != 0xFFU) ? s_ov_idx : s_uv_idx;

            DBG_W("  >> cell%u 가 임계를 넘겼다 (OV>%d / UV<%d)",
                  (unsigned)(idx + 1U), CFG_CELL_OV_MV, CFG_CELL_UV_MV);
            DBG_W("  cell mV: %ld / %ld / %ld / %ld",
                  (long)p_d->cell_mv[0], (long)p_d->cell_mv[1],
                  (long)p_d->cell_mv[2], (long)p_d->cell_mv[3]);
            DBG_W("  node mV: %ld / %ld / %ld / %ld   (VDDA %ld mV)",
                  (long)p_d->node_mv[0], (long)p_d->node_mv[1],
                  (long)p_d->node_mv[2], (long)p_d->node_mv[3],
                  (long)p_d->vdda_mv);
            if (idx == 0U) {
                DBG_W("  cell1 은 차분이 아니다 -> 접지/기준전위 이동을 먼저 의심할 것");
            }
        }
        s_prev_fault = flt;
    }
}

const char *bms_fault_name(uint16_t flt)
{
    if (FLAG_IS_SET(flt, BMS_FLT_CELL_OV))       { return "CELL_OV"; }
    if (FLAG_IS_SET(flt, BMS_FLT_CELL_UV))       { return "CELL_UV"; }
    if (FLAG_IS_SET(flt, BMS_FLT_PACK_OV))       { return "PACK_OV"; }
    if (FLAG_IS_SET(flt, BMS_FLT_OVER_CURRENT))  { return "OVER_CUR"; }
    if (FLAG_IS_SET(flt, BMS_FLT_OVER_TEMP))     { return "OVER_TMP"; }
    if (FLAG_IS_SET(flt, BMS_FLT_SENSOR_ERR))    { return "SENSOR"; }
    if (FLAG_IS_SET(flt, BMS_FLT_LINK_TIMEOUT))  { return "LINK_TO"; }
    if (FLAG_IS_SET(flt, BMS_FLT_IMBALANCE))     { return "IMBAL_W"; }
    return "NONE";
}
