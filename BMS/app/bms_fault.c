/**
 * @file    bms_fault.c
 * @brief   Fault 판단 (진입 즉시 / 해제 지연 = 비대칭 설계)
 *
 * @details 설계 원칙
 *   "안전이 확실할 때만 열고, 불확실하면 기본 닫힘(default closed)"
 *
 *   1) 진입 : 임계 초과가 CONFIRM_CNT(3회 = 300ms) 연속되면 Fault 확정.
 *      1회만 보고 바로 확정하면 ADC 노이즈 1회에 충전이 끊긴다.
 *      3회 연속을 요구하되, 요구 사양(100ms 내 반영)을 만족하도록
 *      데모에서는 1회로 낮출 수 있게 상수화했다.
 *
 *   2) 해제 : 히스테리시스 밴드 아래로 내려간 뒤
 *      CFG_FAULT_CLEAR_HOLD_MS(3초) 유지되어야 해제.
 *      임계 근처에서 Fault 가 on/off 를 반복하는 채터링(chattering)을 막는다.
 *
 *      예) OV 진입 4200mV / 해제 4150mV
 *          -> 4195~4205mV 를 오가도 한 번 걸린 Fault 는 유지된다.
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

/* 과온 임계만 런타임 변경 대상이다 (CAN 0x205).
 * 매크로가 아니라 변수여야 원격에서 바꿀 수 있으므로 RAM 으로 승격한다.
 * 나머지 임계값은 여전히 컴파일 타임 상수다. */
static int32_t     s_th_over_temp_c10     = CFG_OVER_TEMP_C10;
static int32_t     s_th_over_temp_clr_c10 = CFG_OVER_TEMP_CLR_C10;

/**
 * @brief  개별 Fault 상태 갱신
 * @param  bit_pos : s_ctx 인덱스
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
        /* --- 이미 Fault 상태 : 해제 판단 --- */
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
        /* --- 정상 상태 : 진입 판단 --- */
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
 * @note   원격 명령이 배터리 보호를 무력화하지 못하도록 하드코딩된
 *         안전 범위로 강제 클램프한다. 요청값을 그대로 반영하지 않고
 *         "적용된 값"을 돌려주므로, 상위 노드는 응답만 보고
 *         클램프가 걸렸는지 알 수 있다.
 */
int32_t bms_fault_set_ot_threshold(int32_t c10)
{
    s_th_over_temp_c10     = CLAMP(c10, CFG_OT_LIMIT_MIN_C10, CFG_OT_LIMIT_MAX_C10);
    s_th_over_temp_clr_c10 = s_th_over_temp_c10 - CFG_OT_HYSTERESIS_C10;

    DBG_W("OT threshold -> %ld.%ld C (req %ld.%ld)",
          (long)(s_th_over_temp_c10 / 10), (long)(s_th_over_temp_c10 % 10),
          (long)(c10 / 10), (long)(c10 % 10));
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
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        if (p_d->cell_mv[i] > CFG_CELL_OV_MV)      { ov_set = true; }
        if (p_d->cell_mv[i] >= CFG_CELL_OV_CLR_MV) { ov_clr = false; }
        if (p_d->cell_mv[i] < CFG_CELL_UV_MV)      { uv_set = true; }
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

    /* ---------- 4) 과온 ---------- */
    if (p_d->temp_c10 != BMS_TEMP_INVALID) {
        /* 매크로가 아니라 변수를 쓴다 -> CAN 0x205 로 런타임 변경 가능 */
        fault_eval(4, (p_d->temp_c10 > s_th_over_temp_c10),
                      (p_d->temp_c10 < s_th_over_temp_clr_c10),
                      CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_OVER_TEMP);
    }

    /* ---------- 5) 센서 크로스체크 ---------- */
    diff = ABS_DIFF(p_d->pack_ma, p_d->acs_ma);
    fault_eval(5, (!p_d->sensor_ready) || (diff > CFG_SENSOR_DIFF_MA),
                  (p_d->sensor_ready)  && (diff < (CFG_SENSOR_DIFF_MA / 2)),
                  CFG_FAULT_CLEAR_HOLD_MS, &flt, BMS_FLT_SENSOR_ERR);

    /* ---------- 6) 통신 두절 ----------
     * 해제 대기는 전용 상수를 쓴다. 하트비트 1회의 유효기간이
     * CFG_LINK_TIMEOUT_MS(1초)뿐이라, 공용 3초를 쓰면 해제 조건이
     * 3초 연속 성립하는 일이 없어 LINK_TIMEOUT 이 영구히 남는다. */
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
        /* "무엇이 새로 걸렸고 무엇이 풀렸는지"를 찍는다.
         * 최상위 1개 이름(bms_fault_name(flt))만 찍으면
         * 방금 추가된 비트가 우선순위에 밀려 로그에 안 보인다. */
        uint16_t rise = (uint16_t)(flt & (uint16_t)~s_prev_fault);
        uint16_t fall = (uint16_t)(s_prev_fault & (uint16_t)~flt);

        DBG_W("FAULT 0x%04X -> 0x%04X  set:%s clr:%s permit=%d",
              s_prev_fault, flt,
              bms_fault_name(rise), bms_fault_name(fall),
              (int)p_d->charge_permit);
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
