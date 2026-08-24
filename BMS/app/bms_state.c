/**
 * @file    bms_state.c
 * @brief   BMS 상태머신
 *
 *   [INIT] -> [SELF_CHECK] -> 센서 정상 -> [IDLE]
 *                          -> 센서 이상 -> [FAULT]
 *   [IDLE]         -> permit && link_ok && EVSE 충전 요청 -> [CHARGE_READY]
 *   [CHARGE_READY] -> 충전 전류 감지 -> [CHARGING]   / 요청 취소·링크 끊김 -> [IDLE]
 *   [CHARGING]     -> 요청 취소 또는 전류 소멸 -> [IDLE]
 *   어느 상태든 critical Fault -> [FAULT],  Fault 전부 해제 -> [IDLE]
 *
 * charge_permit 는 FAULT 진입 순간 무조건 0 이다 (entry action 에서 강제).
 * Fault 판단 로직에 버그가 있어도 FSM 이 한 번 더 막는다 —
 * 안전 로직은 한 곳에만 두지 않는다.
 */
#include "bms_state.h"
#include "bms_fault.h"
#include "dbg.h"

#define CHARGE_DETECT_MA        100     /* 이 이상 흐르면 충전 중으로 간주 */
#define CHARGE_STOP_MA          30

static bms_state_t s_state;

static void fsm_enter(bms_data_t *p_d, bms_state_t next)
{
    if (next == s_state) {
        return;
    }

    DBG_I("FSM %s -> %s", bms_fsm_state_name(s_state), bms_fsm_state_name(next));
    s_state = next;

    /* --- entry action --- */
    switch (next) {
    case BMS_ST_FAULT:
    case BMS_ST_INIT:
    case BMS_ST_SELF_CHECK:
    case BMS_ST_IDLE:
        p_d->charge_permit = false;     /* default closed */
        break;
    case BMS_ST_CHARGE_READY:
    case BMS_ST_CHARGING:
        /* permit 은 bms_fault_check() 결과를 그대로 따른다 */
        break;
    default:
        break;
    }
    p_d->state = s_state;
}

void bms_fsm_init(bms_data_t *p_d)
{
    s_state = BMS_ST_INIT;
    p_d->state         = BMS_ST_INIT;
    p_d->charge_permit = false;
}

void bms_fsm_run(bms_data_t *p_d)
{
    bool critical = ((p_d->fault & BMS_FLT_CRITICAL_MASK) != 0U);
    bool link_ok  = ((p_d->fault & BMS_FLT_LINK_TIMEOUT) == 0U);

    /* 하트비트가 온다는 것과 충전을 요청했다는 것은 다른 사실이다.
     * link_ok 만 보면 EVSE 가 "충전 안 함(0x201=0)" 을 보내는 동안에도 permit=1 을 뿌린다.
     * E-Stop 은 그 위에서 무조건 이긴다. */
    bool evse_wants_charge = p_d->evse_charge_req && p_d->evse_connected &&
                             !p_d->evse_estop;

    switch (s_state) {

    case BMS_ST_INIT:
        fsm_enter(p_d, BMS_ST_SELF_CHECK);
        break;

    case BMS_ST_SELF_CHECK:
        if (!p_d->sensor_ready) {
            fsm_enter(p_d, BMS_ST_FAULT);
        } else {
            uint8_t i;
            bool    range_ok = true;
            for (i = 0; i < BMS_CELL_COUNT; i++) {
                if ((p_d->cell_mv[i] < CFG_SELFCHK_CELL_MIN_MV) ||
                    (p_d->cell_mv[i] > CFG_SELFCHK_CELL_MAX_MV)) {
                    range_ok = false;
                }
            }
            if (!range_ok) {
                DBG_E("self check: cell voltage out of range (wiring?)");
                fsm_enter(p_d, BMS_ST_FAULT);
            } else {
                fsm_enter(p_d, BMS_ST_IDLE);
            }
        }
        break;

    case BMS_ST_IDLE:
        if (critical) {
            fsm_enter(p_d, BMS_ST_FAULT);
        } else if (p_d->charge_permit && link_ok && evse_wants_charge) {
            fsm_enter(p_d, BMS_ST_CHARGE_READY);
        }
        break;

    case BMS_ST_CHARGE_READY:
        if (critical) {
            fsm_enter(p_d, BMS_ST_FAULT);
        } else if (p_d->pack_ma > CHARGE_DETECT_MA) {
            fsm_enter(p_d, BMS_ST_CHARGING);
        } else if (!link_ok || !evse_wants_charge) {
            fsm_enter(p_d, BMS_ST_IDLE);
        }
        break;

    case BMS_ST_CHARGING:
        if (critical) {
            fsm_enter(p_d, BMS_ST_FAULT);
        } else if (!evse_wants_charge) {
            /* 중지 요청 또는 비상정지. 전류가 남아 있어도 즉시 IDLE 로 내려간다 —
             * CHARGE_STOP_MA 아래로 떨어지기를 기다리면 그동안 릴레이가 붙어 있게 된다. */
            fsm_enter(p_d, BMS_ST_IDLE);
        } else if (p_d->pack_ma < CHARGE_STOP_MA) {
            fsm_enter(p_d, BMS_ST_IDLE);
        }
        break;

    case BMS_ST_FAULT:
        p_d->charge_permit = false;     /* 상태 유지 중에도 매 주기 강제 */
        if (!critical) {
            fsm_enter(p_d, BMS_ST_IDLE);
        }
        break;

    default:
        fsm_enter(p_d, BMS_ST_FAULT);
        break;
    }

    p_d->state = s_state;
}

const char *bms_fsm_state_name(bms_state_t st)
{
    static const char *name[BMS_ST_MAX] = {
        "INIT", "SELF_CHK", "IDLE", "CHG_RDY", "CHARGING", "FAULT"
    };
    return (st < BMS_ST_MAX) ? name[st] : "?";
}
