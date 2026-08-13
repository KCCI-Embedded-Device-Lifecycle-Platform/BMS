/**
 * @file    bms_app.c
 * @brief   BMS 최상위 super-loop 스케줄러
 *
 * RTOS 없이 주기 슬롯만 돌린다. 각 태스크가 짧고 블로킹이 없어서 RTOS 오버헤드가
 * 필요 없고, 실행 순서가 고정이라 타이밍 분석이 쉽다.
 *
 *   100ms : 셀 ADC, 팩 센서, Fault, FSM, 릴레이, 0x100/0x103
 *   500ms : NTC, ACS712, OLED, 0x101/0x102
 *   1000ms: SOC, 0x104, 콘솔 요약
 *
 * 측정 -> 판단 -> 송신이 같은 100ms 슬롯 안에서 끝나므로
 * "Fault 후 100ms 이내 permit=0" 요구사항이 성립한다.
 */
#include "bms_app.h"
#include "dbg.h"
#include "hw_tick.h"
#include "hw_adc.h"
#include "hw_i2c.h"
#include "hw_gpio.h"
#include "hw_uart.h"

#include "cell_adc.h"
#include "pack_sensor.h"
#include "ntc.h"
#include "acs712.h"
#include "oled.h"
#include "status_led.h"

#include "bms_fault.h"
#include "bms_soc.h"
#include "bms_state.h"
#include "bms_link.h"
#include "bms_can.h"        /* EVSE 수신 상태를 블랙보드로 옮기기 위해서만 */
#include "bms_ui.h"

/* 전역 블랙보드. static 으로 캡슐화하고 getter 로만 노출한다. */
static bms_data_t s_bms;

static uint32_t s_t100, s_t500, s_t1000;

/* --- Fault 주입 ('f') ---
 * 배터리 없이 Fault/FSM 경로를 검증하기 위한 시연 경로.
 * ap_collect() 맨 끝에서 블랙보드를 덮어쓰므로 실측값을 이긴다.
 * 링크 쪽은 s_link_sim('p')이 따로 담당한다 -> 'f/n' 은 셀/온도만 건드린다. */
typedef enum {
    INJ_OFF = 0,
    INJ_NORMAL,
    INJ_CELL_OV,
    INJ_CELL_UV,
    INJ_IMBALANCE,
    INJ_OVER_TEMP,
    INJ_MAX
} inject_t;

static uint8_t s_inject;        /* inject_t */

/* EVSE 하트비트 흉내 ('p').
 * 하트비트는 CFG_LINK_TIMEOUT_MS(1초)만 유효하므로 1회 갱신으로는 곧 LINK_TIMEOUT 이
 * 재발한다. 실제 EVSE 처럼 계속 갱신해야 해서 토글로 두고 100ms 슬롯마다 갱신한다. */
static bool s_link_sim;

/* --- EVSE 역할 ('e') : 2보드 CAN 테스트 ---
 * 같은 펌웨어를 올린 Nucleo 두 장으로 실버스를 만든다.
 * 보드 B 에서 'e' 를 누르면 0x200/0x201 을 100ms 로 쏘는 EVSE 흉내가 된다.
 *
 * 역할은 배타적이다 — 켜지면 BMS 프레임(0x100~0x104) 송신을 멈춘다.
 * 같은 ID 를 두 노드가 동시에 실으면 데이터 구간에서 비트 에러가 터지는데,
 * 증상이 "간헐적으로 CAN 이 죽는다" 로만 나와 원인을 찾기 매우 어렵다.
 * 이 보드는 0x200 을 받지 못해 FAULT 에 갇히므로, 자기 로그가 상대 프레임을
 * 덮지 않도록 s_link_sim 도 같이 켠다. */
static bool s_evse_role;
static bool s_evse_role_req  = true;    /* 0x201 충전 요청 ('r') */
static bool s_evse_role_stop;           /* 0x200 E-Stop    ('s') */

/* 0x205 로 쏠 과온 임계 순환값. 700 은 상한(CFG_OT_LIMIT_MAX_C10=600)을 일부러 넘긴다 —
 * 상대 BMS 가 클램프해서 600 을 에코백하는지가 원격 파라미터의 핵심 안전 동작이다. */
static const int16_t k_param_ot[] = { 350, 500, 700, 550 };
static uint8_t       s_param_idx;

static const char *inject_name(uint8_t m)
{
    static const char *name[INJ_MAX] = {
        "OFF", "NORMAL", "CELL_OV", "CELL_UV", "IMBALANCE", "OVER_TEMP"
    };
    return (m < (uint8_t)INJ_MAX) ? name[m] : "?";
}

/**
 * @brief  주입 모드에 맞춰 블랙보드를 덮어쓴다.
 * @note   실측 소스가 있는 항목은 건드리지 않는다. S3 가 켜져 있으면 pack_mv/pack_ma 를
 *         남겨 두어 Fault 시연 중에도 로그/OLED 에 실측값이 보인다.
 */
static void ap_inject_apply(void)
{
    static const int32_t tbl[INJ_MAX][BMS_CELL_COUNT] = {
        /* OFF       */ { 0,    0,    0,    0    },
        /* NORMAL    */ { 3700, 3700, 3700, 3700 },
        /* CELL_OV   */ { 4250, 3700, 3700, 3700 },   /* > CFG_CELL_OV_MV   */
        /* CELL_UV   */ { 2900, 3700, 3700, 3700 },   /* < CFG_CELL_UV_MV   */
        /* IMBALANCE */ { 3900, 3700, 3650, 3700 },   /* 편차 250mV, Warning */
        /* OVER_TEMP */ { 3700, 3700, 3700, 3700 }
    };
    uint8_t i;
    int32_t mx, mn, sum = 0;

    if (s_inject == (uint8_t)INJ_OFF) {
        return;
    }

    for (i = 0; i < BMS_CELL_COUNT; i++) {
        s_bms.cell_mv[i] = tbl[s_inject][i];
        sum += s_bms.cell_mv[i];
    }

    mx = s_bms.cell_mv[0];
    mn = s_bms.cell_mv[0];
    for (i = 1; i < BMS_CELL_COUNT; i++) {
        mx = MAX(mx, s_bms.cell_mv[i]);
        mn = MIN(mn, s_bms.cell_mv[i]);
    }
    s_bms.cell_max_mv  = mx;
    s_bms.cell_min_mv  = mn;
    s_bms.imbalance_mv = mx - mn;

#if (BRINGUP_S3_PACK_SENSOR == 1)
    UNUSED_ARG(sum);                /* 팩 전압/전류는 실측(INA2xx)을 그대로 둔다 */
#else
    s_bms.pack_mv = sum;
    s_bms.pack_ma = 0;
#endif
#if (BRINGUP_S4_NTC_ACS == 0)
    s_bms.acs_ma = s_bms.pack_ma;   /* 둘이 어긋나면 SENSOR_ERR 가 끼어든다 */
#endif

    if (s_inject == (uint8_t)INJ_OVER_TEMP) {
        /* 컴파일 상수가 아니라 "지금 적용 중인" 임계 기준으로 넘긴다.
         * 0x205 로 임계를 올려 둔 상태에서도 'f' 가 동작해야 한다. */
        s_bms.temp_c10 = bms_fault_get_ot_threshold() + 50;
    }
#if (BRINGUP_S4_NTC_ACS == 0)
    else {
        s_bms.temp_c10 = 250;       /* NTC 가 없으므로 상온으로 채운다 */
    }
#endif

    s_bms.sensor_ready = true;      /* 실제 센서가 없어도 주입 중엔 정상 취급 */
}

/* ------------------------------------------------------------------ */
static void ap_collect(void)
{
    uint8_t i;
    int32_t mx, mn;

    s_bms.vdda_mv = hw_adc_get_vdda_mv();

#if (BRINGUP_S2_CELL_ADC == 1)
    cell_adc_update();
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        s_bms.node_mv[i] = cell_adc_get_node_mv(i);
        s_bms.cell_mv[i] = cell_adc_get_cell_mv(i);
    }
    s_bms.pack_mv = cell_adc_get_pack_mv();

    mx = s_bms.cell_mv[0];
    mn = s_bms.cell_mv[0];
    for (i = 1; i < BMS_CELL_COUNT; i++) {
        mx = MAX(mx, s_bms.cell_mv[i]);
        mn = MIN(mn, s_bms.cell_mv[i]);
    }
    s_bms.cell_max_mv  = mx;
    s_bms.cell_min_mv  = mn;
    s_bms.imbalance_mv = mx - mn;
#else
    UNUSED_ARG(i); UNUSED_ARG(mx); UNUSED_ARG(mn);
#endif

#if (BRINGUP_S3_PACK_SENSOR == 1)
    if (pack_sensor_update()) {
        s_bms.pack_mv = pack_sensor_get_bus_mv();   /* 션트 센서 값을 우선 채택 */
        s_bms.pack_ma = pack_sensor_get_current_ma();
    }
#endif

    /* EVSE 수신 상태 -> 블랙보드. FSM 이 bms_can.h 를 직접 알지 않게 하는 유일한 통로다. */
#if (BRINGUP_S6_LINK == 1)
    /* 링크 두절 중에는 수신 캐시를 계속 비운다. 안 지우면 링크 복구 순간
     * stale charge_req=1 로 아무도 요청하지 않은 충전이 재개된다.
     * (s_bms.fault 는 한 주기 늦지만 그 사이 LINK_TIMEOUT 이 permit=0 으로 묶어 둔다) */
    if (FLAG_IS_SET(s_bms.fault, BMS_FLT_LINK_TIMEOUT)) {
        bms_can_clear_evse_state();
    }
    s_bms.evse_charge_req = bms_can_evse_charge_req();
    s_bms.evse_connected  = bms_can_evse_connected();
    s_bms.evse_relay_on   = bms_can_evse_relay_on();
    s_bms.evse_estop      = bms_can_evse_estop();
    s_bms.evse_fault      = bms_can_evse_fault();
#else
    /* CAN 이 없으면 0x201 이 영원히 안 와서 CHARGE_READY 로 못 올라간다.
     * 하트비트와 충전 요청을 s_link_sim 하나에 묶어 'p' 한 번으로
     * "EVSE 가 사라진" 상황이 통째로 재현되게 한다. */
    s_bms.evse_charge_req = s_link_sim;
    s_bms.evse_connected  = s_link_sim;
    s_bms.evse_relay_on   = false;
    s_bms.evse_estop      = false;
    s_bms.evse_fault      = 0U;
#endif

    /* 주입은 항상 마지막 : 실측값을 덮어써야 시연이 성립한다 */
    ap_inject_apply();
}

/**
 * @brief  충전 차단 릴레이(PA8) 지령
 * @note   permit 은 "배터리가 받아도 되는가"만 답한다. 접점을 닫으려면 EVSE 가 실제로
 *         요청했고 FSM 이 충전 구간에 있어야 한다 — permit 만 보면 아무도 요청하지 않은
 *         IDLE 에서 접점이 붙는다. FSM 이 이미 permit=0 을 강제하므로 사실상 삼중이지만,
 *         릴레이는 되돌리기 어려운 물리 동작이라 중복을 의도적으로 남긴다.
 */
static void ap_relay_update(void)
{
    bool on = s_bms.charge_permit
              && ((s_bms.state == BMS_ST_CHARGE_READY) || (s_bms.state == BMS_ST_CHARGING));

    if (on != s_bms.relay_on) {
        DBG_W("RELAY %s (permit=%d state=%s)%s",
              on ? "CLOSE" : "OPEN", (int)s_bms.charge_permit,
              bms_fsm_state_name(s_bms.state),
              (BRINGUP_S8_RELAY == 1) ? "" : "  [OUTPUT DISABLED]");
        s_bms.relay_on = on;
    }

#if (BRINGUP_S8_RELAY == 1)
    /* 매 주기 다시 쓴다. 노이즈로 포트 래치가 흐트러져도 100ms 안에 복구된다. */
    hw_gpio_set(HW_OUT_RELAY_CHG, on);
#else
    /* "안 건드린다" 가 아니라 "명시적으로 연다". 액티브 레벨을 잘못 잡았을 때
     * 부팅 직후 초기화 한 번에만 의존하지 않기 위해서다. */
    hw_gpio_set(HW_OUT_RELAY_CHG, false);
#endif
}

/* ------------------------------------------------------------------ */
static void ap_task_100ms(void)
{
    ap_collect();

    if (s_link_sim) {
        bms_fault_notify_link();        /* EVSE 가 붙어 있는 상황을 흉내 */
    }

#if (BRINGUP_S7_FSM_FAULT == 1)
    bms_fault_check(&s_bms);
    bms_fsm_run(&s_bms);
    status_led_set_state(s_bms.state);
#endif

    /* FSM 직후 / CAN 송신 직전. "Fault 후 100ms 내 차단" 을 물리 접점까지 같은 슬롯에서 끝낸다. */
    ap_relay_update();

    status_led_update();

#if (BRINGUP_S6_LINK == 1)
    /* 실제 EVSE 도 0x200/0x201 을 100ms 로 보낸다. 같은 슬롯에 두어야
     * 상대 BMS 의 LINK_TIMEOUT 여유가 실물과 같아진다. */
    if (s_evse_role) {
        bms_can_sim_evse(s_evse_role_req, s_evse_role_stop);
    } else {
        bms_link_send_main(&s_bms);
    }
#endif
}

static void ap_task_500ms(void)
{
#if (BRINGUP_S4_NTC_ACS == 1)
    ntc_update();
    acs712_update();
    s_bms.temp_c10 = ntc_get_temp_c10(0);
    s_bms.acs_ma   = acs712_get_ma();
#endif

#if (BRINGUP_S5_OLED == 1)
    bms_ui_update(&s_bms);
#endif

#if (BRINGUP_S6_LINK == 1)
    if (!s_evse_role) {
        bms_link_send_cell(&s_bms);
    }
#endif
}

static void ap_task_1000ms(void)
{
#if (BRINGUP_S2_CELL_ADC == 1)
    bms_soc_update(&s_bms);
#endif

#if (BRINGUP_S6_LINK == 1)
    if (!s_evse_role) {
        bms_link_send_version();
    }
    /* CAN 요약을 상태 요약보다 먼저 찍어 "원인 -> 결과" 순서로 읽히게 한다.
     * 링크가 죽어 있으면 아래 줄의 FLT/P 는 전부 그 결과일 뿐이다. */
    bms_can_log_stats();
    if (s_evse_role) {
        DBG_W("[EVSE ROLE] 0x200/0x201 송신 중 (req=%d estop=%d) - BMS 프레임 정지",
              (int)s_evse_role_req, (int)s_evse_role_stop);
    }
#endif

    /* --- 콘솔 요약 로그 --- */
#if (BRINGUP_S1_LED_DBG == 1)
    DBG_I("[%s] C:%ld/%ld/%ld/%ld mV  PACK:%ldmV %ldmA  T:%ld.%ldC  SOC:%u%%  FLT:0x%04X P:%d RLY:%d",
          bms_fsm_state_name(s_bms.state),
          (long)s_bms.cell_mv[0], (long)s_bms.cell_mv[1],
          (long)s_bms.cell_mv[2], (long)s_bms.cell_mv[3],
          (long)s_bms.pack_mv, (long)s_bms.pack_ma,
          (long)(s_bms.temp_c10 / 10), (long)(s_bms.temp_c10 % 10),
          s_bms.soc, s_bms.fault, (int)s_bms.charge_permit, (int)s_bms.relay_on);
#endif
}

/* ==================================================================
 *  셀 ADC 캘리브레이션 콘솔
 *
 *  콘솔이 1바이트 논블로킹이라 "노드 번호 + 실측 mV" 를 한 줄로 못 받는다.
 *  그래서 작은 입력 상태머신을 둔다:
 *      NORMAL --'k'--> SEL_NODE --'1'~'4'--> ENTER_MV --숫자+Enter--> 적용 --> NORMAL
 *                          └───────── 'q' / ESC 로 언제든 취소 ─────────┘
 *
 *  보정 단위는 셀이 아니라 노드(B1/B2/B3/B+)다. 셀 전압은 노드의 차분이라
 *  셀에 직접 게인을 걸 수 없다 — 노드를 맞추면 셀은 따라온다.
 *  DMM 으로 B- 기준 각 노드를 재서 입력한다.
 * ================================================================== */
typedef enum {
    CON_NORMAL = 0,
    CON_CAL_SEL_NODE,
    CON_CAL_ENTER_MV
} con_mode_t;

static uint8_t s_con_mode;
static uint8_t s_cal_node;
static int32_t s_cal_acc;
static uint8_t s_cal_digits;

/** @brief 현재 보정값을 cell_adc_init() 에 그대로 붙여넣을 수 있는 형태로 찍는다 */
static void ap_cal_dump(void)
{
    uint8_t i;

    DBG_I("--- cell_adc calibration (RAM only - 리셋하면 사라진다) ---");
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        DBG_I("    cell_adc_set_gain(%u, %ld);   /* node%u : %ld mV */",
              i, (long)cell_adc_get_gain(i), i + 1U, (long)cell_adc_get_node_mv(i));
    }
    DBG_I("--- 값을 굳히려면 위 4줄을 cell_adc_init() 끝에 넣을 것 ---");
}

static void ap_console_cal(uint8_t ch)
{
    /* 취소를 먼저 처리한다 (숫자 입력 중에도 빠져나올 수 있어야 한다) */
    if ((ch == 27U) || (ch == 'q') || (ch == 'Q')) {
        DBG_W("cal: 취소");
        s_con_mode = (uint8_t)CON_NORMAL;
        return;
    }

    if (s_con_mode == (uint8_t)CON_CAL_SEL_NODE) {
        if ((ch < '1') || (ch > '4')) {
            DBG_W("cal: 노드 번호 1~4 를 누를 것 (q=취소)");
            return;
        }
        s_cal_node   = (uint8_t)(ch - '1');
        s_cal_acc    = 0;
        s_cal_digits = 0;
        s_con_mode   = (uint8_t)CON_CAL_ENTER_MV;
        DBG_I("cal: node%u 현재 측정 %ld mV -> DMM 실측값[mV] 입력 후 Enter (q=취소)",
              s_cal_node + 1U, (long)cell_adc_get_node_mv(s_cal_node));
        return;
    }

    /* --- CON_CAL_ENTER_MV --- */
    if ((ch >= '0') && (ch <= '9')) {
        if (s_cal_digits < 5U) {            /* 5자리 = 99999mV. 그 이상은 오타다 */
            s_cal_acc = (s_cal_acc * 10) + (int32_t)(ch - '0');
            s_cal_digits++;
        }
        return;
    }
    if ((ch != '\r') && (ch != '\n')) {
        return;                             /* 숫자와 Enter 외에는 무시 */
    }
    if (s_cal_digits == 0U) {
        DBG_W("cal: 값이 비어 있다 (q=취소)");
        return;
    }

    if (cell_adc_calibrate_node(s_cal_node, s_cal_acc)) {
        DBG_W("cal: node%u = %ld mV 적용, gain=%ld/65536",
              s_cal_node + 1U, (long)s_cal_acc, (long)cell_adc_get_gain(s_cal_node));
    } else {
        /* 가드레일에 걸린 것. 대개 노드를 잘못 짚었거나 배선이 안 붙어 있다. */
        DBG_E("cal: 거부 (측정 %ld mV / 요청 %ld mV) - 노드 번호와 배선 확인",
              (long)cell_adc_get_node_mv(s_cal_node), (long)s_cal_acc);
    }
    s_con_mode = (uint8_t)CON_NORMAL;
}

/* ------------------------------------------------------------------ */
static void ap_console(void)
{
    uint8_t ch;

    while (hw_uart_get_byte(&ch)) {

        if (s_con_mode != (uint8_t)CON_NORMAL) {
            ap_console_cal(ch);
            continue;
        }

        switch (ch) {
        case 'p': case 'P':                     /* EVSE 하트비트 흉내 on/off */
            s_link_sim = !s_link_sim;
            if (s_link_sim) {
                bms_fault_notify_link();
            }
            DBG_W("link heartbeat sim -> %s", s_link_sim ? "ON" : "OFF");
#if (BRINGUP_S6_LINK == 1)
            /* 'p' 는 CAN 을 우회해 notify_link() 를 직접 부른다.
             * 켜 두면 "CAN 수신이 되고 있다" 와 구분이 안 된다. */
            if (s_link_sim) {
                DBG_W("  (주의: 'p' 는 CAN 을 우회한다. CAN 수신 검증 중이면 끌 것)");
            }
#endif
            break;

        case 't': case 'T':                     /* CAN 프레임 트레이스 on/off */
            if (!bms_can_is_ready()) {
                DBG_E("CAN 이 기동되지 않았다 (BRINGUP_S6_LINK=%d / init 실패 확인)",
                      BRINGUP_S6_LINK);
                break;
            }
            bms_can_set_trace(!bms_can_get_trace());
            DBG_W("CAN frame trace -> %s", bms_can_get_trace() ? "ON" : "OFF");
            break;

        case 'e': case 'E':                     /* EVSE 역할 토글 (2보드 테스트) */
            if (!bms_can_is_ready()) {
                DBG_E("CAN 이 기동되지 않았다 (BRINGUP_S6_LINK=%d / init 실패 확인)",
                      BRINGUP_S6_LINK);
                break;
            }
            s_evse_role = !s_evse_role;
            if (s_evse_role) {
                /* 이 보드는 더 이상 BMS 가 아니다. 자기 LINK_TIMEOUT 로그가
                 * 상대 프레임을 덮지 않도록 하트비트 시뮬을 같이 켠다. */
                s_link_sim = true;
                bms_fault_notify_link();
                DBG_W("EVSE ROLE ON - 이 보드는 0x200/0x201 만 보낸다 (BMS 프레임 정지)");
                DBG_W("  r=충전요청 토글  s=E-Stop 토글  w=0x205 과온임계 쓰기");
            } else {
                DBG_W("EVSE ROLE OFF - BMS 로 복귀 (0x100~0x104 송신 재개)");
            }
            break;

        case 'r': case 'R':                     /* EVSE 역할: 충전 요청 토글 */
            if (!s_evse_role) {
                DBG_E("EVSE 역할이 아니다 ('e' 로 먼저 켤 것)");
                break;
            }
            s_evse_role_req = !s_evse_role_req;
            DBG_W("EVSE charge_req(0x201) -> %d", (int)s_evse_role_req);
            break;

        case 's': case 'S':                     /* EVSE 역할: E-Stop 토글 */
            if (!s_evse_role) {
                DBG_E("EVSE 역할이 아니다 ('e' 로 먼저 켤 것)");
                break;
            }
            s_evse_role_stop = !s_evse_role_stop;
            DBG_W("EVSE E-Stop(0x200[3]) -> %d", (int)s_evse_role_stop);
            break;

        case 'w': case 'W':                     /* EVSE 역할: 0x205 파라미터 쓰기 */
            if (!s_evse_role) {
                DBG_E("EVSE 역할이 아니다 ('e' 로 먼저 켤 것)");
                break;
            }
            bms_can_sim_param(k_param_ot[s_param_idx]);
            s_param_idx = (uint8_t)((s_param_idx + 1U) % ARRAY_SIZE(k_param_ot));
            break;

        case 'v': case 'V':                     /* ADC raw 덤프 (캘리브레이션용) */
            {
                uint16_t raw[ADC_IDX_MAX];
                uint8_t  i;
                hw_adc_snapshot(raw);
                for (i = 0; i < (uint8_t)ADC_IDX_MAX; i++) {
                    DBG_I("adc[%u] raw=%4u  pin=%ld uV", i, raw[i],
                          (long)hw_adc_raw_to_uv(raw[i]));
                }
                DBG_I("VDDA = %ld mV", (long)hw_adc_get_vdda_mv());
            }
            break;

        case 'c': case 'C':                     /* ACS712 오프셋 재캘리브레이션 */
            (void)acs712_calibrate();
            break;

        case 'i': case 'I':                     /* 팩 전류 센서 재초기화 */
            (void)pack_sensor_init();
            break;

        case 'f': case 'F':                     /* Fault 주입 : 다음 모드로 순환 */
            s_inject++;
            if (s_inject >= (uint8_t)INJ_MAX) {
                s_inject = (uint8_t)INJ_NORMAL; /* OFF 는 'n' 으로만 간다 */
            }
            DBG_W("inject -> %s", inject_name(s_inject));
            break;

        case 'n': case 'N':                     /* 주입 해제 (실측값으로 복귀) */
            s_inject = (uint8_t)INJ_OFF;
            DBG_W("inject -> OFF");
            break;

        case 'k': case 'K':                     /* 셀 노드 게인 캘리브레이션 */
#if (BRINGUP_S2_CELL_ADC == 1)
            if (s_inject != (uint8_t)INJ_OFF) {
                /* 캘리브레이션은 dev 계층 값을 직접 읽어 주입과 간섭하지 않지만,
                 * 로그/OLED 에는 주입값이 보여서 헷갈리므로 미리 알린다. */
                DBG_W("cal: 주입이 켜져 있다(%s). 로그의 셀 전압은 가짜다 - 'n' 권장",
                      inject_name(s_inject));
            }
            s_con_mode = (uint8_t)CON_CAL_SEL_NODE;
            DBG_I("cal: 보정할 노드 선택 1~4 (B1/B2/B3/B+), q=취소");
            DBG_I("     현재 %ld / %ld / %ld / %ld mV",
                  (long)cell_adc_get_node_mv(0), (long)cell_adc_get_node_mv(1),
                  (long)cell_adc_get_node_mv(2), (long)cell_adc_get_node_mv(3));
#else
            DBG_E("cal: BRINGUP_S2_CELL_ADC=0 이라 셀 ADC 가 돌지 않는다");
#endif
            break;

        case 'd': case 'D':                     /* 보정값 덤프 (코드로 굳히기 위해) */
            ap_cal_dump();
            break;

        case 'x': case 'X':                     /* 보정값 전체 초기화 */
            cell_adc_reset_cal();
            DBG_W("cal: 전 채널 x1.000 / 0mV 로 초기화");
            break;

        case 'h': case 'H':
            DBG_I("cmd: f=fault inject  n=inject off  p=heartbeat sim toggle");
            DBG_I("     v=adc dump  c=acs cal  i=ina init");
            DBG_I("     k=cell cal  d=cal dump  x=cal reset");
            DBG_I("CAN: t=frame trace  e=EVSE role  r=charge req  s=E-Stop  w=param write");
            DBG_I("now: inject=%s  link_sim=%s  relay=%d%s",
                  inject_name(s_inject), s_link_sim ? "ON" : "OFF",
                  (int)s_bms.relay_on,
                  (BRINGUP_S8_RELAY == 1) ? "" : " (output disabled)");
            DBG_I("CAN: %s  trace=%s  role=%s",
                  bms_can_is_ready() ? ((CFG_CAN_MODE_LOOPBACK == 1) ? "LOOPBACK" : "NORMAL")
                                     : "NOT STARTED",
                  bms_can_get_trace() ? "ON" : "OFF",
                  s_evse_role ? "EVSE" : "BMS");
            break;

        default:
            break;
        }
    }
}

/* ================================================================== */
void bms_app_init(void)
{
    memset(&s_bms, 0, sizeof(s_bms));
    s_bms.temp_c10     = BMS_TEMP_INVALID;
    s_bms.sensor_ready = true;

    dbg_init();
    status_led_init();
    bms_fsm_init(&s_bms);
    bms_fault_init();
    bms_soc_init();
    bms_link_init();

    DBG_I("--- bringup stage flags ---");
    DBG_I("S1 LED/DBG   : %d", BRINGUP_S1_LED_DBG);
    DBG_I("S1B ADC_RAW  : %d", BRINGUP_S1B_ADC_RAW);
    DBG_I("S2 CELL_ADC  : %d", BRINGUP_S2_CELL_ADC);
    DBG_I("S3 PACK SENS : %d (%s)", BRINGUP_S3_PACK_SENSOR, PACK_SENSOR_NAME);
    DBG_I("S4 NTC/ACS   : %d", BRINGUP_S4_NTC_ACS);
    DBG_I("S5 OLED      : %d", BRINGUP_S5_OLED);
    DBG_I("S6 LINK      : %d", BRINGUP_S6_LINK);
    DBG_I("S7 FSM/FAULT : %d", BRINGUP_S7_FSM_FAULT);
    DBG_I("S8 RELAY OUT : %d%s", BRINGUP_S8_RELAY,
          (BRINGUP_S8_RELAY == 1) ? " (PA8 LIVE - 배선 확인 완료?)" : " (출력 차단)");

#if BRINGUP_ADC_HW
    cell_adc_init();
    if (!hw_adc_init()) {
        s_bms.sensor_ready = false;
    }
    /* 첫 오버샘플 평균이 확정될 때까지 대기 (약 6ms) */
    {
        uint32_t t0 = hw_tick_ms();
        while (!hw_adc_is_ready() && !hw_tick_elapsed(t0, 200)) {
            /* busy wait : 초기화 단계에서만 허용 */
        }
        if (!hw_adc_is_ready()) {
            DBG_E("ADC first average timeout");
            s_bms.sensor_ready = false;
        }
    }
#endif

/* 팩 센서(0x40)와 OLED(0x3C)가 I2C1 을 공유한다. 둘 중 하나만 켜도 버스는 필요하다. */
#if (BRINGUP_S3_PACK_SENSOR == 1) || (BRINGUP_S5_OLED == 1)
    hw_i2c_init();
#endif

#if (BRINGUP_S3_PACK_SENSOR == 1)
    if (!pack_sensor_init()) {
        (void)hw_i2c_recover();
        if (!pack_sensor_init()) {
            s_bms.sensor_ready = false;
        }
    }
#endif

#if (BRINGUP_S4_NTC_ACS == 1)
    ntc_init();
    acs712_init();
    (void)acs712_calibrate();       /* 반드시 무전류 상태에서 부팅할 것 */
#endif

#if (BRINGUP_S5_OLED == 1)
    if (oled_init()) {
        bms_ui_init();
    }
#endif

#if (BRINGUP_S7_FSM_FAULT == 1) && (BRINGUP_S2_CELL_ADC == 0)
    /* 셀 ADC 가 없으면 cell_mv 가 전부 0 이라 SELF_CHECK 가 곧바로 실패한다.
     * 시연을 시작할 수 있도록 NORMAL 주입으로 출발한다. */
    s_inject = (uint8_t)INJ_NORMAL;
    DBG_W("!! CELL ADC OFF -> fault injection ON (fake cell data) !!");
    DBG_W("!! 'f' 로 OV/UV/IMBAL/OT 순환, 'n' 으로 해제 !!");
#endif

#if (BRINGUP_S6_LINK == 0)
    /* EVSE 가 없으면 1초 뒤 LINK_TIMEOUT 으로 FAULT 에 갇힌다.
     * 링크 외 동작을 먼저 보려면 하트비트 시뮬이 켜져 있어야 한다. */
    s_link_sim = true;
    DBG_W("!! LINK OFF -> heartbeat sim ON ('p' 로 토글) !!");
#endif

    s_t100  = hw_tick_ms();
    s_t500  = s_t100;
    s_t1000 = s_t100;

    DBG_I("bms_app_init done. sensor_ready=%d", (int)s_bms.sensor_ready);
}

void bms_app_main(void)
{
    /* 수신은 주기 슬롯이 아니라 매 바퀴 돈다.
     * CAN RX FIFO 가 3단뿐이라 100ms 슬롯에 묶으면 넘친다. (UART 백엔드에서는 무동작) */
#if (BRINGUP_S6_LINK == 1)
    bms_link_poll_rx();
#endif

    if (hw_tick_due(&s_t100, CFG_TASK_100MS)) {
        ap_task_100ms();
    }
    if (hw_tick_due(&s_t500, CFG_TASK_500MS)) {
        ap_task_500ms();
    }
    if (hw_tick_due(&s_t1000, CFG_TASK_1000MS)) {
        ap_task_1000ms();
    }

    ap_console();
}

const bms_data_t *bms_app_get_data(void)
{
    return &s_bms;
}
