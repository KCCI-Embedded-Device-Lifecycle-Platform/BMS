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

/* 자가진단은 init(끝)과 콘솔 's' 두 곳에서 불린다. 정의는 init 바로 앞에 두어
 * "부팅 마지막 단계" 라는 순서를 코드 배치로도 드러낸다. */
static void ap_selfcheck(void);

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
    /* CAN 을 끄면 0x201 이 영원히 안 오므로 CHARGE_READY 로 올라갈 수 없다.
     * 예전에는 콘솔 'p' 로 하트비트를 흉내내 이 구간을 메웠지만, 실링크가 붙은 뒤
     * 그 경로는 제거했다 — "정말 받은 것" 과 구분되지 않아 진단을 오염시킨다.
     * 즉 S6=0 은 이제 "링크 없음" 그대로이고, IDLE 에서 더 올라가지 않는 것이 정상이다. */
    s_bms.evse_charge_req = false;
    s_bms.evse_connected  = false;
    s_bms.evse_relay_on   = false;
    s_bms.evse_estop      = false;
    s_bms.evse_fault      = 0U;
#endif
}
/* ==================================================================
 *  릴레이 개폐 창(window) 트레이스 — "릴레이를 닫으면 CELL_OV" 전용 계측
 *
 *  이 현상은 1초 요약 로그로는 원인을 가를 수 없다. 트립하면 FSM 이 같은 슬롯에서
 *  릴레이를 다시 열고, 열리면 값이 원래대로 돌아와서 요약에는 "임계 이하" 만 남는다.
 *  ("CELL_OV 인데 로그의 셀 전압은 4200 미만" 으로 보이는 이유가 이것이다)
 *
 *  그래서 릴레이 지령이 바뀌는 순간에만 창을 열고 100ms 슬롯마다 한 줄씩 찍는다.
 *  상시 출력이 아니라 창 안에서만 나오므로 다른 로그를 덮지 않는다.
 *
 *  dN[i] = "닫히기 직전(= 열린 상태)" 노드값 대비 변화량이다. 이 한 줄로 갈린다:
 *
 *    dN 넷이 비슷하게 +     -> 기준전위(GND)가 통째로 밀린 것이다.
 *                             cell1 = node1 그 자체라 이동분이 셀1 에만 실리고
 *                             cell2~4 는 차분이라 상쇄된다. 원인은 릴레이 코일
 *                             전류 x 공용 접지 배선 저항 -> 코일 GND 를 star point 로.
 *    dN 이 뒤로 갈수록 커짐 -> 팩 전압이 실제로 오른 것 = 진짜 충전 전류다.
 *                             이때 CELL_OV 는 오판이 아니라 정상 동작이다.
 *    dTEC 가 같이 튐        -> 교란이 CAN 차동버스까지 갔다. 셀 ADC 와 CAN 은
 *                             서로 무관한 두 계통이라 공통분모는 접지뿐이다.
 * ================================================================== */
#define RLYW_SLOTS      12U     /* 100ms x 12 = 1.2s. 닫힘->트립(300ms)->열림->복귀를 덮는다 */

static uint8_t  s_rlyw_left;                    /* 남은 출력 슬롯 (0 = 창 닫힘) */
static int32_t  s_rlyw_base[BMS_CELL_COUNT];    /* 릴레이가 열려 있던 마지막 노드값 */
static uint16_t s_rlyw_base_tec;
static uint32_t s_rlyw_t0;

/**
 * @brief  릴레이 지령이 바뀌는 순간 호출 — 기준값을 잡고 창을 연다
 * @param  closing : 이번 변화가 "닫힘" 인가
 * @note   기준은 반드시 "닫기 직전" 값이어야 한다. ap_collect() 가 이번 슬롯 앞에서
 *         이미 돌았고 그때는 접점이 아직 열려 있었으므로 s_bms.node_mv 가 곧
 *         열린 상태의 값이다. 열릴 때는 기준을 새로 잡지 않는다 — 닫힘 구간과
 *         같은 자로 재야 dN 이 비교 가능한 값이 된다.
 */
static void ap_relay_window_arm(bool closing)
{
    uint8_t i;

    if (closing) {
        for (i = 0; i < BMS_CELL_COUNT; i++) {
            s_rlyw_base[i] = s_bms.node_mv[i];
        }
        s_rlyw_base_tec = bms_can_get_tec();
        s_rlyw_t0       = hw_tick_ms();
    }
    s_rlyw_left = RLYW_SLOTS;
}

/**
 * @brief  창이 열려 있는 동안 100ms 슬롯마다 1줄
 * @note   dbg 버퍼가 128B 라 한 줄에 다 못 넣는다. 절대 노드값은 Fault 확정 시
 *         bms_fault.c 가 따로 찍으므로 여기서는 변화량(dN)만 남긴다.
 */
static void ap_relay_window_trace(void)
{
    if (s_rlyw_left == 0U) {
        return;
    }
    s_rlyw_left--;

    DBG_W("RLYW+%lums R:%d dN%+ld/%+ld/%+ld/%+ld C:%ld/%ld/%ld/%ld TEC%+d F:0x%04X",
          (unsigned long)(hw_tick_ms() - s_rlyw_t0), (int)s_bms.relay_on,
          (long)(s_bms.node_mv[0] - s_rlyw_base[0]),
          (long)(s_bms.node_mv[1] - s_rlyw_base[1]),
          (long)(s_bms.node_mv[2] - s_rlyw_base[2]),
          (long)(s_bms.node_mv[3] - s_rlyw_base[3]),
          (long)s_bms.cell_mv[0], (long)s_bms.cell_mv[1],
          (long)s_bms.cell_mv[2], (long)s_bms.cell_mv[3],
          (int)((int32_t)bms_can_get_tec() - (int32_t)s_rlyw_base_tec),
          s_bms.fault);
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
        ap_relay_window_arm(on);      /* 개폐 전후 원값을 남긴다 (RLYW 트레이스) */
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

#if (BRINGUP_S7_FSM_FAULT == 1)
    bms_fault_check(&s_bms);
    bms_fsm_run(&s_bms);
    status_led_set_state(s_bms.state);
#endif

    /* FSM 직후 / CAN 송신 직전. "Fault 후 100ms 내 차단" 을 물리 접점까지 같은 슬롯에서 끝낸다. */
    ap_relay_update();

    /* 노랑 LED 는 릴레이 지령을 그대로 따른다. ap_relay_update() 와 status_led_update()
     * 사이여야 접점 개폐와 표시가 같은 슬롯에서 끝난다 — 한 슬롯 밀리면 "릴레이는 떨어졌는데
     * 노랑이 아직 켜져 있는" 100ms 가 생기고, 그게 차단 실패와 구분되지 않는다. */
    status_led_set_charge(s_bms.relay_on);

    status_led_update();

#if (BRINGUP_S6_LINK == 1)
    bms_link_send_main(&s_bms);
#endif

    /* 릴레이 개폐 창이 열려 있으면 이번 슬롯 원값을 남긴다.
     * 측정->판단->릴레이->송신이 모두 끝난 뒤라 한 줄이 그 슬롯의 최종 상태다. */
    ap_relay_window_trace();
}

static void ap_task_500ms(void)
{
#if (BRINGUP_S4_NTC_ACS == 1)
    ntc_update();
    acs712_update();
    s_bms.temp_c10 = ntc_get_temp_max_c10();
    s_bms.acs_ma   = acs712_get_ma();
#endif

#if (BRINGUP_S5_OLED == 1)
    bms_ui_update(&s_bms);
#endif

#if (BRINGUP_S6_LINK == 1)
    bms_link_send_cell(&s_bms);
#endif
}

static void ap_task_1000ms(void)
{
#if (BRINGUP_S2_CELL_ADC == 1)
    bms_soc_update(&s_bms);
#endif

#if (BRINGUP_S6_LINK == 1)
    bms_link_send_version();
    /* CAN 요약을 상태 요약보다 먼저 찍어 "원인 -> 결과" 순서로 읽히게 한다.
     * 링크가 죽어 있으면 아래 줄의 FLT/P 는 전부 그 결과일 뿐이다. */
    bms_can_log_stats();
#endif

    /* --- 콘솔 요약 로그 --- */
#if (BRINGUP_S1_LED_DBG == 1)
    /* PACK 의 출처를 표시한다. 팩 센서 읽기가 실패하면 ap_collect() 가 셀 ADC 합계를
     * 그대로 두는데, 둘 다 "그럴듯한 팩 전압" 이라 로그만 봐서는 구분이 안 된다.
     * 실제로 이것 때문에 INA226 이 25분간 죽어 있는 줄 모르고 지나갔다. */
#if (BRINGUP_S3_PACK_SENSOR == 1)
    const char *p_pack_src = pack_sensor_is_ok() ? "" : "(cellADC)";
#else
    const char *p_pack_src = "(cellADC)";
#endif

    DBG_I("[%s] C:%ld/%ld/%ld/%ld mV  PACK:%ldmV%s %ldmA  T:%ld.%ldC  SOC:%u%%  FLT:0x%04X P:%d RLY:%d",
          bms_fsm_state_name(s_bms.state),
          (long)s_bms.cell_mv[0], (long)s_bms.cell_mv[1],
          (long)s_bms.cell_mv[2], (long)s_bms.cell_mv[3],
          (long)s_bms.pack_mv, p_pack_src, (long)s_bms.pack_ma,
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
        case 's': case 'S':                     /* 자가진단 재실행 */
            /* 배선을 고친 뒤 리셋 없이 다시 확인할 수 있어야 벤치 시행착오가 짧아진다.
             * sensor_ready 를 다시 세우므로 SELF_CHECK 실패로 FAULT 에 갇힌 상태에서
             * 복구 경로로도 쓸 수 있다 (Fault 해제는 여전히 히스테리시스를 따른다). */
            ap_collect();
            ap_selfcheck();
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
#if (BRINGUP_S4_NTC_ACS == 1)
                /* 블랙보드에는 최댓값 하나만 남으므로, 어느 채널이 단선이고
                 * 어느 채널이 튀는지는 여기서만 보인다. */
                for (i = 0; i < 4U; i++) {
                    DBG_I("ntc[%u] %ld.%ldC  R=%ld ohm", i,
                          (long)(ntc_get_temp_c10(i) / 10),
                          (long)(ntc_get_temp_c10(i) % 10),
                          (long)ntc_get_res_ohm(i));
                }
                DBG_I("ntc valid = %u/4", ntc_get_valid_count());
#endif
            }
            break;

        case 'c': case 'C':                     /* ACS712 오프셋 재캘리브레이션 */
            (void)acs712_calibrate();
            break;

        case 'i': case 'I':                     /* 팩 전류 센서 재초기화 + 즉시 1회 읽기 */
            /* "init 성공" 과 "값이 실제로 들어온다" 는 다른 사실이다.
             * init 은 CONFIG write-verify 까지만 보는데, 주기 읽기는 그 뒤에 따로 죽을 수
             * 있고 한 번 죽으면 s_ok=false 로 조용히 포기한다. 그러면 로그에는
             * "INA226 ok" 만 남고 PACK 값은 셀 ADC 합계에서 안 변해서, 되는 것처럼 보인다.
             * 그 둘을 한 키로 갈라 주지 않으면 여기서 반드시 막힌다. */
            if (pack_sensor_init()) {
                s_bms.sensor_ready = true;
                if (pack_sensor_update()) {
                    DBG_I("INA226 read OK : bus=%ldmV shunt=%lduV I=%ldmA",
                          (long)pack_sensor_get_bus_mv(),
                          (long)pack_sensor_get_shunt_uv(),
                          (long)pack_sensor_get_current_ma());
                    DBG_I("  -> 이 bus 값이 다음 초 로그의 PACK 에 나타나야 정상이다");
                } else {
                    DBG_E("INA226 init 은 됐는데 주기 읽기가 실패한다");
                    DBG_E("  -> PACK 에 보이는 값은 INA226 이 아니라 셀 ADC 합계다");
                }
            }
            break;

        case 'b': case 'B':                     /* I2C 두 버스 복구 + 스캔 + 장치 재초기화 */
            {
                uint8_t bus;
                /* 버스를 나눈 뒤로는 **두 버스를 다 훑는 것**이 핵심이다.
                 * 한쪽만 죽어 있으면 그 버스에 달린 모듈이 곧 범인이고,
                 * 둘 다 죽어 있으면 원인은 모듈이 아니라 공통 GND/3.3V 쪽이다.
                 * 공유 버스일 때는 이 구분이 안 돼서 모듈을 하나씩 떼야 했다.
                 * 스캔 전에 lock-up 복구를 한 번 돌린다 — 슬레이브가 SDA 를 물고 있으면
                 * 스캔 결과가 전부 0 으로 나와서 배선 문제와 구분되지 않는다. */
                for (bus = 0; bus < (uint8_t)HW_I2C_MAX; bus++) {
                    (void)hw_i2c_recover((hw_i2c_bus_t)bus);
                    (void)hw_i2c_scan((hw_i2c_bus_t)bus);
                }
            }
            /* 버스를 세웠으면 그 위 장치도 같이 세운다. 배선을 고친 뒤 리셋 없이
             * 바로 확인할 수 있어야 벤치에서 시행착오가 빨라진다. */
#if (BRINGUP_S3_PACK_SENSOR == 1)
            if (pack_sensor_init()) {
                s_bms.sensor_ready = true;      /* SENSOR_ERR 해제 경로를 열어 준다 */
            }
#endif
#if (BRINGUP_S5_OLED == 1)
            if (oled_init()) {
                bms_ui_init();
            }
#endif
            break;

        case 'k': case 'K':                     /* 셀 노드 게인 캘리브레이션 */
#if (BRINGUP_S2_CELL_ADC == 1)
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

        case 'l': case 'L':                     /* 표시 출력 뮤트 (접지 진단) */
            /* LED 구동 전류가 공용 접지 배선을 타면 셀 ADC 기준전위가 밀린다.
             * 3색 LED 로 바뀌면서 FAULT 는 빨강 상시 점등이라 예전의 100ms 교대가 없다.
             * 지금 100ms 로 교대하는 구간은 SELF_CHECK(초록 0xAAAA) 뿐이므로,
             * 판정은 "뮤트 ON/OFF 로 셀 전압이 통째로 이동하는가"(DC 오프셋)로 본다.
             * 이동이 남아 있으면 원인은 LED 가 아닌 다른 부하다. */
            status_led_set_mute(!status_led_get_mute());
            DBG_W("status LED -> %s", status_led_get_mute() ? "MUTE" : "ON");
            DBG_W("  뮤트 후 셀1 의 100ms 교대가 사라지면 접지 공유가 원인이다");
            break;

        case 'g': case 'G':                     /* 충전 진입 조건 스냅샷 */
            /* "EVSE 버튼을 눌렀는데 충전이 안 된다" 를 한 키로 가른다.
             * CHARGE_READY 진입 조건이 4개(permit / link_ok / charge_req / !estop)라
             * 하나만 빠져도 결과가 똑같이 "안 올라감" 으로 보인다. 1초 요약 로그에는
             * 그 4개 중 permit 밖에 안 나오므로 나머지 셋은 여기서만 보인다. */
            DBG_I("--- charge gate ---");
            DBG_I(" state=%s permit=%d relay=%d fault=0x%04X(%s)",
                  bms_fsm_state_name(s_bms.state), (int)s_bms.charge_permit,
                  (int)s_bms.relay_on, s_bms.fault, bms_fault_name(s_bms.fault));
            DBG_I(" link_ok=%d evse req=%d estop=%d conn=%d rly=%d flt=0x%02X",
                  FLAG_IS_SET(s_bms.fault, BMS_FLT_LINK_TIMEOUT) ? 0 : 1,
                  (int)s_bms.evse_charge_req, (int)s_bms.evse_estop,
                  (int)s_bms.evse_connected, (int)s_bms.evse_relay_on,
                  s_bms.evse_fault);
            DBG_I(" node mV %ld/%ld/%ld/%ld  VDDA %ld",
                  (long)s_bms.node_mv[0], (long)s_bms.node_mv[1],
                  (long)s_bms.node_mv[2], (long)s_bms.node_mv[3],
                  (long)s_bms.vdda_mv);
            DBG_I(" cell mV %ld/%ld/%ld/%ld  imb %ld",
                  (long)s_bms.cell_mv[0], (long)s_bms.cell_mv[1],
                  (long)s_bms.cell_mv[2], (long)s_bms.cell_mv[3],
                  (long)s_bms.imbalance_mv);
            /* 이 여유가 곧 "접지가 몇 mV 밀리면 트립하는가" 다. node 는 x6 복원된
             * 값이므로, 접지에서 (여유/6) mV 만 밀려도 셀1 이 임계를 넘는다. */
            DBG_I(" OV margin %ld mV (= GND %ld mV 이동이면 트립)",
                  (long)(CFG_CELL_OV_MV - s_bms.cell_max_mv),
                  (long)((CFG_CELL_OV_MV - s_bms.cell_max_mv) * 1000L / CFG_DIV_SCALE_X1000));
            DBG_I(" pack %ldmV %ldmA  sensor_ready=%d",
                  (long)s_bms.pack_mv, (long)s_bms.pack_ma, (int)s_bms.sensor_ready);
            break;

        case 'h': case 'H':
            DBG_I("cmd: s=self check  v=adc dump  c=acs cal  i=ina init");
            DBG_I("     b=i2c recover+scan+reinit  k=cell cal  d=cal dump  x=cal reset");
            DBG_I("     g=charge gate 스냅샷  l=LED mute (접지 진단)  t=CAN frame trace");
            DBG_I("now: relay=%d%s",
                  (int)s_bms.relay_on,
                  (BRINGUP_S8_RELAY == 1) ? "" : " (output disabled)");
            DBG_I("CAN: %s  trace=%s",
                  bms_can_is_ready() ? "NORMAL 500k" : "NOT STARTED",
                  bms_can_get_trace() ? "ON" : "OFF");
            break;

        default:
            break;
        }
    }
}

/* ================================================================== */
/* ==================================================================
 *  부팅 자가진단 (POST) — 콘솔 's' 로 언제든 재실행
 *
 *  init 안에 흩어져 있던 "실패하면 sensor_ready=false" 판정을 한 곳에 모아
 *  한 줄씩 PASS/FAIL 로 찍는다. 이게 없으면 "무엇이 왜 안 잡혔는지" 를 부팅 로그
 *  스무 줄쯤 거슬러 올라가며 짜맞춰야 한다.
 *
 *  !! 여기서 하는 것은 존재/범위 확인뿐이고 Fault 판정은 하지 않는다.
 *     "Fault 는 bms_fault_check() 한 곳" 규칙을 깨지 않기 위해서다.
 *
 *  !! **측정 계통만 sensor_ready 를 내린다** (ADC / 셀 / 팩 센서).
 *     OLED·CAN 은 보고만 하고 게이트하지 않는다 — 표시장치가 없다고 배터리 감시를
 *     멈추는 것은 안전 방향이 반대다. CAN 두절은 이미 LINK_TIMEOUT 이 따로 잡는다.
 * ================================================================== */
static bool ap_post(const char *name, bool ok, const char *fmt_detail)
{
    if (ok) {
        DBG_I("  [ OK ] %-9s %s", name, fmt_detail);
    } else {
        DBG_E("  [FAIL] %-9s %s", name, fmt_detail);
    }
    return ok;
}

static void ap_selfcheck(void)
{
    char    det[64];
    bool    measure_ok = true;      /* sensor_ready 를 좌우하는 계통만 누적 */
    int32_t vdda;

    DBG_I("--- self check ---");

    /* --- ADC / VDDA : 모든 아날로그 측정의 기준이라 제일 먼저 본다 --- */
#if BRINGUP_ADC_HW
    vdda = hw_adc_get_vdda_mv();
    /* Vrefint 역산이 3.0~3.6V 밖이면 ADC 가 안 돌거나 기준이 깨진 것이다.
     * 이 값이 틀리면 셀 전압이 전부 같은 비율로 틀리므로 여기서 먼저 걸러야 한다. */
    (void)snprintf(det, sizeof(det), "VDDA %ld mV", (long)vdda);
    measure_ok &= ap_post("ADC", hw_adc_is_ready() && (vdda > 3000) && (vdda < 3600), det);
#else
    UNUSED_ARG(vdda);
    (void)ap_post("ADC", true, "(S1B/S2/S4 OFF - 미사용)");
#endif

    /* --- 셀 전압 범위 : 분압/배선 오류를 여기서 잡는다 --- */
#if (BRINGUP_S2_CELL_ADC == 1)
    {
        uint8_t i;
        bool    range_ok = true;
        for (i = 0; i < BMS_CELL_COUNT; i++) {
            if ((s_bms.cell_mv[i] < CFG_SELFCHK_CELL_MIN_MV) ||
                (s_bms.cell_mv[i] > CFG_SELFCHK_CELL_MAX_MV)) {
                range_ok = false;
            }
        }
        (void)snprintf(det, sizeof(det), "%ld/%ld/%ld/%ld mV (%d~%d)",
                       (long)s_bms.cell_mv[0], (long)s_bms.cell_mv[1],
                       (long)s_bms.cell_mv[2], (long)s_bms.cell_mv[3],
                       CFG_SELFCHK_CELL_MIN_MV, CFG_SELFCHK_CELL_MAX_MV);
        measure_ok &= ap_post("CELL", range_ok, det);
    }
#else
    (void)ap_post("CELL", true, "(S2 OFF - SELF_CHECK 는 실패한다)");
#endif

    /* --- 팩 전류 센서 : 션트 원값을 같이 찍는다 ---
     * 계산된 mA 만 보면 "정말 전류가 없다" 와 "션트가 포화됐다" 가 똑같이 보인다.
     * 원값(uV)이 ±81920 근처면 포화다 (0.1Ω 모듈에서 ±819mA). */
#if (BRINGUP_S3_PACK_SENSOR == 1)
    {
        bool ok = pack_sensor_is_ok() && pack_sensor_update();
        (void)snprintf(det, sizeof(det), "%s %ld mV %ld mA (shunt %ld uV)",
                       PACK_SENSOR_NAME, (long)pack_sensor_get_bus_mv(),
                       (long)pack_sensor_get_current_ma(),
                       (long)pack_sensor_get_shunt_uv());
        measure_ok &= ap_post("PACK", ok, det);
    }
#else
    (void)ap_post("PACK", true, "(S3 OFF - pack_mv 는 셀 ADC 합계)");
#endif

    /* --- NTC / ACS712 : 보고만 한다 (게이트하지 않음) ---
     * 온도가 전부 죽어도 bms_fault_check() 가 과온 판정을 건너뛸 뿐 감시는 계속된다. */
#if (BRINGUP_S4_NTC_ACS == 1)
    {
        uint8_t n = ntc_get_valid_count();
        (void)snprintf(det, sizeof(det), "%u/4 valid, max %ld.%ld C",
                       n, (long)(ntc_get_temp_max_c10() / 10),
                       (long)(ntc_get_temp_max_c10() % 10));
        (void)ap_post("NTC", (n > 0U), det);

        (void)snprintf(det, sizeof(det), "offset %ld uV, %ld mA",
                       (long)acs712_get_offset_uv(), (long)acs712_get_ma());
        (void)ap_post("ACS712", true, det);
    }
#else
    (void)ap_post("NTC/ACS", true, "(S4 OFF - temp=-999.9C, acs=0mA)");
#endif

    /* --- OLED / CAN : 보고 전용 --- */
#if (BRINGUP_S5_OLED == 1)
    (void)ap_post("OLED", hw_i2c_is_ready(HW_I2C_OLED, CFG_OLED_I2C_ADDR),
                  "I2C3 PA8/PC9 0x3C");
#endif
#if (BRINGUP_S6_LINK == 1)
    (void)snprintf(det, sizeof(det), "NORMAL 500k, TEC %u", bms_can_get_tec());
    (void)ap_post("CAN", bms_can_is_ready(), det);
#endif

    /* --- 릴레이 : 부팅 직후에는 반드시 열려 있어야 한다 ---
     * CFG_RELAY_ACTIVE_HIGH 가 실물과 반대면 여기가 통과해도 접점은 붙어 있다
     * (지령만 보므로). 그래서 실물 확인을 대신하지 못한다는 것을 문구에 남긴다. */
    (void)snprintf(det, sizeof(det), "cmd=%d%s", (int)s_bms.relay_on,
                   (BRINGUP_S8_RELAY == 1) ? " (LIVE - 접점 실물 확인 필요)"
                                           : " (S8=0, 출력 차단)");
    (void)ap_post("RELAY", !s_bms.relay_on, det);

    s_bms.sensor_ready = measure_ok;
    if (measure_ok) {
        DBG_I("--- self check PASS ---");
    } else {
        DBG_E("--- self check FAIL -> SELF_CHECK 에서 FAULT 로 간다 ---");
    }
}

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

/* 팩 센서(I2C1 PB6/PB7)와 OLED(I2C3 PA8/PC9)는 버스가 다르다.
 * hw_i2c_init() 이 둘을 한꺼번에 세우므로 스위치는 OR 로 둔다 — 한쪽만 켜도 두 버스가
 * 다 초기화되지만, 안 쓰는 버스는 라인 상태 한 줄만 찍고 끝이라 비용이 없다. */
#if (BRINGUP_S3_PACK_SENSOR == 1) || (BRINGUP_S5_OLED == 1)
    hw_i2c_init();
#endif

#if (BRINGUP_S3_PACK_SENSOR == 1)
    if (!pack_sensor_init()) {
        (void)hw_i2c_recover(HW_I2C_SENSOR);
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

    /* 자가진단 전에 한 번 수집해 둔다. 셀 전압 범위 검사가 블랙보드를 보므로,
     * 이게 없으면 전부 0 인 상태를 읽어 무조건 FAIL 이 난다. */
    ap_collect();
    ap_selfcheck();

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
