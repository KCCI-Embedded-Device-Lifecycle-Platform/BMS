/**
 * @file    bms_app.c
 * @brief   BMS 애플리케이션 최상위 (super-loop 스케줄러)
 *
 * @details 스케줄링 방식
 *      RTOS 를 쓰지 않고 협조형(cooperative) 주기 스케줄러를 쓴다.
 *      각 태스크가 짧고 블로킹이 없으므로 RTOS 오버헤드가 불필요하고,
 *      실행 순서가 정적으로 고정되어 타이밍 분석이 쉽다.
 *
 *      100ms : 셀 ADC, INA219, Fault 판단, FSM, 주요 프레임 송신
 *      500ms : NTC, ACS712, OLED, 셀 상세 프레임
 *      1000ms: SOC, 버전 프레임, 콘솔 요약 로그
 *
 *      "Fault 발생 후 100ms 이내 permit=0" 요구사항은
 *      측정 -> 판단 -> 송신이 같은 100ms 슬롯 안에서 연달아 일어나므로
 *      최악 지연이 1주기(100ms) + Fault 확정 지연으로 결정된다.
 */
#include "bms_app.h"
#include "dbg.h"
#include "hw_tick.h"
#include "hw_adc.h"
#include "hw_i2c.h"
#include "hw_gpio.h"
#include "hw_uart.h"

#include "cell_adc.h"
#include "ina219.h"
#include "ntc.h"
#include "acs712.h"
#include "oled.h"
#include "status_led.h"

#include "bms_fault.h"
#include "bms_soc.h"
#include "bms_state.h"
#include "bms_link.h"
#include "bms_ui.h"

/* 전역 블랙보드 : 파일 스코프 static 으로 캡슐화하고 getter 로만 노출 */
static bms_data_t s_bms;

static uint32_t s_t100, s_t500, s_t1000;

/* ==================================================================
 *  Fault 주입 ('f' 명령)
 *
 *  분압 저항 / 배터리 없이 FSM + Fault 경로를 검증하기 위한 시연용 경로다.
 *  ap_collect() 의 "맨 끝"에서 블랙보드를 덮어쓰므로,
 *  S2/S3 가 켜져 있어도 주입값이 실측값을 이긴다.
 *
 *  LINK_TIMEOUT 은 EVSE 가 없으면 1초 뒤 무조건 걸려 permit 을 0 으로 묶는다.
 *  그 억제는 주입과 분리해서 's_link_sim'('p' 토글) 이 담당한다.
 *  -> 'f/n' 은 셀/온도만, 'p' 는 링크만 건드린다 (서로 간섭 없음)
 * ================================================================== */
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

/* EVSE 하트비트 시뮬레이션 ('p' 토글).
 *
 *  하트비트는 CFG_LINK_TIMEOUT_MS(1초)만 유효하다. 그래서 'p' 를 "1회 갱신"으로
 *  두면 1.3초 뒤 LINK_TIMEOUT 이 다시 걸려 FAULT 로 되돌아간다 —
 *  실제 EVSE 가 100ms 마다 프레임을 보내는 상황을 흉내내려면 계속 갱신해야 한다.
 *  그래서 'p' 는 토글이고, 켜져 있는 동안 100ms 슬롯마다 갱신한다.
 *  (LINK_TIMEOUT 자체를 시연하려면 'p' 로 다시 끈다) */
static bool s_link_sim;

static const char *inject_name(uint8_t m)
{
    static const char *name[INJ_MAX] = {
        "OFF", "NORMAL", "CELL_OV", "CELL_UV", "IMBALANCE", "OVER_TEMP"
    };
    return (m < (uint8_t)INJ_MAX) ? name[m] : "?";
}

/**
 * @brief  주입 모드에 맞춰 블랙보드를 덮어쓴다.
 * @note   덮어쓰는 범위는 "실측 소스가 없는 항목"으로 제한한다.
 *         S3(INA219) 가 켜져 있으면 pack_mv / pack_ma 는 손대지 않는다.
 *         그래야 Fault 시연 중에도 OLED 와 1초 로그에서 INA219 실측값이 보인다.
 *         (셀 합 14.8V 든 INA219 실측 0V 든 PACK_OV 임계 16.8V 아래라 안전하다)
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

#if (BRINGUP_S3_INA219 == 1)
    UNUSED_ARG(sum);                /* 팩 전압/전류는 INA219 실측을 그대로 둔다 */
#else
    s_bms.pack_mv = sum;
    s_bms.pack_ma = 0;
#endif
#if (BRINGUP_S4_NTC_ACS == 0)
    s_bms.acs_ma = s_bms.pack_ma;   /* 둘이 어긋나면 SENSOR_ERR 가 끼어든다 */
#endif

    if (s_inject == (uint8_t)INJ_OVER_TEMP) {
        s_bms.temp_c10 = CFG_OVER_TEMP_C10 + 50;    /* 데모 40.0C -> 45.0C */
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

#if (BRINGUP_S3_INA219 == 1)
    if (ina219_update()) {
        s_bms.pack_mv = ina219_get_bus_mv();     /* INA219 값을 우선 채택 */
        s_bms.pack_ma = ina219_get_current_ma();
    }
#endif

    /* 주입은 항상 마지막 : 실측값을 덮어써야 시연이 성립한다 */
    ap_inject_apply();
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

    status_led_update();

#if (BRINGUP_S6_LINK == 1)
    bms_link_send_main(&s_bms);
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
#endif

    /* --- 콘솔 요약 로그 --- */
#if (BRINGUP_S1_LED_DBG == 1)
    DBG_I("[%s] C:%ld/%ld/%ld/%ld mV  PACK:%ldmV %ldmA  T:%ld.%ldC  SOC:%u%%  FLT:0x%04X P:%d",
          bms_fsm_state_name(s_bms.state),
          (long)s_bms.cell_mv[0], (long)s_bms.cell_mv[1],
          (long)s_bms.cell_mv[2], (long)s_bms.cell_mv[3],
          (long)s_bms.pack_mv, (long)s_bms.pack_ma,
          (long)(s_bms.temp_c10 / 10), (long)(s_bms.temp_c10 % 10),
          s_bms.soc, s_bms.fault, (int)s_bms.charge_permit);
#endif
}

/* ------------------------------------------------------------------ */
static void ap_console(void)
{
    uint8_t ch;

    while (hw_uart_get_byte(&ch)) {
        switch (ch) {
        case 'p': case 'P':                     /* EVSE 하트비트 흉내 on/off */
            s_link_sim = !s_link_sim;
            if (s_link_sim) {
                bms_fault_notify_link();
            }
            DBG_W("link heartbeat sim -> %s", s_link_sim ? "ON" : "OFF");
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

        case 'i': case 'I':                     /* INA219 재초기화 */
            (void)ina219_init();
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

        case 'h': case 'H':
            DBG_I("cmd: f=fault inject  n=inject off  p=heartbeat sim toggle");
            DBG_I("     v=adc dump  c=acs cal  i=ina init");
            DBG_I("now: inject=%s  link_sim=%s",
                  inject_name(s_inject), s_link_sim ? "ON" : "OFF");
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
    DBG_I("S3 INA219    : %d", BRINGUP_S3_INA219);
    DBG_I("S4 NTC/ACS   : %d", BRINGUP_S4_NTC_ACS);
    DBG_I("S5 OLED      : %d", BRINGUP_S5_OLED);
    DBG_I("S6 LINK      : %d", BRINGUP_S6_LINK);
    DBG_I("S7 FSM/FAULT : %d", BRINGUP_S7_FSM_FAULT);

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

/* OLED 를 SPI 로 쓰면 I2C 버스는 INA219 전용이다 */
#if (BRINGUP_S3_INA219 == 1) || \
    ((BRINGUP_S5_OLED == 1) && (CFG_OLED_IFACE == CFG_OLED_IFACE_I2C))
    hw_i2c_init();
#endif

#if (BRINGUP_S3_INA219 == 1)
    if (!ina219_init()) {
        (void)hw_i2c_recover();
        if (!ina219_init()) {
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
    /* 셀 ADC 가 없으면 cell_mv 가 전부 0 이라 SELF_CHECK 가 곧바로 실패하고
     * FAULT 에 갇힌다. 시연을 시작할 수 있도록 NORMAL 주입으로 출발한다. */
    s_inject = (uint8_t)INJ_NORMAL;
    DBG_W("!! CELL ADC OFF -> fault injection ON (fake cell data) !!");
    DBG_W("!! 'f' 로 OV/UV/IMBAL/OT 순환, 'n' 으로 해제 !!");
#endif

#if (BRINGUP_S6_LINK == 0)
    /* EVSE 가 없으면 1초 뒤 LINK_TIMEOUT 이 무조건 걸려 FAULT 에 갇힌다.
     * 링크 이외의 동작을 먼저 보려면 하트비트 시뮬레이션이 켜져 있어야 한다.
     * LINK_TIMEOUT 자체를 시연할 때는 'p' 로 끈다. */
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
    /* 수신은 주기 태스크가 아니라 매 바퀴 돈다.
     * CAN RX FIFO 는 3단뿐이라 100ms 슬롯에 묶으면 EVSE 가 빠르게
     * 보낼 때 넘친다. (UART 백엔드에서는 아무 일도 하지 않는다) */
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
