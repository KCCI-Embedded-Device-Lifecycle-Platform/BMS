/**
 * @file    bms_can.c
 * @brief   bxCAN(CAN1) 송수신 - 500kbps, 표준 11bit ID
 *
 * 인터럽트가 아니라 폴링이다. RX FIFO0 가 하드웨어로 3단 쌓이고 EVSE 는 100ms 주기라
 * 메인 루프 폴링만으로 오버런이 나지 않는다. 인터럽트를 켜면 ISR 안에서 파라미터 쓰기
 * 같은 로직이 돌고, s_evse_* 를 메인 루프와 공유하므로 임계영역까지 필요해진다.
 * (CubeMX 가 CAN1_RX0_IRQn 을 NVIC 에 켜 두었지만 ActivateNotification 을 부르지
 *  않으므로 CAN_IER 이 0 이라 실제로 뜨지 않는다 — 의도된 상태다)
 *
 * 모드는 NORMAL 고정이고, .ioc 가 아니라 이 파일이 못 박는다 (아래 mode override).
 * 한때 CFG_CAN_MODE_LOOPBACK 으로 루프백을 고를 수 있었지만 실버스 검증 후 제거했다 —
 * 루프백은 자기 0x100 을 하트비트로 인정하는 예외가 필요해서, 남겨 두면 실버스 코드에
 * "EVSE 무응답을 못 잡는 것처럼 보이는" 분기가 섞인다.
 *
 * 트랜시버 SN65HVD230 (EVSE 와 동일 부품) 주의사항:
 *   - VCC 는 3.3V. 5V 를 주면 RXD 가 5V 로 나오고 CANH/CANL 코먼모드가 어긋난다.
 *     (3.3V 네이티브라 TJA1050 의 TXD 임계 마진 문제가 아예 없다 — 그래서 이 부품이다)
 *   - Rs(8번) 핀은 GND 또는 10k 로. 뜨면 standby 로 들어가 송신이 에러 없이 죽는다.
 *   - 종단 120옴은 모듈 내장. 전원 OFF 에서 CANH-CANL = 60옴이면 2노드 정상.
 *     외부 저항을 더 달지 말 것.
 *   - MCU TX -> 모듈 D, MCU RX -> 모듈 R (UART 처럼 교차하지 않는다). 공통 GND 필수.
 *
 * 수신 바이트 맵 (EVSE 파트와 합의):
 *   0x200 Data[0] state / [1] relay_on / [2] connected / [3] E-Stop
 *   0x201 Data[0] 0=중지 1=충전 요청
 *   0x202 Data[0] EVSE Fault 코드
 *   0x203 OTA 진입 (미지원 - 명시적 거부)
 *   0x205 Data[0] Param ID / [1:2] int16 LE / [3] Magic
 *   0x105 Data[0] 코드 / [1] 상세 / [2:3] 값 LE  (송신)
 */
#include "bms_can.h"
#include "bms_fault.h"
#include "dbg.h"

extern CAN_HandleTypeDef hcan1;         /* CubeMX 생성 (can.c) */

static bool    s_ready;                 /* HAL_CAN_Start 성공 여부 */
static bool    s_evse_charge_req;
static bool    s_evse_relay_on;
static bool    s_evse_connected;
static bool    s_evse_estop;
static uint8_t s_evse_fault;

/* --- 벤치 계측 (bms_can_log_stats / 트레이스) --- */
static bool     s_trace;                /* 프레임 단위 hexdump on/off */
static uint32_t s_tx_cnt;               /* 메일박스에 실은 횟수         */
static uint32_t s_tx_drop;              /* 메일박스 포화로 못 실은 횟수 */
static uint32_t s_rx_cnt;               /* FIFO 에서 꺼낸 횟수          */
static uint32_t s_rx_drop;              /* 확장ID / RTR 로 버린 횟수    */
static bool     s_was_bus_off;          /* Bus-Off 엣지 검출 */

/** @brief LEC -> 이름. ACK 가 제일 중요하다 (상대 없음/비트레이트/종단이 전부 여기로 나온다). */
static const char *can_lec_name(uint32_t lec)
{
    static const char *name[8] = {
        "-", "STUFF", "FORM", "ACK", "BIT_REC", "BIT_DOM", "CRC", "SW"
    };
    return name[lec & 0x07U];
}

/**
 * @brief  트레이스 한 줄 (방향/ID 태그 + hexdump)
 * @note   115200bps 에서 한 줄이 약 3.5ms 블로킹이다. 100ms 슬롯은 2프레임=7ms 라 여유가
 *         있지만 OLED(23ms)와 겹치는 500ms 슬롯이 제일 빡빡하다 —
 *         타이밍이 의심스러우면 트레이스부터 끄고 볼 것.
 */
static void can_trace(const char *p_dir, uint16_t id, const uint8_t *p_d, uint8_t dlc)
{
    char tag[20];

    (void)snprintf(tag, sizeof(tag), "CAN %s %03X", p_dir, (unsigned int)id);
    dbg_hexdump(tag, p_d, dlc);
}

/**
 * @brief  실제 비트레이트를 계산해 로그로 남기고, 규격과 1% 이상 다르면 에러로 알린다.
 * @note   .ioc 의 Prescaler/BS1/BS2 는 "그때의 APB1" 기준 값이다. RCC 트리를 건드리면
 *         핀도 코드도 그대로인데 비트레이트만 조용히 틀어지고, 증상은 "CAN 이 안 붙는다"
 *         로만 나타난다 (실제로 APB1 이 어긋나 571kbps 가 나온 적 있다).
 *         -> CAN 문제는 부팅 로그의 이 줄부터 본다.
 */
static void can_report_bitrate(void)
{
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t ts1   = ((hcan1.Init.TimeSeg1 >> CAN_BTR_TS1_Pos) & 0x0FU) + 1U;
    uint32_t ts2   = ((hcan1.Init.TimeSeg2 >> CAN_BTR_TS2_Pos) & 0x07U) + 1U;
    uint32_t tq    = 1U + ts1 + ts2;        /* SYNC_SEG 1TQ 포함 */
    uint32_t baud  = pclk1 / (hcan1.Init.Prescaler * tq);

    DBG_I("CAN1 started (PB8 RX / PB9 TX, PCLK1=%luHz, %luTQ, %lubps, SP=%lu%%)",
          (unsigned long)pclk1, (unsigned long)tq, (unsigned long)baud,
          (unsigned long)(((1U + ts1) * 100U) / tq));

    if (ABS_DIFF(baud, (uint32_t)CFG_CAN_BITRATE_BPS) > ((uint32_t)CFG_CAN_BITRATE_BPS / 100U)) {
        DBG_E("!! CAN bitrate %lubps != spec %lubps - RCC(APB1) 또는 .ioc 타이밍 확인 !!",
              (unsigned long)baud, (unsigned long)CFG_CAN_BITRATE_BPS);
    }
}

/* ==================================================================
 *  초기화
 * ================================================================== */
bool bms_can_init(void)
{
    CAN_FilterTypeDef f = {0};

    s_ready           = false;
    s_evse_charge_req = false;
    s_evse_relay_on   = false;
    s_evse_connected  = false;
    s_evse_estop      = false;
    s_evse_fault      = 0U;
    s_tx_cnt          = 0U;
    s_tx_drop         = 0U;
    s_rx_cnt          = 0U;
    s_rx_drop         = 0U;
    s_was_bus_off     = false;

    /* 동작 모드를 bms_cfg.h 로 못 박는다.
     * MX_CAN1_Init() 이 이미 .ioc 값으로 초기화를 끝냈고, 모드는 초기화 시점에만
     * 반영되므로 핸들만 고쳐서는 안 바뀐다 -> 재초기화한다 (아직 Start 전이라 안전). */
    if (hcan1.Init.Mode != CAN_MODE_NORMAL) {
        hcan1.Init.Mode = CAN_MODE_NORMAL;
        if (HAL_CAN_Init(&hcan1) != HAL_OK) {
            DBG_E("CAN re-init failed (mode override)");
            return false;
        }
        DBG_W("CAN mode overridden -> NORMAL (.ioc 값 무시)");
    }

    /* 필터 0 : 마스크 전체 0 = 모든 ID 통과. 분기는 소프트웨어 switch 로 한다.
     * 노드/프레임이 적어 CPU 부담이 없고, ID 를 추가할 때 뱅크를 다시 계산하지 않아도 된다.
     * (필터를 하나도 등록하지 않으면 bxCAN 은 수신을 전부 버린다) */
    f.FilterBank           = 0U;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000U;
    f.FilterIdLow          = 0x0000U;
    f.FilterMaskIdHigh     = 0x0000U;
    f.FilterMaskIdLow      = 0x0000U;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = CAN_FILTER_ENABLE;
    /* F446 은 CAN1/CAN2 가 뱅크 28개를 나눠 쓴다. CAN2 를 안 써도 이 값이 있어야
     * 파라미터 검증을 통과한다. */
    f.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
        DBG_E("CAN filter config failed");
        return false;
    }

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        DBG_E("CAN start failed");
        return false;
    }

    s_ready = true;
    can_report_bitrate();

    /* NORMAL 은 "혼자 켜면 실패하는 것이 정상" 인 모드다. 부팅 로그에 박아 두지 않으면
     * 나중에 Bus-Off 를 보고 멀쩡한 배선을 뜯게 된다. */
    DBG_W("CAN NORMAL : 상대 노드가 ACK 를 줘야 송신이 성립한다.");
    DBG_W("  혼자 켜면 TEC 증가 -> Bus-Off 가 정상 (상대 연결 시 ABOM 자동 복귀)");
    DBG_W("  확인 순서 : (1) Rs 핀 GND  (2) VCC 3.3V  (3) CANH-CANL 60ohm  (4) 공통 GND");
    DBG_I("  console : t=frame trace  (1초마다 CAN 요약 로그가 자동으로 나온다)");
    return true;
}

/* ==================================================================
 *  벤치 계측 / 트레이스
 * ================================================================== */
bool bms_can_is_ready(void)              { return s_ready; }
void bms_can_set_trace(bool on)          { s_trace = on;   }
bool bms_can_get_trace(void)             { return s_trace; }

uint16_t bms_can_get_tec(void)
{
    if (!s_ready) {
        return 0U;
    }
    return (uint16_t)((hcan1.Instance->ESR & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos);
}

void bms_can_log_stats(void)
{
    uint32_t esr;
    uint32_t tec, rec, lec;
    bool     bus_off;

    if (!s_ready) {
        DBG_W("CAN not started (BRINGUP_S6_LINK / init 실패 확인)");
        return;
    }

    /* TEC/REC 는 CAN 규격의 하드웨어 에러 카운터라 소프트웨어 카운터보다 정확하다:
     *   tx 증가 + TEC 0      -> 정상 (상대가 ACK 를 준다)
     *   tx 증가 + TEC 증가   -> 버스로는 나가지만 아무도 ACK 안 함
     *   tx_drop 만 증가      -> 메일박스 포화 = 재전송에 갇힘 */
    esr     = hcan1.Instance->ESR;
    tec     = (esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
    rec     = (esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
    lec     = (esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
    bus_off = ((esr & CAN_ESR_BOFF) != 0U);

    DBG_I("CAN NORM tx:%lu/d%lu rx:%lu/d%lu TEC:%lu REC:%lu LEC:%s%s",
          (unsigned long)s_tx_cnt,  (unsigned long)s_tx_drop,
          (unsigned long)s_rx_cnt,  (unsigned long)s_rx_drop,
          (unsigned long)tec, (unsigned long)rec, can_lec_name(lec),
          ((esr & CAN_ESR_EPVF) != 0U) ? " ERR-PASSIVE" : "");

    /* Bus-Off 는 매초 찍으면 로그가 도배된다. 상태가 바뀔 때만 알린다. */
    if (bus_off != s_was_bus_off) {
        if (bus_off) {
            DBG_E("!! CAN BUS-OFF (TEC>255) - 상대 노드/비트레이트/종단 확인 !!");
        } else {
            DBG_W("CAN Bus-Off 복귀 (ABOM)");
        }
        s_was_bus_off = bus_off;
    }
}

/* ==================================================================
 *  송신
 * ================================================================== */
bool bms_can_send(uint16_t std_id, const uint8_t *p_data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef h;
    uint32_t            mailbox;

    if ((!s_ready) || (p_data == NULL) || (dlc > 8U)) {
        return false;
    }

    /* 메일박스 포화를 먼저 걸러야 원인이 카운터로 드러난다.
     * NORMAL + 상대 노드 없음이면 ACK 오류로 늘 이 상태가 된다. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        s_tx_drop++;
        return false;
    }

    h.StdId              = (uint32_t)std_id;
    h.ExtId              = 0U;
    h.IDE                = CAN_ID_STD;
    h.RTR                = CAN_RTR_DATA;
    h.DLC                = (uint32_t)dlc;
    h.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)p_data, &mailbox) != HAL_OK) {
        s_tx_drop++;
        return false;
    }

    /* 이 카운터는 "메일박스에 실었다" 지 "버스로 나갔다" 가 아니다.
     * ACK 를 못 받으면 하드웨어가 재전송 중일 수 있으므로 log_stats 의 TEC 와 같이 볼 것. */
    s_tx_cnt++;
    if (s_trace) {
        can_trace("TX", std_id, p_data, dlc);
    }
    return true;
}

/* ==================================================================
 *  응답 프레임 (0x105)
 * ================================================================== */
static void bms_can_send_resp(uint8_t code, uint8_t detail, uint16_t value)
{
    uint8_t d[4];

    d[0] = code;
    d[1] = detail;
    d[2] = (uint8_t)(value & 0xFFU);
    d[3] = (uint8_t)(value >> 8);
    (void)bms_can_send(CFG_CAN_ID_BMS_RESP, d, 4U);
}

/* ==================================================================
 *  0x205 파라미터 쓰기
 * ================================================================== */
static void bms_can_handle_param(const uint8_t *p_d, uint8_t dlc)
{
    int16_t v;
    int32_t applied;

    /* 임계값을 바꾸는 명령이라 "우연히 맞는" 프레임을 최대한 줄여야 한다 -> Magic 검사. */
    if ((dlc < 4U) || (p_d[3] != PARAM_MAGIC)) {
        bms_can_send_resp(BMS_RESP_PARAM_NAK, (dlc > 0U) ? p_d[0] : 0U, 0U);
        DBG_W("param write rejected (dlc=%u, magic mismatch)", dlc);
        return;
    }

    v = (int16_t)((uint16_t)p_d[1] | ((uint16_t)p_d[2] << 8));

    switch (p_d[0]) {
    case PARAM_ID_OVER_TEMP:
        applied = bms_fault_set_ot_threshold((int32_t)v);
        break;

    default:
        /* 아직 안 붙인 파라미터도 조용히 무시하지 않고 NAK 로 돌려준다.
         * 상위가 "먹혔는지" 헷갈리지 않게 한다. */
        bms_can_send_resp(BMS_RESP_PARAM_NAK, p_d[0], 0U);
        DBG_W("param id 0x%02X not supported", p_d[0]);
        return;
    }

    /* 요청값이 아니라 실제 적용값을 에코백 -> 상위가 클램프를 감지한다 */
    bms_can_send_resp(BMS_RESP_PARAM_ACK, p_d[0], (uint16_t)applied);
}

/* ==================================================================
 *  수신 디스패치
 * ================================================================== */
static void bms_can_dispatch(uint16_t id, const uint8_t *p_d, uint8_t dlc)
{
    switch (id) {

    case CFG_CAN_ID_EVSE_STATUS:            /* 하트비트 겸용, DLC 4 */
        if (dlc != 4U) {
            s_rx_drop++;
            break;
        }
        bms_fault_notify_link();
        s_evse_relay_on  = (p_d[1] != 0U);
        s_evse_connected = (p_d[2] != 0U);
        /* E-Stop 은 정보가 없으면 "안 눌림" 으로 본다. 반대로 두면 DLC 가 짧은 구형
         * 프레임을 비상정지로 오인해 충전이 영구히 막힌다. 진짜 비상정지의 1차 방어선은
         * EVSE 자기 릴레이이고, 여기는 BMS 릴레이를 같이 여는 2차 방어선이다. */
        s_evse_estop     = (p_d[3] != 0U);
        break;

    case CFG_CAN_ID_EVSE_CHARGE_REQ:
        if (dlc != 1U) {
            s_rx_drop++;
            break;
        }
        bms_fault_notify_link();
        s_evse_charge_req = (p_d[0] != 0U);
        break;

    case CFG_CAN_ID_EVSE_FAULT:             /* 표시/로그용 */
        if (dlc != 1U) {
            s_rx_drop++;
            break;
        }
        bms_fault_notify_link();
        s_evse_fault = p_d[0];
        break;

    case CFG_CAN_ID_BMS_OTA_ENTER:
        /* BMS 자체 OTA 는 범위 밖이다. 무시하면 상위가 응답을 기다리며 멈추므로
         * 명시적으로 거부해 "프로토콜은 살아있다" 를 알린다. */
        bms_can_send_resp(BMS_RESP_OTA_REJECT, BMS_REJECT_NOT_SUPPORTED, 0U);
        DBG_W("OTA enter requested -> rejected (not supported)");
        break;

    case CFG_CAN_ID_PARAM_WRITE:
        bms_can_handle_param(p_d, dlc);
        break;


    default:
        break;
    }
}

void bms_can_poll_rx(void)
{
    CAN_RxHeaderTypeDef h;
    uint8_t             data[8];

    if (!s_ready) {
        return;
    }

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0U) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &h, data) != HAL_OK) {
            break;
        }
        s_rx_cnt++;

        /* 확장 ID 는 이 규격에 없다. RTR(리모트)도 반드시 버린다 — DLC 는 실려 오는데
         * 데이터 필드가 없어 data[] 에 직전 프레임 잔재가 남는다. 0x200 짜리 리모트
         * 프레임 하나로 하트비트가 위조되고 relay_on/estop 까지 쓰레기로 갱신된다.
         * 버스에 Gateway 나 진단 장비가 붙으면 실제로 생길 수 있는 경로다. */
        if ((h.IDE != CAN_ID_STD) || (h.RTR != CAN_RTR_DATA)) {
            s_rx_drop++;
            /* 버린 프레임도 트레이스에는 남긴다. "보냈는데 반응이 없다" 의 원인이
             * 확장 ID / 리모트 프레임인 경우를 즉시 구분하기 위해서다. */
            if (s_trace) {
                can_trace("rx-drop", (uint16_t)h.StdId, data, (uint8_t)h.DLC);
            }
            continue;
        }

        if (s_trace) {
            can_trace("RX", (uint16_t)h.StdId, data, (uint8_t)h.DLC);
        }
        bms_can_dispatch((uint16_t)h.StdId, data, (uint8_t)h.DLC);
    }
}

/* ==================================================================
 *  EVSE 상태 getter
 * ================================================================== */
bool    bms_can_evse_charge_req(void) { return s_evse_charge_req; }
bool    bms_can_evse_relay_on(void)   { return s_evse_relay_on;   }
bool    bms_can_evse_connected(void)  { return s_evse_connected;  }
bool    bms_can_evse_estop(void)      { return s_evse_estop;      }
uint8_t bms_can_evse_fault(void)      { return s_evse_fault;      }

/**
 * @brief  EVSE 수신 상태를 전부 버린다 (링크 두절 확정 시)
 *
 * @note   s_evse_* 는 "마지막으로 받은 값" 이라 아무도 지우지 않으면 링크가 끊겨도 남는다.
 *         0x201 을 이벤트로 한 번만 보내는 EVSE 구현이면 이렇게 된다:
 *           0x201=1 -> CHARGE_READY -> EVSE 전원 차단 -> LINK_TIMEOUT -> permit=0 (여기까진 안전)
 *           -> EVSE 재기동 -> 0x200 도착 -> 타임아웃 해제
 *           -> charge_req 가 아직 1 -> 아무도 요청하지 않았는데 릴레이 폐로   <-- 사고
 *
 *         "0x201 을 주기 반복한다" 는 합의가 있어도 여기서 지운다.
 *         상대가 규약을 어겨도 BMS 혼자 안전해야 한다.
 */
void bms_can_clear_evse_state(void)
{
    s_evse_charge_req = false;
    s_evse_relay_on   = false;
    s_evse_connected  = false;
    s_evse_estop      = false;
    s_evse_fault      = 0U;
}
