/**
 * @file    bms_can.c
 * @brief   bxCAN(CAN1) 송수신 계층 - 500kbps, 표준 11bit ID
 *
 * @details 왜 인터럽트가 아니라 폴링인가?
 *      RX FIFO0 는 하드웨어가 3단까지 자동으로 쌓아준다. EVSE 는 100ms
 *      주기로 보내고 메인 루프는 그보다 훨씬 빠르게 도므로, 폴링만으로
 *      오버런이 나지 않는다. 인터럽트를 켜면 ISR 안에서 파라미터 쓰기
 *      같은 로직이 돌아 "ISR 최소 연산" 원칙이 깨지고, s_evse_* 를
 *      메인 루프와 공유하는 순간 임계영역까지 필요해진다.
 *      -> CubeMX 가 CAN1_RX0_IRQn 을 NVIC 에 켜 두었지만
 *         HAL_CAN_ActivateNotification() 을 부르지 않으므로
 *         CAN_IER 의 인터럽트 인에이블 비트가 0 이라 실제로 뜨지 않는다.
 *
 * @note  루프백 모드 (.ioc : CAN1.Mode = CAN_MODE_LOOPBACK)
 *      트랜시버(SN65HVD230 등)와 상대 노드가 없으면 Normal 모드에서는
 *      ACK 를 못 받아 AutoRetransmission 이 무한 재전송을 돌고
 *      메일박스가 3개 다 차서 bms_can_send() 가 계속 false 를 뱉는다.
 *      루프백에서는 송신 프레임이 버스로 나가지 않고 자기 RX FIFO 로
 *      되돌아오므로, 상대 노드 없이 송신/수신/디스패치 경로를 전부
 *      검증할 수 있다. 실제 EVSE 를 붙일 때 CubeMX 에서 Normal 로 바꾼다.
 *
 * @note  수신 바이트 맵 (EVSE 파트와 합의한 규격)
 *      0x200 EVSE_STATUS      Data[0] state / [1] relay_on / [2] connected
 *      0x201 EVSE_CHARGE_REQ  Data[0] 0=중지 1=충전 요청
 *      0x202 EVSE_FAULT       Data[0] EVSE Fault 코드
 *      0x203 BMS_OTA_ENTER    (미지원 - 명시적 거부 응답)
 *      0x205 PARAM_WRITE      Data[0] Param ID / [1:2] int16 LE / [3] Magic
 *      0x105 BMS_RESP         Data[0] 코드 / [1] 상세 / [2:3] 값 LE  (송신)
 */
#include "bms_can.h"
#include "bms_fault.h"
#include "dbg.h"

#if (CFG_LINK_TRANSPORT == CFG_LINK_TRANSPORT_CAN)

extern CAN_HandleTypeDef hcan1;         /* CubeMX 생성 (can.c) */

static bool    s_ready;                 /* HAL_CAN_Start 성공 여부 */
static bool    s_evse_charge_req;
static bool    s_evse_relay_on;
static bool    s_evse_connected;
static uint8_t s_evse_fault;

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
    s_evse_fault      = 0U;

    /* 필터 0 : 마스크 전체 0 = "모든 ID 통과".
     * ID 별 하드웨어 필터를 거는 대신 소프트웨어 switch 로 분기한다.
     * 노드가 몇 개 안 되고 프레임도 드물어서 CPU 부담이 없고,
     * ID 를 추가할 때 필터 뱅크를 다시 계산하지 않아도 된다. */
    f.FilterBank           = 0U;
    f.FilterMode           = CAN_FILTERMODE_IDMASK;
    f.FilterScale          = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh         = 0x0000U;
    f.FilterIdLow          = 0x0000U;
    f.FilterMaskIdHigh     = 0x0000U;
    f.FilterMaskIdLow      = 0x0000U;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation     = CAN_FILTER_ENABLE;
    /* F446 은 CAN1/CAN2 가 필터 뱅크 28개를 나눠 쓴다.
     * CAN2 를 안 쓰더라도 이 값을 넘겨야 파라미터 검증을 통과한다. */
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
    DBG_I("CAN1 started (500kbps, PB8 RX / PB9 TX)");
    return true;
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

    /* 메일박스가 다 찼는데 AddTxMessage 를 부르면 HAL_ERROR 만 나고
     * 블로킹은 아니지만, 여기서 먼저 걸러야 원인이 로그로 드러난다.
     * Normal 모드 + 상대 노드 없음이면 ACK 오류로 늘 이 상태가 된다. */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        return false;
    }

    h.StdId              = (uint32_t)std_id;
    h.ExtId              = 0U;
    h.IDE                = CAN_ID_STD;
    h.RTR                = CAN_RTR_DATA;
    h.DLC                = (uint32_t)dlc;
    h.TransmitGlobalTime = DISABLE;

    return (HAL_CAN_AddTxMessage(&hcan1, &h, (uint8_t *)p_data, &mailbox) == HAL_OK);
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

    /* Magic 바이트로 오조작/노이즈 프레임을 1차 차단한다.
     * 임계값을 바꾸는 명령이라 "우연히 맞는" 경우를 최대한 줄여야 한다. */
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
        /* 정의만 되어 있고 아직 안 붙인 파라미터는 조용히 무시하지 않고
         * NAK 로 돌려준다. 상위가 "먹혔는지" 헷갈리지 않게 한다. */
        bms_can_send_resp(BMS_RESP_PARAM_NAK, p_d[0], 0U);
        DBG_W("param id 0x%02X not supported", p_d[0]);
        return;
    }

    /* 요청값이 아니라 "실제 적용된 값"을 에코백 -> 상위가 클램프를 감지한다 */
    bms_can_send_resp(BMS_RESP_PARAM_ACK, p_d[0], (uint16_t)applied);
}

/* ==================================================================
 *  수신 디스패치
 * ================================================================== */
static void bms_can_dispatch(uint16_t id, const uint8_t *p_d, uint8_t dlc)
{
    switch (id) {

    case CFG_CAN_ID_EVSE_STATUS:            /* 하트비트 겸용 */
        bms_fault_notify_link();
        s_evse_relay_on  = (dlc > 1U) && (p_d[1] != 0U);
        s_evse_connected = (dlc > 2U) && (p_d[2] != 0U);
        break;

    case CFG_CAN_ID_EVSE_CHARGE_REQ:
        bms_fault_notify_link();
        s_evse_charge_req = (dlc > 0U) && (p_d[0] != 0U);
        break;

    case CFG_CAN_ID_EVSE_FAULT:             /* 표시/로그용 */
        bms_fault_notify_link();
        s_evse_fault = (dlc > 0U) ? p_d[0] : 0U;
        break;

    case CFG_CAN_ID_BMS_OTA_ENTER:
        /* BMS 자체 OTA(CAN 중계)는 이번 범위 밖이다.
         * 무시해 버리면 상위가 응답을 기다리며 멈추므로,
         * 명시적으로 거부해 "프로토콜은 살아있다"를 알린다. */
        bms_can_send_resp(BMS_RESP_OTA_REJECT, BMS_REJECT_NOT_SUPPORTED, 0U);
        DBG_W("OTA enter requested -> rejected (not supported)");
        break;

    case CFG_CAN_ID_PARAM_WRITE:
        bms_can_handle_param(p_d, dlc);
        break;

    case CFG_CAN_ID_MAIN:
        /* 루프백에서는 자기가 보낸 0x100 이 그대로 되돌아온다.
         * 상대 노드 없이도 LINK_TIMEOUT 이 풀려야 나머지 FSM 을
         * 벤치에서 확인할 수 있으므로 자신을 하트비트로 인정한다.
         * (Normal 모드에서는 자기 프레임이 되돌아오지 않으므로
         *  이 분기는 타지 않는다) */
        bms_fault_notify_link();
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
        if (h.IDE != CAN_ID_STD) {
            continue;                       /* 확장 ID 는 이 규격에 없다 */
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
uint8_t bms_can_evse_fault(void)      { return s_evse_fault;      }

#else   /* CFG_LINK_TRANSPORT == CFG_LINK_TRANSPORT_UART */

/* UART 백엔드일 때도 링크가 되도록 빈 구현을 남긴다.
 * 호출부(bms_app.c)에 #if 를 흩뿌리지 않기 위함이다. */
bool    bms_can_init(void)                                    { return false; }
bool    bms_can_send(uint16_t id, const uint8_t *p, uint8_t n)
{
    UNUSED_ARG(id); UNUSED_ARG(p); UNUSED_ARG(n);
    return false;
}
void    bms_can_poll_rx(void)         { }
bool    bms_can_evse_charge_req(void) { return false; }
bool    bms_can_evse_relay_on(void)   { return false; }
bool    bms_can_evse_connected(void)  { return false; }
uint8_t bms_can_evse_fault(void)      { return 0U;    }

#endif  /* CFG_LINK_TRANSPORT */
