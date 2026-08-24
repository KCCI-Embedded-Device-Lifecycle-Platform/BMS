/**
 * @file    bms_link.c
 * @brief   EVSE 통신 페이로드 패킹
 *
 * 파일이 둘로 나뉘어 있다:
 *   (1) pack_xxx()        : 프레임 바이트 페이로드 생성
 *   (2) link_send_frame() : 그 바이트를 실제로 내보내는 한 곳
 * 프레임 포맷을 바꿀 때 고칠 곳이 (1) 하나로 모이는 것이 이 분리의 목적이다.
 * 실제 송신은 bms_can.c 가 한다.
 *
 * 한때 UART ASCII 백엔드가 같이 있었다 (1단계 보드 F411RE 에 bxCAN 이 없었다).
 * CAN 이 실보드로 검증된 뒤 제거했다 — 바이트 맵이 같아도 두 경로를 계속 맞춰 두는
 * 비용이 실익보다 컸다. 프레임을 눈으로 봐야 하면 콘솔 't' 트레이스를 쓴다.
 *
 * 0x100 바이트 맵 (DLC 8, EVSE 파트와 합의):
 *   [0:1] Pack V 0.01V LE / [2:3] Pack I 0.01A int16 LE / [4] SOC%
 *   [5] Charge Permit / [6] BMS State / [7] Fault 하위 8bit
 */
#include "bms_link.h"
#include "bms_fault.h"
#include "bms_can.h"
#include "dbg.h"

/* ==================================================================
 *  전송 : 모든 프레임이 이 한 곳을 지난다
 * ================================================================== */
static void link_send_frame(uint16_t can_id, const uint8_t *p_data, uint8_t dlc)
{
#if (BRINGUP_S6_LINK == 1)
    (void)bms_can_send(can_id, p_data, dlc);
#else
    UNUSED_ARG(can_id); UNUSED_ARG(p_data); UNUSED_ARG(dlc);
#endif
}

/* ==================================================================
 *  페이로드 패킹 (전송 방식과 무관)
 * ================================================================== */
static void pack_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

void bms_link_init(void)
{
#if (BRINGUP_S6_LINK == 0)
    /* S6 가 꺼져 있으면 CAN 을 기동하지 않는다. 기동만 해 두면 RX FIFO(3단)를
     * 아무도 비우지 않아 오버런만 쌓인다. */
    DBG_I("link init skipped (BRINGUP_S6_LINK = 0)");
#else
    if (!bms_can_init()) {
        DBG_E("link init FAILED (CAN not started) - frames will be dropped");
        return;
    }
    DBG_I("link init (CAN 500kbps)");
#endif
}

void bms_link_send_main(const bms_data_t *p_d)
{
    uint8_t d[8];
    int32_t v_x100 = DIV_ROUND(p_d->pack_mv, 10);       /* mV -> 0.01V */
    int32_t i_x100 = DIV_ROUND(p_d->pack_ma, 10);       /* mA -> 0.01A */

    v_x100 = CLAMP(v_x100, 0, 65535);
    i_x100 = CLAMP(i_x100, -32768, 32767);

    pack_u16_le(&d[0], (uint16_t)v_x100);
    pack_u16_le(&d[2], (uint16_t)(int16_t)i_x100);
    d[4] = p_d->soc;
    d[5] = (uint8_t)(p_d->charge_permit ? 1U : 0U);
    d[6] = (uint8_t)p_d->state;
    d[7] = (uint8_t)(p_d->fault & 0xFFU);

    link_send_frame(CFG_CAN_ID_MAIN, d, 8);

    /* 0x103 : 상태/Fault 축약 (EVSE 가 빠르게 파싱하기 위한 프레임) */
    {
        uint8_t s[2];
        s[0] = (uint8_t)p_d->state;
        s[1] = (uint8_t)(p_d->fault & 0xFFU);
        link_send_frame(CFG_CAN_ID_STATE, s, 2);
    }
}

void bms_link_send_cell(const bms_data_t *p_d)
{
    uint8_t d[8];
    uint8_t i;
    int32_t mv;

    /* 0x101 : Cell1~4 전압 (int16 mV, LE).
     * 음수는 실제 셀 전압이 아니라 노드 배선/보정 이상을 찾는 진단값이므로
     * 0으로 숨기지 않고 EVSE 에 그대로 전달한다. */
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        mv = CLAMP(p_d->cell_mv[i], -32768, 32767);
        pack_u16_le(&d[i * 2U], (uint16_t)(int16_t)mv);
    }
    link_send_frame(CFG_CAN_ID_CELL, d, 8);

    /* 0x102 : 온도 + 셀 편차 + Cell Min/Max.
     * min/max 는 BMS 가 이미 계산해 들고 있으니 그대로 실어 보낸다 —
     * 상위 노드가 0x101 의 셀 4개를 다시 훑을 필요가 없다. */
    {
        uint8_t t[8];
        pack_u16_le(&t[0], (uint16_t)(int16_t)CLAMP(p_d->temp_c10, -32768, 32767));
        pack_u16_le(&t[2], (uint16_t)CLAMP(p_d->imbalance_mv, 0, 65535));
        pack_u16_le(&t[4], (uint16_t)(int16_t)CLAMP(p_d->cell_min_mv, -32768, 32767));
        pack_u16_le(&t[6], (uint16_t)(int16_t)CLAMP(p_d->cell_max_mv, -32768, 32767));
        link_send_frame(CFG_CAN_ID_TEMP, t, 8);
    }
}

void bms_link_send_version(void)
{
    uint8_t d[4];

    d[0] = CFG_FW_VERSION_MAJOR;
    d[1] = CFG_FW_VERSION_MINOR;
    d[2] = 0;                       /* OTA 상태 (2단계) */
    d[3] = 0;
    link_send_frame(CFG_CAN_ID_VERSION, d, 4);
}

/**
 * @brief  EVSE -> BMS 수신 처리
 * @note   S6 가 꺼져 있으면 bms_can_poll_rx() 가 CAN 미기동 상태에서 즉시 돌아오므로
 *         호출부에 #if 를 두지 않는다.
 */
void bms_link_poll_rx(void)
{
    bms_can_poll_rx();
}
