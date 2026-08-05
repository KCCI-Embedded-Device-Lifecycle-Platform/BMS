/**
 * @file    bms_link.c
 * @brief   EVSE 통신 계층 (1단계: UART / 2단계: bxCAN)
 *
 * @details 왜 이렇게 나눴는가?
 *      1단계 보드였던 STM32F411RE 에는 bxCAN 페리페럴이 없었다.
 *      그렇다고 CAN 을 나중에 통째로 새로 짜면 1단계 작업이 버려진다.
 *
 *      그래서 이 파일을 두 부분으로 명확히 나눴다.
 *        (1) pack_xxx()   : CAN 프레임과 100% 동일한 8바이트 페이로드 생성
 *        (2) link_send()  : 그 8바이트를 실제로 내보내는 전송 함수
 *
 *      바이트 맵, 엔디안, 스케일링 로직은 전부 그대로 재사용된다.
 *
 * @note  2단계 보드(F446RE)로 옮기면서 bxCAN(CAN1, PB8 RX / PB9 TX)이
 *        붙었다. 실제 프레임 송수신은 bms_can.c 가 담당하고,
 *        이 파일은 bms_cfg.h 의 CFG_LINK_TRANSPORT 로 백엔드만 고른다.
 *        설계 의도대로 (1) 패킹 로직은 한 줄도 바뀌지 않았다.
 *
 * @note  바이트 맵 (CAN ID 0x100, DLC 8) - EVSE 파트와 합의한 규격
 *      Data[0..1]  Pack Voltage   0.01V 단위, Little Endian
 *      Data[2..3]  Pack Current   0.01A 단위, Little Endian, int16 (부호 있음)
 *      Data[4]     SOC            1% 단위
 *      Data[5]     Charge Permit  0 = 금지 / 1 = 허가
 *      Data[6]     BMS State      bms_state_t
 *      Data[7]     Fault Code     비트마스크 하위 8bit
 */
#include "bms_link.h"
#include "bms_fault.h"
#include "bms_can.h"
#include "hw_uart.h"
#include "dbg.h"

/* ==================================================================
 *  전송 백엔드 : bms_cfg.h 의 CFG_LINK_TRANSPORT 하나로 갈린다
 * ================================================================== */
static void link_send_frame(uint16_t can_id, const uint8_t *p_data, uint8_t dlc)
{
#if (BRINGUP_S6_LINK == 1)

  #if (CFG_LINK_TRANSPORT == CFG_LINK_TRANSPORT_CAN)
    /* [2단계] 실제 CAN 프레임. 페이로드는 아래 UART 경로와 100% 동일하다. */
    (void)bms_can_send(can_id, p_data, dlc);

  #else
    /* [1단계] UART ASCII 프레임 : 로직 분석기 없이 눈으로 검증 가능 */
    char     line[64];
    int      n;
    uint8_t  i;

    n = snprintf(line, sizeof(line), "$%03X,%u,", can_id, dlc);
    for (i = 0; i < dlc; i++) {
        n += snprintf(&line[n], sizeof(line) - (size_t)n, "%02X", p_data[i]);
    }
    n += snprintf(&line[n], sizeof(line) - (size_t)n, "\r\n");
    hw_uart_write((const uint8_t *)line, (uint16_t)n);
  #endif

#else
    UNUSED_ARG(can_id); UNUSED_ARG(p_data); UNUSED_ARG(dlc);
#endif
}

/* ==================================================================
 *  페이로드 패킹 (CAN 규격 그대로 - 전송 방식과 무관)
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

#elif (CFG_LINK_TRANSPORT == CFG_LINK_TRANSPORT_CAN)
    if (!bms_can_init()) {
        DBG_E("link init FAILED (CAN not started) - frames will be dropped");
        return;
    }
    DBG_I("link init (transport = CAN 500kbps)");

#else
    DBG_I("link init (transport = UART, CAN-compatible payload)");
#endif
}

void bms_link_send_main(const bms_data_t *p_d)
{
    uint8_t d[8];
    int32_t v_x100 = DIV_ROUND(p_d->pack_mv, 10);       /* mV  -> 0.01V */
    int32_t i_x100 = DIV_ROUND(p_d->pack_ma, 10);       /* mA  -> 0.01A */

    v_x100 = CLAMP(v_x100, 0, 65535);
    i_x100 = CLAMP(i_x100, -32768, 32767);

    pack_u16_le(&d[0], (uint16_t)v_x100);
    pack_u16_le(&d[2], (uint16_t)(int16_t)i_x100);
    d[4] = p_d->soc;
    d[5] = (uint8_t)(p_d->charge_permit ? 1U : 0U);
    d[6] = (uint8_t)p_d->state;
    d[7] = (uint8_t)(p_d->fault & 0xFFU);

    link_send_frame(CFG_CAN_ID_MAIN, d, 8);

    /* 0x103 : 상태/Fault 전용 (EVSE 가 빠르게 파싱하기 위한 축약 프레임) */
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

    /* 0x101 : Cell1~4 전압 (mV, LE) - 8바이트에 딱 맞는다 */
    for (i = 0; i < BMS_CELL_COUNT; i++) {
        mv = CLAMP(p_d->cell_mv[i], 0, 65535);
        pack_u16_le(&d[i * 2U], (uint16_t)mv);
    }
    link_send_frame(CFG_CAN_ID_CELL, d, 8);

    /* 0x102 : 온도(0.1C, int16) + 셀 편차(mV) + Cell Min + Cell Max
     * EVSE / Gateway 가 cell_min / cell_max 를 직접 요구하므로,
     * BMS 가 이미 계산해 들고 있는 값을 그대로 실어 보낸다.
     * (상위 노드가 0x101 의 셀 4개를 다시 훑어 최대/최소를 뽑을 필요가 없다) */
    {
        uint8_t t[8];
        pack_u16_le(&t[0], (uint16_t)(int16_t)CLAMP(p_d->temp_c10, -32768, 32767));
        pack_u16_le(&t[2], (uint16_t)CLAMP(p_d->imbalance_mv, 0, 65535));
        pack_u16_le(&t[4], (uint16_t)CLAMP(p_d->cell_min_mv,  0, 65535));
        pack_u16_le(&t[6], (uint16_t)CLAMP(p_d->cell_max_mv,  0, 65535));
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
 * @note   CAN 백엔드에서는 RX FIFO 를 비우며 명령을 디스패치한다.
 *         UART 백엔드에는 수신 경로가 없다. 하트비트 대용 'p' 명령은
 *         bms_app.c 의 콘솔 핸들러가 처리하므로, 여기서 hw_uart_get_byte()
 *         를 부르면 콘솔이 먹을 바이트를 가로채게 된다. 그래서 비워 둔다.
 */
void bms_link_poll_rx(void)
{
#if (CFG_LINK_TRANSPORT == CFG_LINK_TRANSPORT_CAN)
    bms_can_poll_rx();
#endif
}
