/**
 * @file    bms_can.h
 * @brief   bxCAN(CAN1) 송수신 계층
 * @note    bms_link.c 가 "무엇을 보낼지"(바이트 맵)를 정하고,
 *          이 파일이 "어떻게 내보낼지"(CAN 프레임)를 담당한다.
 */
#ifndef __BMS_CAN_H_
#define __BMS_CAN_H_
#include "common_def.h"

/**
 * @brief  필터 설정 + CAN 기동
 * @retval false 면 CAN 이 뜨지 않은 것이므로 송수신을 시도하면 안 된다.
 */
bool bms_can_init(void);

/**
 * @brief  표준 ID 데이터 프레임 1개 송신 (논블로킹)
 * @retval false : 송신 메일박스 3개가 모두 차 있음 (버스 미연결/ACK 없음)
 */
bool bms_can_send(uint16_t std_id, const uint8_t *p_data, uint8_t dlc);

/**
 * @brief  RX FIFO0 를 비우면서 EVSE 명령을 처리한다.
 * @note   메인 루프에서 매 바퀴 호출한다. FIFO 는 3단뿐이라
 *         100ms 태스크에 넣으면 EVSE 가 빠르게 보낼 때 넘칠 수 있다.
 */
void bms_can_poll_rx(void);

/* --- EVSE -> BMS 최신 상태 (수신 프레임에서 갱신) --- */
bool    bms_can_evse_charge_req(void);      /* 0x201 : 충전 요청       */
bool    bms_can_evse_relay_on(void);        /* 0x200 : EVSE 릴레이 on  */
bool    bms_can_evse_connected(void);       /* 0x200 : 커넥터 체결     */
uint8_t bms_can_evse_fault(void);           /* 0x202 : EVSE 측 Fault   */

#endif /* __BMS_CAN_H_ */
