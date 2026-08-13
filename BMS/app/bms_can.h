/**
 * @file    bms_can.h
 * @brief   bxCAN(CAN1) 송수신 계층
 * @note    bms_link.c 가 "무엇을 보낼지"(바이트 맵), 이 파일이 "어떻게 내보낼지"(프레임).
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
 * @retval false : 송신 메일박스 3개가 모두 참 (버스 미연결/ACK 없음)
 */
bool bms_can_send(uint16_t std_id, const uint8_t *p_data, uint8_t dlc);

/**
 * @brief  RX FIFO0 를 비우면서 EVSE 명령을 처리한다.
 * @note   FIFO 가 3단뿐이라 100ms 슬롯에 묶으면 넘친다 -> 메인 루프에서 매 바퀴 호출.
 */
void bms_can_poll_rx(void);

/* --- EVSE -> BMS 최신 상태 (수신 프레임에서 갱신) ---
 * app 계층은 이 getter 를 직접 부르지 않는다. ap_collect() 가 블랙보드로 옮겨 주고
 * FSM/UI 는 블랙보드만 읽는다. "상태는 s_bms 한 곳" 규칙이니 우회하지 말 것. */
bool    bms_can_evse_charge_req(void);      /* 0x201 : 충전 요청       */
bool    bms_can_evse_relay_on(void);        /* 0x200 : EVSE 릴레이 on  */
bool    bms_can_evse_connected(void);       /* 0x200 : 커넥터 체결     */
bool    bms_can_evse_estop(void);           /* 0x200 : 비상정지 눌림   */
uint8_t bms_can_evse_fault(void);           /* 0x202 : EVSE 측 Fault   */

/**
 * @brief  EVSE 수신 상태 전체 초기화 (LINK_TIMEOUT 확정 시)
 * @note   안 지우면 링크 복구 순간 stale charge_req 로 충전이 재개된다.
 *         호출 지점은 ap_collect() 한 곳뿐이다.
 */
void    bms_can_clear_evse_state(void);

/* ==================================================================
 *  벤치 검증용 (시리얼 콘솔에서 조작)
 * ================================================================== */

/** @brief HAL_CAN_Start 까지 성공했는가. 실패면 송수신 시도 자체가 무의미하다. */
bool    bms_can_is_ready(void);

/**
 * @brief  프레임 단위 TX/RX 트레이스 on/off (콘솔 't')
 * @note   100ms 마다 0x100/0x103 이 나가서 상시 켜면 초당 20줄 이상이 쏟아진다.
 *         평소엔 log_stats 의 1초 요약만 보고, 바이트가 필요할 때만 켠다.
 */
void    bms_can_set_trace(bool on);
bool    bms_can_get_trace(void);

/**
 * @brief  1초 요약 : 송수신 카운터 + TEC / REC / LEC / Bus-Off
 * @note   NORMAL 브링업에서 제일 먼저 볼 줄이다. "송신은 됐는데 상대가 못 받는다" 와
 *         "애초에 버스로 못 나갔다" 는 둘 다 '아무 일도 안 일어남' 으로 보이는데,
 *         TEC 와 LEC 로만 갈린다 (LEC=ACK 면 아무도 응답하지 않은 것).
 *         ESR 해석을 app 으로 끌어올리지 않으려고 getter 가 아니라 로그로 노출한다.
 */
void    bms_can_log_stats(void);

/**
 * @brief  EVSE 상태(0x200) + 충전 요청(0x201) 1회 송신 - EVSE 역할 흉내
 * @note   같은 펌웨어를 올린 Nucleo 한 장을 'e' 로 EVSE 역할로 돌리면 실버스 2노드가
 *         성립한다. s_link_sim('p')과 달리 필터->FIFO->디스패치 경로를 전부 실제로 탄다.
 * @warning 이 프레임을 내보내는 보드는 BMS 프레임 송신을 멈춰야 한다. 같은 ID 를 두
 *         노드가 실으면 데이터 구간에서 비트 에러가 터진다 (호출부가 배타적으로 다룬다).
 */
void    bms_can_sim_evse(bool charge_req, bool estop);

/**
 * @brief  파라미터 쓰기(0x205) 1회 송신 - 과온 임계 변경 요청
 * @note   상대 BMS 가 클램프한 실제 적용값을 0x105 로 되돌려 준다.
 */
void    bms_can_sim_param(int16_t ot_c10);

#endif /* __BMS_CAN_H_ */
