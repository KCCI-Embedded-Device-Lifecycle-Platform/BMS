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
 * @brief  송신 에러 카운터(TEC) 현재값
 * @note   log_stats 는 1초 요약이라 "릴레이가 닫히는 그 100ms 에 TEC 가 튀었는가" 를
 *         볼 수 없다. 셀 ADC 기준전위와 CAN 차동버스는 서로 무관한 두 계통이므로,
 *         릴레이 개폐에 둘이 같이 흔들린다면 공통분모는 접지뿐이다 — 그 상관을
 *         같은 줄에서 보려고 카운터만 별도로 연다 (ESR 해석은 여전히 이 파일 안에 둔다).
 */
uint16_t bms_can_get_tec(void);


#endif /* __BMS_CAN_H_ */
