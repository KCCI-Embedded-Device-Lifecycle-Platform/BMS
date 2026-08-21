#ifndef __STATUS_LED_H_
#define __STATUS_LED_H_
#include "common_def.h"

void status_led_init(void);
void status_led_set_state(bms_state_t st);
void status_led_update(void);       /* 100ms 주기 */

/**
 * @brief  노랑(Charge) LED 점등 여부 — 릴레이 지령을 그대로 넘긴다.
 * @note   bms_data_t 를 통째로 받지 않는 이유: dev 계층은 app 타입을 몰라야 한다.
 *         ap_relay_update() 직후에 부를 것 (같은 100ms 슬롯의 최신 값이어야 한다).
 */
void status_led_set_charge(bool on);

/**
 * @brief  표시 출력(LED 3색) 전체 강제 OFF 토글 — 접지 진단용
 * @note   LED 구동 전류가 공용 접지 배선을 타면 셀 ADC 기준전위가 밀린다.
 *         cell1 은 차분이 아니라 node1 그 자체라 그 이동이 셀1 에만 통째로 실린다.
 *         3색 LED 는 상시 점등 구간이 많아(초록 0xFFFE, 빨강 FAULT 상시) 예전처럼
 *         100ms 교대를 찾는 대신, 뮤트 ON/OFF 로 셀1 이 통째로 이동하는지를 본다 —
 *         리셋 없이 같은 부팅에서 비교된다.
 *         측정에만 쓰는 진단 스위치다 — 판단/차단 경로는 건드리지 않는다.
 */
void status_led_set_mute(bool mute);
bool status_led_get_mute(void);
#endif
