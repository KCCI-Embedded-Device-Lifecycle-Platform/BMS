#ifndef __CELL_ADC_H_
#define __CELL_ADC_H_
#include "common_def.h"

void    cell_adc_init(void);
void    cell_adc_update(void);                      /* 100ms 주기 호출 */
int32_t cell_adc_get_node_mv(uint8_t idx);          /* 0~3 : B1,B2,B3,B+ */
int32_t cell_adc_get_cell_mv(uint8_t idx);          /* 0~3 : Cell1~4     */
int32_t cell_adc_get_pack_mv(void);                 /* = node[3]         */
/* 채널별 게인 보정 (Q16 고정소수점, 65536 = x1.0) */
void    cell_adc_set_gain(uint8_t idx, int32_t gain_q16);
void    cell_adc_set_offset(uint8_t idx, int32_t off_mv);
int32_t cell_adc_get_gain(uint8_t idx);

/**
 * @brief  1점 게인 캘리브레이션 : "이 노드의 진짜 전압은 actual_mv 다" 를 알려준다
 *
 * @param  idx       0~3 (B1, B2, B3, B+)
 * @param  actual_mv DMM 으로 잰 해당 노드의 실제 전압 [mV]
 * @retval false     측정값이 너무 작거나 보정폭이 비정상 범위 -> 반영하지 않음
 *
 * @note   게인만 잡고 오프셋은 안 잡는다. 지배 오차가 분압 저항 공차(곱셈성)라
 *      16.8V 에서 168mV 인데, ADC 오프셋 기여분은 x6 을 먹어도 20mV 수준이다.
 *      오프셋까지 잡으려면 0V 근처 두 번째 점이 필요한데 스택 분압에서 그건
 *      노드 단락이라 벤치에서 위험하다. 지배 오차만 정확히 잡는 쪽이 낫다.
 *      현재 게인에 비율을 곱하는 방식이라 여러 번 눌러도 누적이 아니라 수렴한다.
 *
 * @warning 결과는 RAM 에만 남는다 (리셋하면 x1.0). 굳히려면 콘솔 'd' 덤프를
 *      cell_adc_init() 에 박아 넣을 것.
 */
bool    cell_adc_calibrate_node(uint8_t idx, int32_t actual_mv);

/** @brief 전 채널 보정값을 x1.0 / 0mV 로 되돌린다 */
void    cell_adc_reset_cal(void);
#endif
