#ifndef __CELL_ADC_H_
#define __CELL_ADC_H_
#include "common_def.h"

void    cell_adc_init(void);
void    cell_adc_update(void);                      /* 100ms 주기 호출 */
int32_t cell_adc_get_node_mv(uint8_t idx);          /* 0~3 : B1,B2,B3,B+ */
int32_t cell_adc_get_cell_mv(uint8_t idx);          /* 0~3 : Cell1~4     */
int32_t cell_adc_get_pack_mv(void);                 /* = node[3]         */

/**
 * @brief  분압 복원 전의 ADC 핀 전압 [uV]
 * @note   노드 값이 이상할 때 "ADC 가 이상한가" 와 "분압비 설정이 실물과 다른가" 를
 *      가르는 유일한 지점이다. 복원된 mV 만 봐서는 둘이 구분되지 않는다.
 */
int32_t cell_adc_get_pin_uv(uint8_t idx);

/**
 * @brief  그 노드의 ADC 가 풀스케일에 붙어 있는가 (= 핀 전압이 VDDA 이상)
 *
 * @note   포화는 값이 "조금 틀린" 것이 아니라 그 채널에 정보가 없다는 뜻이다.
 *      그런데 복원된 mV 만 보면 그냥 큰 값으로 보여서 과전압 셀과 구분되지 않는다
 *      (node = VDDA x 배율 이라 x6 이면 딱 19.8V 근처로 그럴듯하게 나온다).
 *      분압기 하단 저항이 뜨거나 분압비가 너무 작으면 바로 이 상태가 된다.
 */
bool     cell_adc_is_railed(uint8_t idx);

/** @brief 그 노드의 ADC 원값 (0~4095) */
uint16_t cell_adc_get_raw(uint8_t idx);

/** @brief 그 노드에 적용 중인 분압 복원 배율 (x1000 스케일, 6.000 -> 6000) */
int32_t cell_adc_get_div_x1000(uint8_t idx);

/* 채널별 게인 보정 (Q16 고정소수점, 65536 = x1.0) */
void    cell_adc_set_gain(uint8_t idx, int32_t gain_q16);
int32_t cell_adc_get_gain(uint8_t idx);

/**
 * @brief  1점 게인 캘리브레이션 : "이 노드의 진짜 전압은 actual_mv 다" 를 알려준다
 *
 * @param  idx       0~3 (B1, B2, B3, B+)
 * @param  actual_mv DMM 으로 잰 해당 노드의 실제 전압 [mV]
 * @retval false     측정값이 너무 작거나 보정폭이 ±25% 밖 -> 반영하지 않음.
 *                   이때는 게인이 아니라 분압비가 다른 것이므로
 *                   cell_adc_measure_div_x1000() 으로 실제 배율을 확인할 것.
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

/**
 * @brief  DMM 실측값으로부터 그 노드의 "실제 분압비" 를 역산한다 (x1000 스케일)
 * @retval 0  핀 전압이 0 이거나 입력이 너무 작다 = 분압비 문제가 아니라 결선 문제
 *
 * @note   보정이 아니라 측정이다. 아무것도 바꾸지 않고 값만 돌려준다 —
 *      결과는 사람이 확인한 뒤 CFG_DIV_SCALE_X1000_LIST 에 옮겨 적는다.
 *      가드레일에 걸린 캘리브레이션이 막다른 길로 끝나지 않게 하는 것이 목적이고,
 *      런타임에 자동 반영하지 않는 것도 의도다 (분압비는 하드웨어 사실이라
 *      전원을 껐다 켜면 사라지는 곳에 두면 안 된다).
 */
int32_t cell_adc_measure_div_x1000(uint8_t idx, int32_t actual_mv);

/** @brief 전 채널 보정값을 x1.0 / 0mV 로 되돌린다 */
void    cell_adc_reset_cal(void);
#endif
