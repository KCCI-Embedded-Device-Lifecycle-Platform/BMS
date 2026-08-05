#ifndef __HW_TICK_H_
#define __HW_TICK_H_
#include "common_def.h"

uint32_t hw_tick_ms(void);
void     hw_tick_delay(uint32_t ms);
/**
 * @brief  주기 도래 여부 확인 (논블로킹 협조형 스케줄러)
 * @param  p_last  : 마지막 실행 시각을 보관할 변수 주소
 * @param  period  : 주기 [ms]
 * @retval true 이면 지금 실행할 차례
 * @note   HAL_GetTick() 오버플로(약 49.7일)에도 안전하도록
 *         뺄셈 결과를 부호없는 정수로 비교한다.
 */
bool     hw_tick_due(uint32_t *p_last, uint32_t period);
bool     hw_tick_elapsed(uint32_t start, uint32_t period);
#endif
