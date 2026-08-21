#ifndef __HW_GPIO_H_
#define __HW_GPIO_H_
#include "common_def.h"

/**
 * @note  hw_gpio_set(ch, true) 는 "논리적 활성" 이고, 물리 레벨(액티브 하이/로우)은
 *        hw_gpio.c 의 매핑 테이블이 흡수한다.
 *        hw_gpio_init() 이 전부 비활성으로 놓으므로 부팅 순간 릴레이는 반드시 열려 있다.
 *        이것이 fail-safe 의 출발점이라 호출 순서를 바꾸지 말 것
 *        (status_led_init() 이 hw_gpio_init() 을 가장 먼저 부른다).
 */
typedef enum {
    HW_OUT_LED_RUN = 0,     /* PA5  : 초록 Power LED (온보드 LD2 와 공용) */
    HW_OUT_LED_CHARGE,      /* PC6  : 노랑 Charge LED          */
    HW_OUT_LED_FAULT,       /* PC8  : 빨강 Fault LED           */
    HW_OUT_RELAY_CHG,       /* PB5  : 충전 차단 릴레이 (NO 접점) */
    HW_OUT_MAX
} hw_out_t;

void hw_gpio_init(void);
void hw_gpio_set(hw_out_t ch, bool on);
#endif
