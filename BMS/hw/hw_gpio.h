#ifndef __HW_GPIO_H_
#define __HW_GPIO_H_
#include "common_def.h"

/**
 * @note  hw_gpio_set(ch, true) 는 "논리적으로 활성" 을 뜻한다.
 *        물리 레벨(액티브 하이/로우)은 hw_gpio.c 의 매핑 테이블이 흡수한다.
 *        hw_gpio_init() 이 전부 false(=비활성) 로 놓으므로
 *        OLED_CS/OLED_RES 는 High(비선택/리셋해제), OLED_DC 는 Low(커맨드)로 시작한다.
 */
typedef enum {
    HW_OUT_LED_RUN = 0,     /* PA5  : 온보드 LD2              */
    HW_OUT_LED_FAULT,       /* PC8  : Fault LED               */
    HW_OUT_BUZZER,          /* PC9  : 부저                    */
    HW_OUT_OLED_CS,         /* PB12 : OLED CS   (액티브 Low)  */
    HW_OUT_OLED_DC,         /* PC7  : OLED D/C  (High = 데이터) */
    HW_OUT_OLED_RES,        /* PC6  : OLED RES  (액티브 Low)  */
    HW_OUT_MAX
} hw_out_t;

void hw_gpio_init(void);
void hw_gpio_set(hw_out_t ch, bool on);
void hw_gpio_toggle(hw_out_t ch);
#endif
