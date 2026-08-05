/**
 * @file    hw_gpio.c
 * @brief   출력 GPIO 논리적 이름 <-> 물리 핀 매핑
 * @note    CubeMX 의 User Label 에 의존하지 않도록 여기서 직접 정의한다.
 *          핀이 바뀌면 이 파일 한 곳만 수정하면 된다.
 *          (핀 자체의 MODER/OTYPER 설정은 CubeMX 가 gpio.c 에서 이미 수행)
 */
#include "hw_gpio.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState active;       /* 액티브 레벨 */
} hw_pin_map_t;

static const hw_pin_map_t s_map[HW_OUT_MAX] = {
    /* HW_OUT_LED_RUN   */ { GPIOA, GPIO_PIN_5,  GPIO_PIN_SET   },
    /* HW_OUT_LED_FAULT */ { GPIOC, GPIO_PIN_8,  GPIO_PIN_SET   },
    /* HW_OUT_BUZZER    */ { GPIOC, GPIO_PIN_9,  GPIO_PIN_SET   },
    /* HW_OUT_OLED_CS   */ { GPIOB, GPIO_PIN_12, GPIO_PIN_RESET },
    /* HW_OUT_OLED_DC   */ { GPIOC, GPIO_PIN_7,  GPIO_PIN_SET   },
    /* HW_OUT_OLED_RES  */ { GPIOC, GPIO_PIN_6,  GPIO_PIN_RESET },
};

void hw_gpio_init(void)
{
    uint8_t i;
    for (i = 0; i < (uint8_t)HW_OUT_MAX; i++) {
        hw_gpio_set((hw_out_t)i, false);
    }
}

void hw_gpio_set(hw_out_t ch, bool on)
{
    GPIO_PinState lv;

    if (ch >= HW_OUT_MAX) {
        return;
    }
    lv = on ? s_map[ch].active
            : ((s_map[ch].active == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(s_map[ch].port, s_map[ch].pin, lv);
}

void hw_gpio_toggle(hw_out_t ch)
{
    if (ch >= HW_OUT_MAX) {
        return;
    }
    HAL_GPIO_TogglePin(s_map[ch].port, s_map[ch].pin);
}
