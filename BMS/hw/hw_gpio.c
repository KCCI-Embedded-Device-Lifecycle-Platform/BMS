/**
 * @file    hw_gpio.c
 * @brief   출력 GPIO 논리적 이름 <-> 물리 핀 매핑
 * @note    CubeMX 의 User Label 에 의존하지 않도록 여기서 직접 정의한다.
 *          핀이 바뀌면 이 파일 한 곳만 수정하면 된다.
 *          (핀 자체의 MODER/OTYPER 설정은 CubeMX 가 gpio.c 에서 이미 수행)
 */
#include "hw_gpio.h"

/* 릴레이 모듈은 제품마다 액티브 레벨이 갈린다 (저가 모듈 상당수가 액티브 Low).
 * bms_cfg.h 를 HAL 심볼로부터 독립시키기 위해 1/0 스위치로 받고 여기서 변환한다. */
#if (CFG_RELAY_ACTIVE_HIGH == 1)
  #define RELAY_ACTIVE_LEVEL    GPIO_PIN_SET
#else
  #define RELAY_ACTIVE_LEVEL    GPIO_PIN_RESET
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState active;       /* 액티브 레벨 */
} hw_pin_map_t;

/* LED 3색은 전부 애노드를 핀에, 캐소드를 저항 거쳐 GND 로 (액티브 High) 전제한다.
 * 공통 애노드 모듈(액티브 Low)을 쓰면 여기만 GPIO_PIN_RESET 으로 바꾸면 된다 —
 * status_led.c 는 "논리적 점등" 만 다루므로 한 줄도 안 바뀐다. */
static const hw_pin_map_t s_map[HW_OUT_MAX] = {
    /* HW_OUT_LED_RUN    */ { GPIOA, GPIO_PIN_5, GPIO_PIN_SET       },
    /* HW_OUT_LED_CHARGE */ { GPIOC, GPIO_PIN_6, GPIO_PIN_SET       },
    /* HW_OUT_LED_FAULT  */ { GPIOC, GPIO_PIN_8, GPIO_PIN_SET       },
    /* HW_OUT_RELAY_CHG  */ { GPIOB, GPIO_PIN_5, RELAY_ACTIVE_LEVEL },
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

