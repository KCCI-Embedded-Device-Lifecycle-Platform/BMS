/**
 * @file    status_led.c
 * @brief   상태별 LED 블링크 패턴 + 부저
 * @note    보드에 UART 를 못 물릴 상황(시연장)에서도
 *          LED 패턴만 보고 BMS 상태를 알 수 있게 한다.
 *          패턴은 16bit 비트맵 = 100ms x 16 = 1.6초 주기.
 *          1 이면 점등. 테이블만 바꾸면 패턴을 바꿀 수 있다.
 */
#include "status_led.h"
#include "hw_gpio.h"

static const uint16_t s_pattern[BMS_ST_MAX] = {
    /* INIT         */ 0xFFFFU,     /* 상시 점등     */
    /* SELF_CHECK   */ 0xAAAAU,     /* 빠른 점멸     */
    /* IDLE         */ 0x0001U,     /* 1.6초에 한 번 */
    /* CHARGE_READY */ 0x0101U,     /* 두 번 깜빡    */
    /* CHARGING     */ 0x00FFU,     /* 느린 호흡     */
    /* FAULT        */ 0x5555U      /* 최고속 점멸   */
};

static bms_state_t s_state;
static uint8_t     s_phase;

void status_led_init(void)
{
    hw_gpio_init();
    s_state = BMS_ST_INIT;
    s_phase = 0;
}

void status_led_set_state(bms_state_t st)
{
    if (st < BMS_ST_MAX) {
        s_state = st;
    }
}

void status_led_update(void)
{
    bool on;

    on = ((s_pattern[s_state] >> s_phase) & 0x1U) != 0U;

    if (s_state == BMS_ST_FAULT) {
        /* Fault LED 는 PC8 = 외부 배선이 필요한 핀이다.
         * 외부 LED 를 안 달았으면 FAULT 가 눈에 전혀 안 보이므로,
         * CFG_FAULT_LED_MIRROR_LD2 로 온보드 LD2(PA5)에도 같이 낸다. */
#if (CFG_FAULT_LED_MIRROR_LD2 == 1)
        hw_gpio_set(HW_OUT_LED_RUN,   on);
#else
        hw_gpio_set(HW_OUT_LED_RUN,   false);
#endif
        hw_gpio_set(HW_OUT_LED_FAULT, on);
        /* 부저는 8비트마다 짧게만 울려 소음을 줄인다 */
        hw_gpio_set(HW_OUT_BUZZER, (bool)(on && (s_phase < 2U)));
    } else {
        hw_gpio_set(HW_OUT_LED_RUN,   on);
        hw_gpio_set(HW_OUT_LED_FAULT, false);
        hw_gpio_set(HW_OUT_BUZZER,    false);
    }

    s_phase = (uint8_t)((s_phase + 1U) & 0x0FU);
}
