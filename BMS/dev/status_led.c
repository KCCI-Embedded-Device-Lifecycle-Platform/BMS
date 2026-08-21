/**
 * @file    status_led.c
 * @brief   3색 상태 LED (초록 Power / 노랑 Charge / 빨강 Fault)
 *
 * @note    UART 를 못 물리는 상황(시연장)에서도 LED 만으로 상태를 알 수 있게 한다.
 *          한 색이 여러 의미를 겸하던 예전 1색 구조와 달리, 이제 색마다 질문이 하나씩이다:
 *              초록 = 보드가 살아있는가        (전원 + super-loop 생존)
 *              노랑 = 충전 경로가 닫혔는가      (릴레이 지령)
 *              빨강 = 보호 조건을 위반했는가    (FAULT)
 *
 * @note    **노랑과 빨강은 동시에 켜지지 않는다.** 이것이 페일세이프의 시각적 증거다.
 *          단, 그 배타성을 LED 단에서 `if (!fault)` 로 만들지 않는다 — 아래 참조.
 *
 *          초록 패턴만 16bit 비트맵(100ms x 16 = 1.6초 주기)이고, 1 이면 점등.
 */
#include "status_led.h"
#include "hw_gpio.h"

/* 초록(Power) LED 패턴.
 * 부팅 구간(INIT/SELF_CHECK)만 눈에 띄게 깜빡이고, 운전에 들어가면
 * "점등 + 1.6초마다 100ms 노치" 로 바뀐다. 완전 상시 ON 으로 두지 않는 이유는
 * 펌웨어가 죽어도 포트 래치가 그대로 남아 똑같이 켜져 보이기 때문이다 —
 * 노치가 멈추면 "전원은 있는데 루프가 섰다" 가 리셋 없이 구분된다.
 * FAULT 에서도 초록을 끄지 않는다. 끄면 "전원이 나갔다" 로 오독된다. */
static const uint16_t s_power_pattern[BMS_ST_MAX] = {
    /* INIT         */ 0xF0F0U,     /* 0.8초 주기 느린 깜빡 (부팅 중)   */
    /* SELF_CHECK   */ 0xAAAAU,     /* 100ms 빠른 점멸 (자가진단 중)    */
    /* IDLE         */ 0xFFFEU,     /* 점등 + 하트비트 노치             */
    /* CHARGE_READY */ 0xFFFEU,
    /* CHARGING     */ 0xFFFEU,
    /* FAULT        */ 0xFFFEU
};

static bms_state_t s_state;
static uint8_t     s_phase;
static bool        s_charge;    /* 릴레이 지령 (노랑 LED 소스) */
static bool        s_mute;      /* 접지 진단용 : 표시 출력 전체 차단 */

void status_led_init(void)
{
    hw_gpio_init();
    s_state  = BMS_ST_INIT;
    s_phase  = 0;
    s_charge = false;
}

void status_led_set_charge(bool on)
{
    s_charge = on;
}

void status_led_set_state(bms_state_t st)
{
    if (st < BMS_ST_MAX) {
        s_state = st;
    }
}

void status_led_update(void)
{
    bool power_on = ((s_power_pattern[s_state] >> s_phase) & 0x1U) != 0U;
    bool fault    = (s_state == BMS_ST_FAULT);

    /* 뮤트 중에도 phase 는 계속 돌린다. 해제한 순간 패턴이 이어져야
     * "뮤트 때문에 달라진 것" 과 "패턴 위상이 달라진 것" 이 섞이지 않는다. */
    if (s_mute) {
        hw_gpio_set(HW_OUT_LED_RUN,    false);
        hw_gpio_set(HW_OUT_LED_CHARGE, false);
        hw_gpio_set(HW_OUT_LED_FAULT,  false);
        s_phase = (uint8_t)((s_phase + 1U) & 0x0FU);
        return;
    }

    /* --- 초록 : 전원 + 생존 --- */
    hw_gpio_set(HW_OUT_LED_RUN, power_on);

    /* --- 노랑 : 릴레이 지령을 그대로 --- */
    /* `s_charge && !fault` 로 쓰지 않는 이유:
     * FAULT 는 이미 permit=0 을 강제하고 ap_relay_update() 가 접점을 여므로
     * s_charge 는 구조적으로 false 가 된다. 여기서 한 번 더 가리면,
     * 그 차단이 실제로 안 일어났을 때 유일한 증거가 눈앞에서 사라진다.
     * → 노랑과 빨강이 동시에 켜지면 그것은 표시 버그가 아니라 **차단 실패**다. */
    hw_gpio_set(HW_OUT_LED_CHARGE, s_charge);

    /* --- 빨강 : FAULT 상시 점등 --- */
    /* 점멸이 아니라 점등인 이유는 노랑과의 배타성이 한눈에 읽혀야 해서다
     * (점멸이면 꺼져 있는 순간과 정상 상태가 헷갈린다).
     *
     * !! 부저를 떼면서 FAULT 의 알림 수단은 이 LED 하나가 됐다. 소리가 주의를
     *    끌어 주지 않으므로, 보드를 안 보고 있는 동안의 FAULT 는 콘솔 1초 요약이나
     *    CAN 0x103 으로만 잡힌다. */
    hw_gpio_set(HW_OUT_LED_FAULT, fault);

    s_phase = (uint8_t)((s_phase + 1U) & 0x0FU);
}

void status_led_set_mute(bool mute) { s_mute = mute;  }
bool status_led_get_mute(void)      { return s_mute;  }
