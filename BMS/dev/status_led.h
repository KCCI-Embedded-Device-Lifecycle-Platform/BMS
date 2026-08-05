#ifndef __STATUS_LED_H_
#define __STATUS_LED_H_
#include "common_def.h"

void status_led_init(void);
void status_led_set_state(bms_state_t st);
void status_led_update(void);       /* 100ms 주기 */
#endif
