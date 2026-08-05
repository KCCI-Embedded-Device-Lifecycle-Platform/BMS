#ifndef __BMS_STATE_H_
#define __BMS_STATE_H_
#include "common_def.h"

void        bms_fsm_init(bms_data_t *p_d);
void        bms_fsm_run(bms_data_t *p_d);       /* 100ms 주기 */
const char *bms_fsm_state_name(bms_state_t st);
#endif
