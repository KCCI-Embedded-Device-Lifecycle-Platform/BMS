#ifndef __BMS_SOC_H_
#define __BMS_SOC_H_
#include "common_def.h"

void    bms_soc_init(void);
void    bms_soc_update(bms_data_t *p_d);    /* 1s 주기 */
#endif
