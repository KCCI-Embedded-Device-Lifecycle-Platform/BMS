#ifndef __NTC_H_
#define __NTC_H_
#include "common_def.h"

#define NTC_TEMP_INVALID    BMS_TEMP_INVALID

void    ntc_init(void);
void    ntc_update(void);                   /* 500ms 주기 */
int32_t ntc_get_temp_c10(uint8_t idx);      /* 0.1C 단위, 오류 시 NTC_TEMP_INVALID */
int32_t ntc_get_res_ohm(uint8_t idx);
#endif
