#ifndef __NTC_H_
#define __NTC_H_
#include "common_def.h"

#define NTC_TEMP_INVALID    BMS_TEMP_INVALID

void    ntc_init(void);
void    ntc_update(void);                   /* 500ms 주기 */
int32_t ntc_get_temp_c10(uint8_t idx);      /* 0.1C 단위, 오류 시 NTC_TEMP_INVALID */
int32_t ntc_get_res_ohm(uint8_t idx);

/* 유효한 채널들의 최댓값. 과온 보호는 반드시 이 값을 봐야 한다 —
 * 채널 하나만 보면 나머지 센서가 과온이어도 못 잡는다.
 * 전 채널이 단선/단락이면 NTC_TEMP_INVALID (호출부는 1채널 때와 동일하게 동작). */
int32_t ntc_get_temp_max_c10(void);
uint8_t ntc_get_valid_count(void);        /* 살아있는 채널 수 (브링업 진단용) */
#endif
