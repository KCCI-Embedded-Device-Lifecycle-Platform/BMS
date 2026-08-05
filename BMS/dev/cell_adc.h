#ifndef __CELL_ADC_H_
#define __CELL_ADC_H_
#include "common_def.h"

void    cell_adc_init(void);
void    cell_adc_update(void);                      /* 100ms 주기 호출 */
int32_t cell_adc_get_node_mv(uint8_t idx);          /* 0~3 : B1,B2,B3,B+ */
int32_t cell_adc_get_cell_mv(uint8_t idx);          /* 0~3 : Cell1~4     */
int32_t cell_adc_get_pack_mv(void);                 /* = node[3]         */
/* 채널별 게인 보정 (Q16 고정소수점, 65536 = x1.0) */
void    cell_adc_set_gain(uint8_t idx, int32_t gain_q16);
void    cell_adc_set_offset(uint8_t idx, int32_t off_mv);
int32_t cell_adc_get_gain(uint8_t idx);
#endif
