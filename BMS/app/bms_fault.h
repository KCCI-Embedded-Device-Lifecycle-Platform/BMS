#ifndef __BMS_FAULT_H_
#define __BMS_FAULT_H_
#include "common_def.h"

void        bms_fault_init(void);
void        bms_fault_check(bms_data_t *p_d);       /* 100ms 주기 */
void        bms_fault_notify_link(void);            /* EVSE 프레임 수신 시 호출 */
const char *bms_fault_name(uint16_t flt);           /* 최상위 1개 Fault 이름 */

/* --- 과온 임계 런타임 조정 (CAN 0x205) --- */
int32_t     bms_fault_get_ot_threshold(void);       /* [0.1C] */
int32_t     bms_fault_set_ot_threshold(int32_t c10);/* 클램프 후 적용값 반환 */
#endif
