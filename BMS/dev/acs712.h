#ifndef __ACS712_H_
#define __ACS712_H_
#include "common_def.h"

void    acs712_init(void);
bool    acs712_calibrate(void);     /* 무전류 상태에서 1회 호출 (오프셋 확정) */
void    acs712_update(void);        /* 500ms 주기 */
int32_t acs712_get_ma(void);
int32_t acs712_get_offset_uv(void);
bool    acs712_is_ok(void);
#endif
