#ifndef __INA226_H_
#define __INA226_H_
#include "common_def.h"

/* --- 레지스터 맵 (데이터시트 Table 3) ---
 * INA219 와 0x00~0x05 는 이름이 같지만 의미/스케일이 다르다.
 * 0xFE/0xFF 는 INA219 에 없는 ID 레지스터로, 모듈 실장 여부 확인에 쓴다. */
#define INA226_REG_CONFIG       0x00U
#define INA226_REG_SHUNT_V      0x01U
#define INA226_REG_BUS_V        0x02U
#define INA226_REG_POWER        0x03U
#define INA226_REG_CURRENT      0x04U
#define INA226_REG_CALIB        0x05U
#define INA226_REG_MASK_EN      0x06U
#define INA226_REG_ALERT_LIM    0x07U
#define INA226_REG_MFG_ID       0xFEU       /* 0x5449 = "TI" */
#define INA226_REG_DIE_ID       0xFFU       /* 0x2260 */

bool    ina226_init(void);
bool    ina226_update(void);            /* 100ms 주기 */
int32_t ina226_get_bus_mv(void);
int32_t ina226_get_current_ma(void);    /* + 충전 / - 방전 */
int32_t ina226_get_shunt_uv(void);
bool    ina226_is_ok(void);
#endif
