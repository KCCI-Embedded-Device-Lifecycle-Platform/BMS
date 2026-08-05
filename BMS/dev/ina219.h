#ifndef __INA219_H_
#define __INA219_H_
#include "common_def.h"

/* --- 레지스터 맵 (데이터시트 Table 5) --- */
#define INA219_REG_CONFIG       0x00U
#define INA219_REG_SHUNT_V      0x01U
#define INA219_REG_BUS_V        0x02U
#define INA219_REG_POWER        0x03U
#define INA219_REG_CURRENT      0x04U
#define INA219_REG_CALIB        0x05U

bool    ina219_init(void);
bool    ina219_update(void);            /* 100ms 주기 */
int32_t ina219_get_bus_mv(void);
int32_t ina219_get_current_ma(void);    /* + 충전 / - 방전 */
int32_t ina219_get_shunt_uv(void);
bool    ina219_is_ok(void);
#endif
