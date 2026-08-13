/**
 * @file    pack_sensor.h
 * @brief   팩 전압/전류 센서 선택 계층 (INA226 / INA219)
 *
 * @note    app 계층은 칩 이름을 몰라야 한다. 부품이 바뀌어도 고치는 곳은 bms_app.c 가
 *          아니라 bms_cfg.h 의 CFG_PACK_SENSOR 한 줄이다.
 *          (헤더 전용 inline 이라 계층이 하나 늘어도 코드는 늘지 않는다)
 *
 *          이 계층이 실제로 값을 한 사례: 최종 부품은 INA226 이지만 모듈이 없어 지금은
 *          INA219 로 돈다. 레지스터 스케일이 전부 달라 드라이버를 통째로 갈았는데도
 *          app 코드는 한 줄도 안 바뀌었다. 지우면 bms_app.c 가 ina2xx.h 를 직접 알게 된다.
 *
 *          다른 센서를 붙여도 아래 5개 함수만 맞추면 된다. 선택되지 않은 드라이버는
 *          --gc-sections 가 걷어내므로 플래시에 남지 않는다.
 */
#ifndef __PACK_SENSOR_H_
#define __PACK_SENSOR_H_
#include "common_def.h"

#if (CFG_PACK_SENSOR == CFG_PACK_SENSOR_INA226)

#include "ina226.h"
#define PACK_SENSOR_NAME    "INA226"

static inline bool    pack_sensor_init(void)            { return ina226_init(); }
static inline bool    pack_sensor_update(void)          { return ina226_update(); }
static inline int32_t pack_sensor_get_bus_mv(void)      { return ina226_get_bus_mv(); }
static inline int32_t pack_sensor_get_current_ma(void)  { return ina226_get_current_ma(); }
static inline bool    pack_sensor_is_ok(void)           { return ina226_is_ok(); }

#elif (CFG_PACK_SENSOR == CFG_PACK_SENSOR_INA219)

#include "ina219.h"
#define PACK_SENSOR_NAME    "INA219"

static inline bool    pack_sensor_init(void)            { return ina219_init(); }
static inline bool    pack_sensor_update(void)          { return ina219_update(); }
static inline int32_t pack_sensor_get_bus_mv(void)      { return ina219_get_bus_mv(); }
static inline int32_t pack_sensor_get_current_ma(void)  { return ina219_get_current_ma(); }
static inline bool    pack_sensor_is_ok(void)           { return ina219_is_ok(); }

#else
#error "CFG_PACK_SENSOR must be CFG_PACK_SENSOR_INA226 or CFG_PACK_SENSOR_INA219"
#endif

#endif /* __PACK_SENSOR_H_ */
