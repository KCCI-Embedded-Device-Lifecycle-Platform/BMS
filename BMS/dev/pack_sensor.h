/**
 * @file    pack_sensor.h
 * @brief   팩 전류/EVSE측 버스 전압 센서 추상화 계층 (현재 INA226)
 * @note    현재 실물 순서는 EVSE -> INA226 -> ACS712 -> 릴레이 -> 배터리다.
 *          따라서 Bus Voltage는 배터리 PACK이 아니라 릴레이 바깥 EVSE측 전압이며,
 *          배터리 PACK 전압은 cell_adc의 B+ 노드를 사용한다.
 *
 *          app 계층은 칩 이름을 몰라야 한다. 부품이 바뀌어도 고치는 곳은 bms_app.c 가
 *          아니라 이 헤더 하나다.
 *          (헤더 전용 inline 이라 계층이 하나 늘어도 코드는 늘지 않는다)
 *
 *          이 계층이 실제로 값을 한 사례: INA226 모듈이 없던 기간에 INA219 로 돌렸는데,
 *          레지스터 스케일이 전부 달라 드라이버를 통째로 갈았는데도 app 코드는 한 줄도
 *          안 바뀌었다. 2026-08-14 에 INA226 이 입고되어 INA219 는 제거했지만,
 *          이 계층은 남긴다 — 지우면 bms_app.c 가 ina226.h 를 직접 알게 되고,
 *          다음 부품 교체 때 다시 app 계층까지 번진다.
 *
 *          다른 센서를 붙일 때 맞춰야 하는 것은 아래 5개 함수뿐이다.
 */
#ifndef __PACK_SENSOR_H_
#define __PACK_SENSOR_H_
#include "common_def.h"

#include "ina226.h"
#define PACK_SENSOR_NAME    "INA226"

static inline bool    pack_sensor_init(void)            { return ina226_init(); }
static inline bool    pack_sensor_update(void)          { return ina226_update(); }
static inline int32_t pack_sensor_get_bus_mv(void)      { return ina226_get_bus_mv(); }
static inline int32_t pack_sensor_get_current_ma(void)  { return ina226_get_current_ma(); }
static inline bool    pack_sensor_is_ok(void)           { return ina226_is_ok(); }
static inline bool    pack_sensor_is_saturated(void)    { return ina226_is_saturated(); }
static inline int32_t pack_sensor_get_full_scale_ma(void) { return ina226_get_full_scale_ma(); }
/* 원값(션트 전압). 전류가 0 으로 보일 때 "정말 안 흐른다" 와 "값이 안 들어온다" 를
 * 가르는 유일한 창구다 — 계산된 mA 만으로는 둘이 똑같이 0 이다. */
static inline int32_t pack_sensor_get_shunt_uv(void)    { return ina226_get_shunt_uv(); }

#endif /* __PACK_SENSOR_H_ */
