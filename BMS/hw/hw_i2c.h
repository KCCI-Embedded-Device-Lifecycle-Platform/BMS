#ifndef __HW_I2C_H_
#define __HW_I2C_H_
#include "common_def.h"

void hw_i2c_init(void);
bool hw_i2c_is_ready(uint8_t addr7);
bool hw_i2c_write(uint8_t addr7, const uint8_t *p_data, uint16_t len);
bool hw_i2c_reg_write16(uint8_t addr7, uint8_t reg, uint16_t val);   /* 빅엔디안 */
bool hw_i2c_reg_read16(uint8_t addr7, uint8_t reg, uint16_t *p_val);
bool hw_i2c_recover(void);      /* 버스 lock-up 시 9클럭 강제 토글 */
uint8_t hw_i2c_scan(void);      /* 0x08~0x77 스캔, 발견 주소를 로그로 출력. 반환=개수 */
#endif
