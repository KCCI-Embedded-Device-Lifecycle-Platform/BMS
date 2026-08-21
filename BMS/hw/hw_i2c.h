#ifndef __HW_I2C_H_
#define __HW_I2C_H_
#include "common_def.h"

/**
 * @brief  I2C 버스 선택
 *
 * @note   INA226 과 OLED 를 **물리적으로 다른 버스**에 둔다. 원래 둘 다 I2C1(PB6/PB7)에
 *         있었고 주소(0x40 / 0x3C)가 달라 전기적으로는 문제가 없었지만, 실장에서 걸린다:
 *
 *         1) **풀업이 병렬로 붙는다.** 두 모듈 다 4.7k 를 온보드로 달고 나오므로 공유하면
 *            2.35k 가 되어 상승엣지가 뭉개진다 -> 한쪽 모듈의 풀업을 **떼어내야** 했다.
 *            버스를 나누면 각 모듈이 자기 풀업을 그대로 쓰므로 그 납땜이 사라진다.
 *         2) 배선이 Y 분기 없이 모듈당 2선으로 끝난다.
 *         3) 한쪽 버스가 죽었을 때 **범인이 곧바로 특정된다.** 공유일 때는 모듈을
 *            하나씩 떼면서 찾아야 했다.
 *
 * @warning **I2C2 가 아니라 I2C3 이다.** F446 은 I2C 계열 AF 가 AF4 뿐이라
 *          I2C2_SDA 후보가 PB11/PF0/PH5 인데 **LQFP64 에는 셋 다 핀이 없다.**
 *          (다른 F4 에 있는 PB3 AF9 매핑은 F446 에 존재하지 않는다 —
 *           GPIO_AF9_I2C2 가 stm32f4xx_hal_gpio_ex.h 의 F446 블록에 아예 없다.)
 *          그래서 남은 선택지는 I2C3(SCL=PA8 / SDA=PC9) 하나뿐이었고,
 *          그 두 핀을 비우려고 **릴레이를 PA8->PB5 로 옮기고 부저(PC9)를 제거**했다.
 *          => 이 버스 배치를 되돌리려면 그 두 가지도 같이 되돌려야 한다.
 *
 * @warning 버스를 나눠도 **super-loop 블로킹은 줄지 않는다.** HAL 을 폴링으로 쓰므로
 *          OLED flush 23ms 동안 CPU 가 잡혀 있는 것은 그대로다. 이 분리는 전기/실장
 *          문제를 푸는 것이지 스케줄링 문제를 푸는 것이 아니다 —
 *          OLED 를 100ms 슬롯으로 옮겨도 된다는 뜻이 아니다.
 */
typedef enum {
    HW_I2C_SENSOR = 0,      /* I2C1 : PB6 SCL / PB7 SDA -> INA226 (0x40)  */
    HW_I2C_OLED,            /* I2C3 : PA8 SCL / PC9 SDA -> SSD1306 (0x3C) */
    HW_I2C_MAX
} hw_i2c_bus_t;

void hw_i2c_init(void);     /* 두 버스를 모두 초기화 */
bool hw_i2c_is_ready(hw_i2c_bus_t bus, uint8_t addr7);
bool hw_i2c_write(hw_i2c_bus_t bus, uint8_t addr7, const uint8_t *p_data, uint16_t len);
bool hw_i2c_reg_write16(hw_i2c_bus_t bus, uint8_t addr7, uint8_t reg, uint16_t val);  /* 빅엔디안 */
bool hw_i2c_reg_read16(hw_i2c_bus_t bus, uint8_t addr7, uint8_t reg, uint16_t *p_val);
bool hw_i2c_recover(hw_i2c_bus_t bus);      /* 버스 lock-up 시 9클럭 강제 토글 */
uint8_t hw_i2c_scan(hw_i2c_bus_t bus);      /* 0x08~0x77 스캔, 발견 주소를 로그로 출력. 반환=개수 */
#endif
