#ifndef __HW_SPI_H_
#define __HW_SPI_H_
#include "common_def.h"

/**
 * @brief  SPI2 (PB13 SCK / PB15 MOSI) 마스터 송신 전용 래퍼
 * @note   CS / DC / RES 는 SPI 페리페럴이 아니라 일반 GPIO 이므로
 *         hw_gpio 로 따로 제어한다 (HW_OUT_OLED_xxx).
 */
bool hw_spi_init(void);
bool hw_spi_write(const uint8_t *p_data, uint16_t len);
#endif
