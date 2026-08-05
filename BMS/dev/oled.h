#ifndef __OLED_H_
#define __OLED_H_
#include "common_def.h"

bool oled_init(void);
void oled_clear(void);
void oled_flush(void);                                      /* 프레임버퍼 -> 패널 */
void oled_draw_char(uint8_t x, uint8_t page, char c);
void oled_draw_str(uint8_t x, uint8_t page, const char *s); /* page 0~7 (8px 단위) */
void oled_printf(uint8_t x, uint8_t page, const char *fmt, ...);
bool oled_is_ok(void);
#endif
