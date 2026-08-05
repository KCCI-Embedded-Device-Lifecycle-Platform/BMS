#ifndef __HW_UART_H_
#define __HW_UART_H_
#include "common_def.h"

#define HW_UART_RX_BUF_SIZE     64U     /* 반드시 2의 거듭제곱 (마스크 연산) */

void    hw_uart_init(void);
void    hw_uart_write(const uint8_t *p_data, uint16_t len);
void    hw_uart_puts(const char *p_str);
bool    hw_uart_get_byte(uint8_t *p_byte);   /* 논블로킹, 링버퍼에서 1바이트 */
#endif
