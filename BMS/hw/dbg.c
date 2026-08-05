#include "dbg.h"
#include "hw_uart.h"

#define DBG_BUF_SIZE    128U

/* static : 스택에 128B 를 매번 잡지 않기 위함.
 * 단일 스레드 super-loop 이고 ISR 에서 호출하지 않는다는 전제.  */
static char s_buf[DBG_BUF_SIZE];

void dbg_init(void)
{
    hw_uart_init();
    hw_uart_puts("\r\n\r\n");
    hw_uart_puts("===============================\r\n");
    hw_uart_puts(" EV Charger BMS  (STM32F446RE)\r\n");
    hw_uart_puts(" Stage 1 : Board / Arch bringup\r\n");
    hw_uart_puts("===============================\r\n");
}

void dbg_printf(const char *fmt, ...)
{
    va_list ap;
    int     n;

    va_start(ap, fmt);
    n = vsnprintf(s_buf, DBG_BUF_SIZE, fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n >= (int)DBG_BUF_SIZE) {
            n = (int)DBG_BUF_SIZE - 1;      /* 잘림 방지 */
        }
        hw_uart_write((const uint8_t *)s_buf, (uint16_t)n);
    }
}

void dbg_hexdump(const char *tag, const uint8_t *p, uint16_t len)
{
    uint16_t i;

    dbg_printf("%s [%u] ", tag, len);
    for (i = 0; i < len; i++) {
        dbg_printf("%02X ", p[i]);
    }
    dbg_printf("\r\n");
}
