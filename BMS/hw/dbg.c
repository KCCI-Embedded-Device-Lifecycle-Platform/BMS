#include "dbg.h"
#include "hw_uart.h"

/* 128B 였을 때 1초 요약 줄의 최악 케이스가 120B 로 8B 밖에 안 남아 있었다.
 * 넘치면 잘리는 것이 하필 끝의 CRLF 라 다음 줄과 붙어 버려서, 증상이
 * "로그가 깨진다" 로만 보이고 원인이 버퍼라는 것은 드러나지 않았다.
 * 지금은 (1) 여유를 늘리고 (2) CRLF 를 본문과 분리해 절대 안 잘리게 하고
 * (3) 잘린 줄에는 '~' 를 남긴다. */
#define DBG_BUF_SIZE    192U

/* hexdump 한 줄에 실을 최대 바이트 수. CAN 은 8이면 충분하지만 여유를 둔다. */
#define DBG_HEX_MAX     16U

/* static : 스택에 매번 잡지 않기 위함.
 * 단일 스레드 super-loop 이고 ISR 에서 호출하지 않는다는 전제.  */
static char s_buf[DBG_BUF_SIZE];

void dbg_init(void)
{
    hw_uart_init();
    hw_uart_puts("\r\n\r\n");
    hw_uart_puts("=========================================\r\n");
    hw_uart_puts(" EV Charger BMS   STM32F446RE / CAN 500k\r\n");
    dbg_printf(" FW %u.%u    console : 'h' = help\r\n",
               CFG_FW_VERSION_MAJOR, CFG_FW_VERSION_MINOR);
    hw_uart_puts("=========================================\r\n");
}

void dbg_log(char level, const char *fmt, ...)
{
    va_list  ap;
    int      n;
    int      head;
    size_t   room;
    uint32_t t = HAL_GetTick();

    /* 초.십분의일초. raw ms(7자리)보다 폭이 좁고, 슬롯이 100ms 단위라 이 분해능이면
     * 어느 슬롯에서 나온 줄인지 가려진다. HAL_GetTick() 은 한 번만 부른다 —
     * 두 번 부르면 ms 경계에서 초와 소수부가 다른 시각을 가리킬 수 있다. */
    head = snprintf(s_buf, DBG_BUF_SIZE, "[%5lu.%lu %c] ",
                    (unsigned long)(t / 1000UL),
                    (unsigned long)((t % 1000UL) / 100UL),
                    level);
    if ((head < 0) || (head >= (int)DBG_BUF_SIZE)) {
        return;
    }
    room = DBG_BUF_SIZE - (size_t)head;

    va_start(ap, fmt);
    n = vsnprintf(&s_buf[head], room, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }

    if (n >= (int)room) {
        /* vsnprintf 는 "썼으면 필요했을 길이" 를 돌려주므로 실제 길이로 되돌린다.
         * 잘렸다는 사실 자체가 진단 정보다 (버퍼를 늘려야 한다는 뜻). */
        n = (int)room - 1;
        if (n > 0) {
            s_buf[head + n - 1] = '~';
        }
    }

    hw_uart_write((const uint8_t *)s_buf, (uint16_t)(head + n));
    /* CRLF 를 본문과 나눠 쓴다. 한 버퍼에 같이 담으면 본문이 길 때 줄바꿈부터
     * 잘려서 줄이 서로 붙는다 — 그게 가장 알아보기 어려운 실패였다. */
    hw_uart_puts("\r\n");
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

const char *dbg_temp(int32_t c10)
{
    /* 3개 순환 : bms_fault_set_ot_threshold() 가 한 줄에 두 개를 찍는다.
     * 한 개짜리 static 이면 두 번째 호출이 첫 번째를 덮어써서 같은 값이 두 번 나온다. */
    static char    s_t[3][12];
    static uint8_t s_idx;
    char          *p;
    int32_t        a;

    s_idx = (uint8_t)((s_idx + 1U) % 3U);
    p     = s_t[s_idx];

    if (c10 == BMS_TEMP_INVALID) {
        /* "센서 없음" 과 "0.0C 실측" 은 완전히 다른 사실이라 숫자로 찍으면 안 된다.
         * 예전에는 이것이 -999.-9C 로 나와서 오측정처럼 보였다. */
        (void)snprintf(p, sizeof(s_t[0]), "---");
    } else {
        /* 클램프는 버퍼 방어가 아니라 **진단 방어**다. NTC LUT 가 깨졌거나 원격
         * 파라미터가 이상하게 들어와 32bit 쓰레기값이 오면, 잘린 문자열이 그럴듯한
         * 온도처럼 보이는 것이 최악이다. 범위 밖은 범위 끝값으로 눈에 띄게 만든다.
         * (부수적으로 자릿수 상한이 정해져 -Wformat-truncation 도 사라진다) */
        c10 = CLAMP(c10, -9999, 9999);
        a   = (c10 < 0) ? -c10 : c10;
        (void)snprintf(p, sizeof(s_t[0]), "%s%ld.%ld",
                       (c10 < 0) ? "-" : "", (long)(a / 10), (long)(a % 10));
    }
    return p;
}

void dbg_hexdump(const char *tag, const uint8_t *p, uint16_t len)
{
    static const char k_hex[] = "0123456789ABCDEF";
    char              hex[(DBG_HEX_MAX * 3U) + 4U];
    uint16_t          i;
    uint16_t          n   = 0;
    uint16_t          cnt = (len > DBG_HEX_MAX) ? DBG_HEX_MAX : len;

    for (i = 0; i < cnt; i++) {
        if (i != 0U) {
            hex[n++] = ' ';
        }
        hex[n++] = k_hex[(p[i] >> 4) & 0x0FU];
        hex[n++] = k_hex[p[i] & 0x0FU];
    }
    if (len > DBG_HEX_MAX) {
        hex[n++] = '.'; hex[n++] = '.';      /* 잘렸다는 사실을 남긴다 */
    }
    hex[n] = 0;

    /* 레벨 문자를 'T' 로 따로 준다. 트레이스는 초당 20줄이 넘는 최대 출력원이라
     * 다른 로그와 한눈에 갈리는 것이 중요하고, DBG 레벨이 아니라 콘솔 't' 로
     * 켜고 끄는 것이라 DBG_I 로 묶으면 레벨을 내렸을 때 't' 가 조용히 무력해진다.
     *
     * 한 줄을 통째로 만들어 넘기는 것도 의도다. 예전에는 바이트마다 dbg_printf() 를
     * 불러서 (1) 줄머리(시간)가 안 붙고 (2) 한 줄에 HAL_UART_Transmit 이 10번 넘게
     * 나갔다. 프레임 간격을 보려고 켜는 트레이스에 시간이 없으면 켤 이유가 없다. */
    dbg_log('T', "%s [%u] %s", tag, len, hex);
}
