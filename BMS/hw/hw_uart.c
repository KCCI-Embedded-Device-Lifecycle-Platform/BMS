/**
 * @file    hw_uart.c
 * @brief   USART2 (ST-LINK VCP, 115200-8N1) 래퍼
 * @note    송신은 블로킹 (디버그 로그 용도라 순서 보장이 더 중요),
 *          수신은 인터럽트 + 링버퍼 (ISR 최소 연산 원칙).
 */
#include "hw_uart.h"

extern UART_HandleTypeDef huart2;       /* CubeMX 생성 (usart.c) */

/* --- RX 링버퍼 : ISR 과 메인루프가 공유하므로 volatile 필수 --- */
static volatile uint8_t  s_rx_buf[HW_UART_RX_BUF_SIZE];
static volatile uint16_t s_rx_head;     /* ISR 이 쓴다  */
static volatile uint16_t s_rx_tail;     /* main 이 읽는다 */
static uint8_t           s_rx_tmp;      /* HAL 수신 임시 1바이트 */

void hw_uart_init(void)
{
    s_rx_head = 0;
    s_rx_tail = 0;
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_tmp, 1);
}

void hw_uart_write(const uint8_t *p_data, uint16_t len)
{
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)p_data, len, 100);
}

void hw_uart_puts(const char *p_str)
{
    hw_uart_write((const uint8_t *)p_str, (uint16_t)strlen(p_str));
}

bool hw_uart_get_byte(uint8_t *p_byte)
{
    if (s_rx_head == s_rx_tail) {
        return false;                   /* 비어 있음 */
    }
    *p_byte   = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & (HW_UART_RX_BUF_SIZE - 1U));
    return true;
}

/* --- HAL weak 콜백 오버라이드 : main.c 를 건드리지 않고 후킹 --- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART2) {
        return;
    }

    next = (uint16_t)((s_rx_head + 1U) & (HW_UART_RX_BUF_SIZE - 1U));
    if (next != s_rx_tail) {            /* 가득 차면 새 데이터 버림 (덮어쓰기 금지) */
        s_rx_buf[s_rx_head] = s_rx_tmp;
        s_rx_head = next;
    }
    (void)HAL_UART_Receive_IT(&huart2, &s_rx_tmp, 1);   /* 다음 1바이트 재무장 */
}

/**
 * @brief  newlib 의 _write() -> __io_putchar() 경로를 후킹하여 printf 를 UART 로 보낸다.
 * @note   CubeMX 가 생성한 syscalls.c 를 수정할 필요가 없다.
 */
int __io_putchar(int ch)
{
    uint8_t c = (uint8_t)ch;
    (void)HAL_UART_Transmit(&huart2, &c, 1, 10);
    return ch;
}
