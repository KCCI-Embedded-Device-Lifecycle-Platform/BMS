/**
 * @file    dbg.h
 * @brief   레벨별 디버그 로그 매크로
 * @note    BRINGUP_DBG_LEVEL 로 컴파일 타임에 제거되므로,
 *          최종 릴리스에서 0 으로 두면 코드/플래시가 남지 않는다.
 */
#ifndef __DBG_H_
#define __DBG_H_
#include "common_def.h"

void dbg_init(void);
void dbg_printf(const char *fmt, ...);
void dbg_hexdump(const char *tag, const uint8_t *p, uint16_t len);

#if (BRINGUP_DBG_LEVEL >= 1)
  #define DBG_E(fmt, ...)   dbg_printf("[E][%lu] " fmt "\r\n", (unsigned long)HAL_GetTick(), ##__VA_ARGS__)
#else
  #define DBG_E(fmt, ...)   do {} while (0)
#endif
#if (BRINGUP_DBG_LEVEL >= 2)
  #define DBG_W(fmt, ...)   dbg_printf("[W][%lu] " fmt "\r\n", (unsigned long)HAL_GetTick(), ##__VA_ARGS__)
#else
  #define DBG_W(fmt, ...)   do {} while (0)
#endif
#if (BRINGUP_DBG_LEVEL >= 3)
  #define DBG_I(fmt, ...)   dbg_printf("[I] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define DBG_I(fmt, ...)   do {} while (0)
#endif
#if (BRINGUP_DBG_LEVEL >= 4)
  #define DBG_D(fmt, ...)   dbg_printf("[D] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define DBG_D(fmt, ...)   do {} while (0)
#endif

#endif /* __DBG_H_ */
