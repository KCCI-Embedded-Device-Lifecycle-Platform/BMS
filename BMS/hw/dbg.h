/**
 * @file    dbg.h
 * @brief   레벨별 디버그 로그 매크로
 * @note    BRINGUP_DBG_LEVEL 로 컴파일 타임에 제거되므로,
 *          최종 릴리스에서 0 으로 두면 코드/플래시가 남지 않는다.
 *
 * 줄 머리(타임스탬프 + 레벨)는 dbg_log() 가 붙이고 **모든 레벨이 같은 폭**이다.
 * 예전에는 DBG_E/DBG_W 만 tick 을 달고 DBG_I 는 "[I] " 로 끝나서 왼쪽 끝이 줄마다
 * 달랐다. 그 불일치가 로그를 세로로 훑지 못하게 만든 주범이었고, 1초 요약(DBG_I)에
 * 시간이 없어 저장한 로그의 시간축이 통째로 끊기는 문제도 같이 있었다.
 */
#ifndef __DBG_H_
#define __DBG_H_
#include "common_def.h"

void dbg_init(void);

/** @brief 줄 머리(시간/레벨)와 CRLF 를 붙여 한 줄을 내보낸다. 매크로가 쓰는 실체. */
void dbg_log(char level, const char *fmt, ...);

/** @brief 줄 머리 없이 그대로 내보낸다 (hexdump 처럼 한 줄을 나눠 찍을 때만) */
void dbg_printf(const char *fmt, ...);

void dbg_hexdump(const char *tag, const uint8_t *p, uint16_t len);

/**
 * @brief  0.1 단위 고정소수점을 사람이 읽는 문자열로 (온도 전용)
 * @return "24.3" / "-5.5" / 센서 없음(BMS_TEMP_INVALID)이면 "---"
 * @note   "%ld.%ld" 로 찍으면 음수에서 소수점 뒤에도 부호가 붙는다
 *         (-9999 -> "-999.-9", -55 -> "-5.-5"). 반드시 이 함수를 쓸 것.
 *         반환 버퍼가 3개 순환이므로 한 printf 안에서 최대 3개까지 안전하다.
 */
const char *dbg_temp(int32_t c10);

#if (BRINGUP_DBG_LEVEL >= 1)
  #define DBG_E(fmt, ...)   dbg_log('E', fmt, ##__VA_ARGS__)
#else
  #define DBG_E(fmt, ...)   do {} while (0)
#endif
#if (BRINGUP_DBG_LEVEL >= 2)
  #define DBG_W(fmt, ...)   dbg_log('W', fmt, ##__VA_ARGS__)
#else
  #define DBG_W(fmt, ...)   do {} while (0)
#endif
#if (BRINGUP_DBG_LEVEL >= 3)
  #define DBG_I(fmt, ...)   dbg_log('I', fmt, ##__VA_ARGS__)
#else
  #define DBG_I(fmt, ...)   do {} while (0)
#endif
#if (BRINGUP_DBG_LEVEL >= 4)
  #define DBG_D(fmt, ...)   dbg_log('D', fmt, ##__VA_ARGS__)
#else
  #define DBG_D(fmt, ...)   do {} while (0)
#endif

#endif /* __DBG_H_ */
