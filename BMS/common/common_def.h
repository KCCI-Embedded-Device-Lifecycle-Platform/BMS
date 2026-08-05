/**
 * @file    common_def.h
 * @brief   프로젝트 전역 공통 헤더 (표준 라이브러리 + HAL + 공통 타입 일괄 포함)
 * @note    모든 .c 파일은 이 헤더 하나만 include 하면 된다.
 */
#ifndef __COMMON_DEF_H_
#define __COMMON_DEF_H_

/* ---------- C Standard Library ---------- */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <sys/types.h>

/* ---------- STM32 HAL / CubeMX ---------- */
#include "main.h"               /* CubeMX 생성 헤더 (내부에서 stm32f4xx_hal.h 포함) */

/* ---------- Project Common ---------- */
#include "bms_types.h"
#include "bms_cfg.h"
#include "bringup.h"

/* ---------- 공통 매크로 ---------- */
#define UNUSED_ARG(x)           ((void)(x))
#define ARRAY_SIZE(a)           (sizeof(a) / sizeof((a)[0]))

#define BIT(n)                  (1UL << (n))
#define FLAG_SET(v, f)          ((v) |=  (f))
#define FLAG_CLR(v, f)          ((v) &= ~(f))
#define FLAG_IS_SET(v, f)       (((v) & (f)) != 0U)

#ifndef MIN
#define MIN(a, b)               (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b)               (((a) > (b)) ? (a) : (b))
#endif
#define CLAMP(v, lo, hi)        (MIN(MAX((v), (lo)), (hi)))
#define ABS_DIFF(a, b)          (((a) > (b)) ? ((a) - (b)) : ((b) - (a)))

/* 반올림 나눗셈 : 정수 고정소수점 연산에서 절삭 오차를 줄인다 */
#define DIV_ROUND(n, d)         (((n) + ((d) / 2)) / (d))

/* 임계영역 (ISR과 공유하는 데이터 접근 시) */
#define CRITICAL_ENTER()        do { __disable_irq(); } while (0)
#define CRITICAL_EXIT()         do { __enable_irq();  } while (0)

#endif /* __COMMON_DEF_H_ */
