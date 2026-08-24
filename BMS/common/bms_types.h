/**
 * @file    bms_types.h
 * @brief   BMS 전역 공용 타입 정의
 * @note    모든 물리량은 정수 고정소수점으로 표현한다.
 *          - 전압 : mV   (1 = 0.001V)
 *          - 전류 : mA   (1 = 0.001A,  + = 충전 / - = 방전)
 *          - 온도 : 0.1C (1 = 0.1도,   예: 255 = 25.5C)
 *          float 를 쓰지 않는 이유: 100ms 루프와 ISR 에서의 변환 비용과
 *          printf 코드 크기를 피하기 위해서다.
 */
#ifndef __BMS_TYPES_H_
#define __BMS_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

#define BMS_CELL_COUNT      4U
#define BMS_TEMP_INVALID    (-9999)     /* 온도 측정 불가(단선/단락) */

/* ------------------------------------------------------------------
 * BMS 상태 (FSM)
 * ------------------------------------------------------------------ */
typedef enum {
    BMS_ST_INIT = 0,        /* 주변장치 초기화            */
    BMS_ST_SELF_CHECK,      /* 센서 자가진단              */
    BMS_ST_IDLE,            /* 대기 (감시만 수행)         */
    BMS_ST_CHARGE_READY,    /* 충전 허가 (permit = 1)     */
    BMS_ST_CHARGING,        /* 충전 중                    */
    BMS_ST_FAULT,           /* 이상 (permit = 0)          */
    BMS_ST_MAX
} bms_state_t;

/* ------------------------------------------------------------------
 * Fault 코드 : 비트마스크
 *   복수 Fault 동시 발생을 하나의 변수로 표현하기 위해 비트필드 사용.
 *   CAN 0x100 Data[7] 에는 하위 8bit만 실린다.
 * ------------------------------------------------------------------ */
typedef enum {
    BMS_FLT_NONE         = 0x0000U,
    BMS_FLT_CELL_OV      = 0x0001U,     /* 셀 과전압       */
    BMS_FLT_CELL_UV      = 0x0002U,     /* 셀 저전압       */
    BMS_FLT_PACK_OV      = 0x0004U,     /* 팩 과전압       */
    BMS_FLT_OVER_CURRENT = 0x0008U,     /* 과전류          */
    BMS_FLT_OVER_TEMP    = 0x0010U,     /* 과온            */
    BMS_FLT_SENSOR_ERR   = 0x0020U,     /* 센서 불일치/무응답 */
    BMS_FLT_LINK_TIMEOUT = 0x0040U,     /* EVSE 통신 두절  */
    BMS_FLT_IMBALANCE    = 0x0080U      /* 셀 불균형(Warning) */
} bms_fault_t;

/* Warning 은 충전 금지 사유에서 제외한다 */
#define BMS_FLT_CRITICAL_MASK   (BMS_FLT_CELL_OV | BMS_FLT_CELL_UV      | \
                                 BMS_FLT_PACK_OV | BMS_FLT_OVER_CURRENT | \
                                 BMS_FLT_OVER_TEMP | BMS_FLT_SENSOR_ERR | \
                                 BMS_FLT_LINK_TIMEOUT)

/* Fault 는 8개가 상한이다 (0x100 Data[7] / 0x103 Data[1] 이 1바이트) — 지금 다 썼다.
 * 내부 필드가 uint16_t 라 9번째를 추가해도 컴파일은 통과하고, (fault & 0xFF) 에서
 * 조용히 잘려 "BMS 는 FAULT 인데 EVSE 는 모른다" 로만 나타난다 -> 빌드에서 막는다.
 * 9번째가 정말 필요하면 프레임 포맷 변경이 먼저다 (0x103 을 DLC 3 으로 늘리는 것이 가장 싸다). */
#define BMS_FLT_WIRE_MASK       0x00FFU
_Static_assert(((BMS_FLT_CRITICAL_MASK | BMS_FLT_IMBALANCE) & ~BMS_FLT_WIRE_MASK) == 0U,
               "BMS fault bits must fit in 8bit (CAN 0x100 Data[7] / 0x103 Data[1])");

/* ------------------------------------------------------------------
 * 측정/판단 결과 블랙보드
 *   app 계층에서 100ms 마다 dev 계층 값을 모아 한 번에 채운다.
 *   fault / soc / fsm / ui / link 는 모두 이 구조체만 참조한다.
 * ------------------------------------------------------------------ */
typedef struct {
    /* --- 측정값 --- */
    int32_t     node_mv[BMS_CELL_COUNT];    /* B1,B2,B3,B+ 누적 전압 */
    int32_t     cell_mv[BMS_CELL_COUNT];    /* 차분한 셀별 전압      */
    int32_t     pack_mv;                    /* 셀 ADC B+ 기반 배터리 팩 전압 */
    int32_t     evse_bus_mv;                /* INA226 EVSE측 Bus Voltage    */
    int32_t     pack_ma;                    /* INA226 션트 전류              */
    int32_t     acs_ma;                     /* ACS712 홀 전류(대조)  */
    int32_t     temp_c10;                   /* NTC 온도 0.1C         */
    int32_t     vdda_mv;                    /* Vrefint 로 실측한 VDDA*/

    /* --- 파생값 --- */
    int32_t     cell_max_mv;
    int32_t     cell_min_mv;
    int32_t     imbalance_mv;               /* max - min             */
    uint8_t     soc;                        /* 0 ~ 100 %             */

    /* --- EVSE 수신 상태 (CAN 0x200 / 0x201 / 0x202) ---
     * bms_can.c 의 값을 ap_collect() 가 여기로 옮긴다.
     * "판단 계층은 블랙보드만 읽는다" 를 지키기 위한 통로다. */
    bool        evse_charge_req;            /* 0x201 : 충전 시작 요청 */
    bool        evse_connected;             /* 0x200 : 커넥터 체결    */
    bool        evse_relay_on;              /* 0x200 : EVSE 릴레이 on */
    bool        evse_estop;                 /* 0x200 : 비상정지 눌림  */
    uint8_t     evse_fault;                 /* 0x202 : EVSE 측 Fault  */

    /* --- 판단 결과 --- */
    uint16_t    fault;                      /* bms_fault_t 비트마스크 */
    bms_state_t state;
    bool        charge_permit;
    bool        sensor_ready;               /* dev 초기화 성공 여부   */
    bool        pack_current_saturated;     /* INA226 포화 -> ACS712 전류 채택 */
    bool        relay_on;                   /* PB5 충전 차단 릴레이 현재 지령 */
} bms_data_t;

#endif /* __BMS_TYPES_H_ */
