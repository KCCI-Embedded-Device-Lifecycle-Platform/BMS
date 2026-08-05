/**
 * @file    bms_cfg.h
 * @brief   임계값 / 하드웨어 상수 / 주기 설정 집중 관리
 * @note    "매직넘버는 코드에 두지 않는다."
 *          모든 값을 여기 모아야 2단계 OTA 로 임계값을 바꾸는 시연이 가능하다.
 */
#ifndef __BMS_CFG_H_
#define __BMS_CFG_H_

/* ==================================================================
 * 0. 데모 모드
 *    실제 리튬 임계값은 위험하므로 시연 시 낮춘 값을 쓴다.
 * ================================================================== */
#define CFG_DEMO_MODE                   1

/* ==================================================================
 * 1. Fault 임계값
 * ================================================================== */
#define CFG_CELL_OV_MV                  4200        /* 셀 과전압 진입 */
#define CFG_CELL_OV_CLR_MV              4150        /* 해제 (히스테리시스 50mV) */
#define CFG_CELL_UV_MV                  3000
#define CFG_CELL_UV_CLR_MV              3100
#define CFG_PACK_OV_MV                  16800
#define CFG_PACK_OV_CLR_MV              16600
#define CFG_IMBALANCE_MV                150

#if (CFG_DEMO_MODE == 1)
  #define CFG_OVER_CURRENT_MA           1000        /* 데모: 1A */
  #define CFG_OVER_TEMP_C10             400         /* 데모: 40.0C */
  #define CFG_OVER_TEMP_CLR_C10         370
#else
  #define CFG_OVER_CURRENT_MA           2000
  #define CFG_OVER_TEMP_C10             550
  #define CFG_OVER_TEMP_CLR_C10         500
#endif

/* Fault 진입 확정 횟수 (100ms x N).
 * 3 = 300ms. ADC 노이즈 1회로 충전이 끊기는 것을 막는 값이다.
 * "임계 초과 후 100ms 이내 permit=0" 을 엄격하게 시연해야 하면 1 로 낮춘다. */
#define CFG_FAULT_CONFIRM_CNT           3U

/* 센서 크로스체크 : INA219 와 ACS712 전류 차이 허용치 */
#define CFG_SENSOR_DIFF_MA              500
/* EVSE 무응답 판정 시간 */
#define CFG_LINK_TIMEOUT_MS             1000
/* Fault 해제 유지 시간 (해제 조건이 이 시간 이상 유지되어야 복귀) */
#define CFG_FAULT_CLEAR_HOLD_MS         3000
/* LINK_TIMEOUT 전용 해제 유지 시간.
 * 이 값이 CFG_LINK_TIMEOUT_MS 보다 크면 "하트비트 1회 -> 해제" 가 원리적으로
 * 불가능해진다 (하트비트 유효기간 1초 < 해제 대기 3초). 반드시 아래로 둘 것.
 * 300ms = 100ms 슬롯 3회 연속 정상 수신. */
#define CFG_LINK_CLEAR_HOLD_MS          300
/* SELF_CHECK 에서 인정하는 셀 전압 범위 (배선 오류 검출용) */
#define CFG_SELFCHK_CELL_MIN_MV         2000
#define CFG_SELFCHK_CELL_MAX_MV         4500

/* FAULT 상태를 보드 온보드 LED(LD2 / PA5)로도 같이 점멸시킬지.
 * Fault LED 는 PC8 인데 이건 외부 LED 를 직접 배선해야 보이는 핀이다.
 * 1 이면 외부 LED 가 없어도 LD2 로 FAULT 를 눈으로 확인할 수 있다. */
#define CFG_FAULT_LED_MIRROR_LD2        1

/* ==================================================================
 * 2. 하드웨어 상수
 * ================================================================== */
/* --- 셀 전압 분압기 : 100k(상단) + 20k(하단) --> 1/6 --- */
#define CFG_DIV_R_TOP_OHM               100000L
#define CFG_DIV_R_BOT_OHM               20000L
/* 복원 배율을 x1000 스케일 정수로 보관 (6.000 -> 6000) */
#define CFG_DIV_SCALE_X1000             (((CFG_DIV_R_TOP_OHM + CFG_DIV_R_BOT_OHM) * 1000L) \
                                          / CFG_DIV_R_BOT_OHM)

/* --- ADC --- */
#define CFG_ADC_FULL_SCALE              4095L       /* 12bit */
#define CFG_ADC_OVERSAMPLE_N            16U         /* ISR 누적 평균 횟수 */
/* VREFINT_CAL : 공장 캘리브레이션 값. F411 / F446 모두 같은 주소다
 * (RM0390 / DS10693 - VREFIN_CAL @ 0x1FFF7A2A). 보드 교체와 무관. */
#define CFG_VREFINT_CAL_ADDR            ((uint16_t *)0x1FFF7A2AUL)
#define CFG_VREFINT_CAL_VDDA_MV         3300L       /* 캘리브레이션 시 VDDA */

/* --- INA219 --- */
#define CFG_INA219_I2C_ADDR             0x40U       /* 7bit */
#define CFG_INA219_SHUNT_MOHM           100L        /* 모듈 기본 R100 = 0.1ohm */

/* --- NTC --- */
#define CFG_NTC_PULLUP_OHM              10000L
#define CFG_NTC_R25_OHM                 10000L
/* B값은 LUT 에 이미 반영되어 있음 (B3950) */

/* --- ACS712-05B + 레벨시프트(10k 상단 / 20k 하단 = x2/3) --- */
#define CFG_ACS712_SENS_UV_PER_A        185000L     /* 185 mV/A */
#define CFG_ACS712_DIV_NUM              3L          /* 복원 배율 3/2 */
#define CFG_ACS712_DIV_DEN              2L
#define CFG_ACS712_CAL_SAMPLES          32U         /* 무전류 오프셋 캘리 횟수 */

/* --- OLED ---
 * SunFounder TS0229 / CN0219 같은 듀얼 모듈은 핀 개수로 모드를 판별한다.
 *   4핀 (GND VCC SCL SDA)              -> CFG_OLED_IFACE_I2C
 *   7핀 (GND VCC D0 D1 RES DC CS)      -> CFG_OLED_IFACE_SPI  (공장 기본값)
 * SPI 를 쓰면 I2C 버스가 INA219 전용으로 남아 100ms 팩 전류 측정이
 * 1KB 프레임버퍼 전송에 밀리지 않는다.
 */
#define CFG_OLED_IFACE_I2C              0
#define CFG_OLED_IFACE_SPI              1
#define CFG_OLED_IFACE                  CFG_OLED_IFACE_SPI

#define CFG_OLED_I2C_ADDR               0x3CU       /* 7bit, 모듈에 따라 0x3D */
#define CFG_OLED_WIDTH                  128U
#define CFG_OLED_HEIGHT                 64U
/* 컨트롤러 컬럼 오프셋 : SSD1306 = 0, SH1106 = 2.
 * 화면이 좌우로 2픽셀 밀려 보이면 SH1106 이므로 2 로 바꾼다. */
#define CFG_OLED_COL_OFFSET             0U

/* --- SPI2 (OLED SPI 모드 전용) ---
 *   PB13 SCK / PB15 MOSI (AF5)  : hw_spi 가 직접 설정한다 (CubeMX 미설정)
 *   PB12 CS / PC7 DC / PC6 RES  : CubeMX 가 GPIO 출력으로 이미 설정
 * SPI2 는 APB1(42MHz). /8 = 5.25MHz 로 SSD1306 상한 10MHz 안쪽이다. */
#define CFG_OLED_SPI_CLK_DIV            8U

/* ==================================================================
 * 3. 스케줄 주기
 * ================================================================== */
#define CFG_TASK_100MS                  100U
#define CFG_TASK_500MS                  500U
#define CFG_TASK_1000MS                 1000U

/* ==================================================================
 * 4. 통신
 * ================================================================== */
/* --- 전송 백엔드 선택 ---
 * UART : 1단계. 로직 분석기 없이 터미널로 프레임을 눈으로 검증한다.
 * CAN  : 2단계. F446 의 bxCAN(CAN1, PB8 RX / PB9 TX)을 쓴다.
 *        페이로드 바이트 맵은 둘이 100% 동일하다. */
#define CFG_LINK_TRANSPORT_UART         0
#define CFG_LINK_TRANSPORT_CAN          1
#define CFG_LINK_TRANSPORT              CFG_LINK_TRANSPORT_CAN

/* ---- 송신 ID (BMS -> EVSE) ---- */
#define CFG_CAN_ID_MAIN                 0x100U      /* 100ms */
#define CFG_CAN_ID_CELL                 0x101U      /* 500ms */
#define CFG_CAN_ID_TEMP                 0x102U      /* 500ms */
#define CFG_CAN_ID_STATE                0x103U      /* 100ms */
#define CFG_CAN_ID_VERSION              0x104U      /* 1s    */
#define CFG_CAN_ID_BMS_RESP             0x105U      /* 응답 (요청 시에만) */

/* ---- 수신 ID (EVSE -> BMS) ---- */
#define CFG_CAN_ID_EVSE_STATUS          0x200U      /* 하트비트 겸용 */
#define CFG_CAN_ID_EVSE_CHARGE_REQ      0x201U
#define CFG_CAN_ID_EVSE_FAULT           0x202U
#define CFG_CAN_ID_BMS_OTA_ENTER        0x203U
#define CFG_CAN_ID_PARAM_WRITE          0x205U      /* 0x201 충돌 회피 */

/* ---- 응답 코드 (0x105 Data[0]) ---- */
#define BMS_RESP_PARAM_ACK              0x01U
#define BMS_RESP_PARAM_NAK              0x02U
#define BMS_RESP_OTA_REJECT             0x10U
#define BMS_REJECT_NOT_SUPPORTED        0x01U

/* ---- 파라미터 ID (0x205 Data[0]) ---- */
#define PARAM_ID_OVER_TEMP              0x01U
#define PARAM_ID_OVER_CURRENT           0x02U
#define PARAM_ID_CELL_OV                0x03U
#define PARAM_MAGIC                     0xA5U       /* 오조작/노이즈 1차 차단 */

/* ---- 원격 파라미터 안전 범위 ----
 * 원격 명령이 배터리 보호를 무력화하지 못하도록 하드코딩된 범위로 강제
 * 클램프한다. 버스 노이즈나 상위 노드 오류로 200C 가 들어와도 60C 를 넘지 않는다.
 * (CFG_OVER_TEMP_C10 이 데모 40C 이므로 하한은 그보다 낮게 둔다) */
#define CFG_OT_LIMIT_MIN_C10            300         /* 30.0C */
#define CFG_OT_LIMIT_MAX_C10            600         /* 60.0C */
#define CFG_OT_HYSTERESIS_C10           30          /* 해제 임계 = 진입 - 3.0C */

#define CFG_FW_VERSION_MAJOR            0U
#define CFG_FW_VERSION_MINOR            1U

#endif /* __BMS_CFG_H_ */
