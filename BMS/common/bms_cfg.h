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

/* --- 과온 임계는 데모에서도 낮추지 않는다 (EVSE 파트와 합의) ---
 * 과온은 3계층이고 트립 순서가 곧 설계다:
 *   EVSE 45.0C -> OTA 후 35.0C  <- 정책값. 여기가 먼저 트립해야 한다
 *   BMS  55.0C                  <- 배터리 보호 최후선
 *   보호보드 하드웨어 고정       <- 물리 차단
 * BMS 를 40C 로 낮추면 EVSE 가 자기 기준을 보기도 전에 permit=0 을 받아 릴레이를 꺼서
 * OTA 시연이 통째로 안 보인다. 시연 온도대(40C 근처)에서 BMS 가 트립하지 않는 것이 정상. */
#define CFG_OVER_TEMP_C10               550         /* 55.0C */
#define CFG_OVER_TEMP_CLR_C10           500         /* 50.0C */

#if (CFG_DEMO_MODE == 1)
  #define CFG_OVER_CURRENT_MA           1000        /* 데모: 1A */
#else
  #define CFG_OVER_CURRENT_MA           2000
#endif

/* Fault 진입 확정 횟수 (100ms x N). 3 = 300ms — ADC 노이즈 1회로 충전이 끊기는 것을 막는다.
 * "임계 초과 후 100ms 이내 permit=0" 을 엄격히 시연해야 하면 1 로 낮춘다. */
#define CFG_FAULT_CONFIRM_CNT           3U

/* 센서 크로스체크 : 팩 전류 센서(션트) 와 ACS712(홀) 전류 차이 허용치 */
#define CFG_SENSOR_DIFF_MA              500
/* EVSE 무응답 판정 시간 */
#define CFG_LINK_TIMEOUT_MS             1000
/* Fault 해제 유지 시간 (해제 조건이 이 시간 이상 유지되어야 복귀) */
#define CFG_FAULT_CLEAR_HOLD_MS         3000
/* LINK_TIMEOUT 전용 해제 유지 시간 (= 100ms 슬롯 3회 연속 정상 수신).
 * CFG_LINK_TIMEOUT_MS 보다 크면 하트비트 유효기간보다 해제 대기가 길어져
 * LINK_TIMEOUT 이 영구히 남는다. 반드시 그 아래로 둘 것. */
#define CFG_LINK_CLEAR_HOLD_MS          300
/* SELF_CHECK 에서 인정하는 셀 전압 범위 (배선 오류 검출용) */
#define CFG_SELFCHK_CELL_MIN_MV         2000
#define CFG_SELFCHK_CELL_MAX_MV         4500

/* FAULT 를 온보드 LD2(PA5)로도 같이 점멸시킬지.
 * Fault LED(PC8)는 외부 LED 를 배선해야 보이므로, 1 이면 배선 없이도 눈으로 확인된다. */
#define CFG_FAULT_LED_MIRROR_LD2        1

/* --- 충전 차단 릴레이 (PA8, NO 접점) ---
 * EVSE 릴레이를 대체하는 것이 아니라 permit 을 물리적으로 한 번 더 끊는 직렬 이중화다.
 * 최종 차단 권한은 EVSE 에 있다. 그래서 켜는 조건도 permit 보다 좁다 (ap_relay_update).
 *
 * !! 액티브 레벨을 실물로 확인할 것. 저가 모듈 상당수가 액티브 Low 이고, 이 값이 틀리면
 *    부팅하자마자 릴레이가 붙는다. 부팅 직후 릴레이가 떨어져 있어야 정상이다. */
#define CFG_RELAY_ACTIVE_HIGH           1           /* 1 = High 로 붙음 / 0 = Low 로 붙음 */

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

/* --- 팩 전압/전류 센서 : INA226 (I2C) ---
 * 2026-08-14 에 모듈이 입고되어 대체 부품(INA219) 경로를 제거했다.
 * 컴파일 타임 선택(CFG_PACK_SENSOR)도 같이 없앴다 — 고를 것이 하나뿐인 스위치는
 * "고를 수 있다" 는 착각만 남기고 실제로는 검증되지 않는 분기가 된다.
 * 그래도 app 계층은 여전히 dev/pack_sensor.h 만 보므로 칩 이름을 모른다.
 * 다른 칩으로 갈아탈 일이 생기면 고칠 곳은 bms_app.c 가 아니라 그 헤더 하나다.
 *
 * !! INA226 에는 PGA 가 없다. 션트 풀스케일이 +-81.92mV 로 고정이라
 *    "션트 값 = 측정 범위" 이고, 이것이 이 칩의 유일한 함정이다:
 *      0.1옴 (모듈 기본 R100) -> +-819mA   <- 과전류 임계 1A 를 아예 못 잰다
 *      0.01옴 (교체품)        -> +-8.19A
 *    포화는 런타임에 "전류가 안 올라가네" 로만 보이므로, ina226.c 가 이 값과
 *    CFG_OVER_CURRENT_MA 를 비교해 빌드에서 #warning 으로 잡는다.
 *    경고가 뜨면 임계값을 낮추지 말고 션트를 바꿀 것 (임계를 낮추면 실팩에서 무의미하다).
 *
 * 분해능은 션트 LSB 2.5uV / 버스 LSB 1.25mV + 하드웨어 평균(AVG=16회)이라
 * 이 프로젝트 임계값(500mA/1000mA, 16.8V)에는 여유가 충분하다.
 *
 * OLED(0x3C)와 같은 I2C1 버스지만 주소가 달라 충돌은 없다.
 * 풀업 4.7k 는 버스에 한 쌍만 남길 것 — 두 모듈에 다 붙어 있으면 2.35k 가 되어
 * 상승엣지가 뭉개진다. */
#define CFG_INA226_I2C_ADDR             0x40U       /* 7bit (A1/A0 = GND) */
#define CFG_INA226_SHUNT_MOHM           100L        /* 모듈 기본 R100. 0.01ohm 교체 시 10 */

/* --- I2C1 버스 속도 (.ioc 가 아니라 여기가 단일 소스) ---
 * hw_i2c_init() 이 MX_I2C1_Init() 뒤에 이 값으로 덮어쓰고 재초기화한다
 * (bms_can_init() 의 CAN 모드 덮어쓰기와 같은 패턴).
 *
 * 매크로로 뺀 이유는 이것이 **진단 도구**이기 때문이다:
 *   "모듈이 고장인가" vs "배선 신호 무결성인가" 를 가르는 가장 빠른 시험이
 *   400k -> 100k 로 내려 보는 것인데, .ioc 로 바꾸면 CubeMX 가 USER CODE 밖을
 *   통째로 재생성해서 벤치에서 한 번 해 보기엔 대가가 너무 크다.
 *
 * 100k 에서 되고 400k 에서 안 되면 모듈은 멀쩡하다 — 배선 길이/풀업/용량 문제다.
 * 그 경우 400k 로 되돌리는 것이 목표이지, 100k 로 눌러 두고 넘어가지 말 것
 * (OLED flush 시간이 23ms -> 약 92ms 로 늘어 500ms 슬롯을 크게 잡아먹는다). */
#define CFG_I2C_SPEED_HZ                400000L

/* --- NTC --- */
#define CFG_NTC_PULLUP_OHM              10000L
#define CFG_NTC_R25_OHM                 10000L
/* B값은 LUT 에 이미 반영되어 있음 (B3950) */

/* --- ACS712-05B + 레벨시프트(10k 상단 / 20k 하단 = x2/3) --- */
#define CFG_ACS712_SENS_UV_PER_A        185000L     /* 185 mV/A */
#define CFG_ACS712_DIV_NUM              3L          /* 복원 배율 3/2 */
#define CFG_ACS712_DIV_DEN              2L
#define CFG_ACS712_CAL_SAMPLES          32U         /* 무전류 오프셋 캘리 횟수 */

/* --- OLED (SSD1306 128x64, I2C 전용) ---
 * 최종 부품(ELB200168)이 I2C 4핀 전용이라 .ioc 에서 SPI2 를 내렸다 (그 자리에 PA8 릴레이).
 *
 * 대가는 flush 시간 : 8페이지 x 129byte x 9bit / 400kHz = 약 23ms 블로킹이고,
 * 같은 I2C1 을 쓰는 팩 센서 읽기도 그동안 밀린다.
 * -> OLED 는 500ms 슬롯에만 둔다. "Fault 후 100ms 내 permit=0" 이 걸린 100ms 슬롯에
 *    절대 넣지 말 것.
 *
 * SPI 로 되돌리려면 .ioc 에 SPI2 를 다시 켜는 것이 먼저다 (지금은
 * HAL_SPI_MODULE_ENABLED 가 꺼져 있어 코드만 복원하면 빌드가 깨진다). */
#define CFG_OLED_I2C_ADDR               0x3CU       /* 7bit, 모듈에 따라 0x3D */
#define CFG_OLED_WIDTH                  128U
#define CFG_OLED_HEIGHT                 64U
/* 대비 (0x81 레지스터). 칩 리셋 기본값은 0x7F 인데 그건 **외부 Vcc** 를 전제한 값이라,
 * 내부 charge pump(0x8D 0x14)로 구동하는 이 모듈에서는 눈에 띄게 흐리게 나온다.
 * 0xCF 가 내부 charge pump 구동 시의 표준 권장값이다.
 * 더 밝게 하려면 0xFF 까지 올릴 수 있지만 OLED 는 번인이 있으므로 상시 표시에는 권하지 않는다. */
#define CFG_OLED_CONTRAST               0xCFU
/* 컨트롤러 컬럼 오프셋 : SSD1306 = 0, SH1106 = 2.
 * 화면이 좌우로 2픽셀 밀려 보이면 SH1106 이므로 2 로 바꾼다. */
#define CFG_OLED_COL_OFFSET             0U

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

/* EVSE / Gateway 와 합의한 버스 속도. 이 매크로는 "규격" 일 뿐 하드웨어를 바꾸지 않는다
 * (실제 값은 APB1 x .ioc 의 Prescaler/BS1/BS2 로 결정). bms_can_init() 이 부팅 때
 * 실측 계산값과 비교해 어긋나면 에러를 띄운다.
 *   APB1 42MHz / Prescaler 6 / (1+11+2)TQ = 500kbps, 샘플포인트 85.7%
 * 샘플포인트까지 양쪽이 맞아야 한다 — 비트레이트만 맞추면 짧은 점퍼선에서는 되다가
 * 실배선 길이로 늘리는 순간 에러 프레임이 터진다. EVSE(F429ZI)도 APB1 42MHz 면 동일. */
#define CFG_CAN_BITRATE_BPS             500000L

/* --- CAN 동작 모드 (.ioc 가 아니라 여기가 단일 소스) ---
 * 1 = LOOPBACK : 트랜시버/상대 노드 없이 송수신 경로를 자기완결로 검증
 * 0 = NORMAL   : 실제 버스. 상대 노드가 ACK 를 줘야 송신이 성립한다.
 * bms_can_init() 이 이 값으로 hcan1.Init.Mode 를 덮어쓰고 재초기화하므로
 * CubeMX 로 .ioc 를 재생성해도 여기가 이긴다.
 *
 * !! NORMAL 에서 한쪽 보드만 켜면 ACK 를 줄 노드가 없어 Bus-Off 로 가는 것이 정상이다.
 *    ABOM=ENABLE 이라 상대를 연결하면 자동 복귀한다. */
#define CFG_CAN_MODE_LOOPBACK           0

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
 * 원격 명령이 배터리 보호를 무력화하지 못하도록 강제 클램프한다.
 * 노이즈로 200C 가 들어와도 60C 를 넘지 않고, 하한 30C 는 EVSE 의 OTA 시연값(35C)보다 낮다. */
#define CFG_OT_LIMIT_MIN_C10            300         /* 30.0C */
#define CFG_OT_LIMIT_MAX_C10            600         /* 60.0C */
#define CFG_OT_HYSTERESIS_C10           30          /* 해제 임계 = 진입 - 3.0C */

#define CFG_FW_VERSION_MAJOR            0U
#define CFG_FW_VERSION_MINOR            1U

#endif /* __BMS_CFG_H_ */
