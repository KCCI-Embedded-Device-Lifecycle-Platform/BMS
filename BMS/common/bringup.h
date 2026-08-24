/**
 * @file    bringup.h
 * @brief   단계별 브링업 스위치
 * @note    부품이 도착하는 순서대로 0 -> 1 로 켜면서 하나씩 검증한다.
 *          한 단계가 확실히 동작한 뒤에 다음 단계를 켤 것.
 *          (한꺼번에 켜면 어느 모듈이 원인인지 절대 못 찾는다)
 */
#ifndef __BRINGUP_H_
#define __BRINGUP_H_

#define BRINGUP_S1_LED_DBG      1   /* LED 블링크 + UART 로그        (부품 불필요) */
#define BRINGUP_S1B_ADC_RAW     1   /* ADC1+DMA 기동만 ('v' 덤프/VDDA 검증)  (부품 불필요) */
#define BRINGUP_S2_CELL_ADC     1   /* 셀 4채널 ADC + 분압 복원      (저항, 시뮬레이터) */
/* 이 스위치는 "팩 전류 센서를 쓸 것인가" 만 결정한다.
 * 칩은 INA226 하나로 고정이다 (2026-08-14 모듈 입고, INA219 경로 제거).
 * 다른 칩으로 갈아탈 일이 생기면 고칠 곳은 dev/pack_sensor.h 다. */
#define BRINGUP_S3_PACK_SENSOR  1  /* 팩 전류/EVSE측 전압 I2C       (INA226) */
#define BRINGUP_S4_NTC_ACS      1   /* 온도 + 전류 크로스체크        (NTC, ACS712) */
#define BRINGUP_S5_OLED         1   /* 로컬 표시                     (SSD1306) */
#define BRINGUP_S6_LINK         1   /* 상태 프레임 송신 (UART->CAN)  */
#define BRINGUP_S7_FSM_FAULT    1   /* FSM + Fault 통합              */
/* 릴레이의 "실제 출력". 0 이면 논리 지령만 계산하고 PB5 는 계속 열어 둔다
 * (로그/OLED 에는 지령이 그대로 나오므`로 릴레이 없이 판단 로직만 먼저 검증할 수 있다).
 * 되돌릴 수 없는 물리 동작이라 1 로 올리기 전에 확인할 것:
 *   (1) CFG_RELAY_ACTIVE_HIGH 가 실물과 맞는가 (틀리면 부팅 즉시 접점이 붙는다)
 *   (2) 보드만 켠 상태에서 릴레이가 떨어져 있는가 */
#define BRINGUP_S8_RELAY        1   /* 충전 차단 릴레이 출력         (릴레이 모듈) */

/* S1B 는 S2/S4 의 하위 집합이다. 셀 ADC 나 NTC 를 켜면 자동으로 포함된다.
 * 분압 저항 없이 ADC 계층(DMA/오버샘플링/Vrefint) 만 먼저 검증할 때 쓴다. */
#define BRINGUP_ADC_HW          ((BRINGUP_S1B_ADC_RAW == 1) || \
                                 (BRINGUP_S2_CELL_ADC == 1) || \
                                 (BRINGUP_S4_NTC_ACS  == 1))

/* 디버그 로그 레벨 : 0=OFF 1=ERR 2=WARN 3=INFO 4=DEBUG */
#define BRINGUP_DBG_LEVEL       4

#endif /* __BRINGUP_H_ */
