#ifndef __BMS_APP_H_
#define __BMS_APP_H_
#include "common_def.h"

/**
 * @brief  BMS 애플리케이션 진입점
 * @note   main.c 의 USER CODE 블록 안에서만 부른다 (CubeMX 재생성에도 살아남는다):
 *         Includes 에 #include "bms_app.h", BEGIN 2 에 bms_app_init(),
 *         while(1) 안(BEGIN 3)에 bms_app_main().
 */
void bms_app_init(void);
void bms_app_main(void);

const bms_data_t *bms_app_get_data(void);
#endif
