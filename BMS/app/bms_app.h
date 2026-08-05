#ifndef __BMS_APP_H_
#define __BMS_APP_H_
#include "common_def.h"

/**
 * @brief  BMS 애플리케이션 진입점
 * @note   main.c 의 USER CODE 구역에 아래 2줄만 추가하면 된다.
 *         (USER CODE 블록 안이므로 CubeMX 재생성에도 살아남는다)
 *
 *          // USER CODE BEGIN Includes
 *          #include "bms_app.h"
 *          // USER CODE END Includes
 *
 *          // USER CODE BEGIN 2
 *          bms_app_init();
 *          // USER CODE END 2
 *
 *          while (1) {
 *              // USER CODE BEGIN 3
 *              bms_app_main();
 *          }
 */
void bms_app_init(void);
void bms_app_main(void);

const bms_data_t *bms_app_get_data(void);
#endif
