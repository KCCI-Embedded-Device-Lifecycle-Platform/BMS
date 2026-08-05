/**
 * @file    hw_spi.c
 * @brief   SPI2 (PB13 SCK / PB15 MOSI, Full-Duplex Master) 래퍼
 *
 * @details F446 로 옮기면서 SPI2 를 .ioc 에 정식으로 넣었다.
 *      F411 시절에는 이 파일이 클럭/GPIO/핸들을 전부 직접 잡았지만,
 *      이제 MX_SPI2_Init() 이 먼저 hspi2 를 초기화한다. 그 상태에서 여기서
 *      또 별도 핸들로 HAL_SPI_Init() 을 하면 한 페리페럴에 핸들이 둘이 되어
 *      상태 추적(HAL_SPI_STATE_*)이 어긋난다.
 *      -> ADC/USART 와 똑같이 CubeMX 핸들을 그대로 쓴다.
 *
 * @note  다만 전송 속도만은 bms_cfg.h 가 계속 단일 소스다.
 *        .ioc 의 프리스케일러와 CFG_OLED_SPI_CLK_DIV 가 어긋나면
 *        여기서 CFG 값으로 덮어쓰고 재초기화한다.
 *
 * @note  SPI1 을 쓰지 않는 이유 : SCK 가 PA5 라 온보드 LD2 와 겹친다.
 *        MISO(PB14)는 OLED 가 읽기를 지원하지 않으므로 연결하지 않는다.
 */
#include "hw_spi.h"
#include "dbg.h"

extern SPI_HandleTypeDef hspi2;         /* CubeMX 생성 (spi.c) */

#define SPI_TIMEOUT_MS      100U

/* CFG_OLED_SPI_CLK_DIV -> HAL 프리스케일러 상수 매핑
 * (bms_cfg.h 를 HAL 심볼로부터 독립시키기 위해 여기서 변환한다) */
#if   (CFG_OLED_SPI_CLK_DIV == 2U)
  #define SPI_PRESCALER     SPI_BAUDRATEPRESCALER_2
#elif (CFG_OLED_SPI_CLK_DIV == 4U)
  #define SPI_PRESCALER     SPI_BAUDRATEPRESCALER_4
#elif (CFG_OLED_SPI_CLK_DIV == 8U)
  #define SPI_PRESCALER     SPI_BAUDRATEPRESCALER_8
#elif (CFG_OLED_SPI_CLK_DIV == 16U)
  #define SPI_PRESCALER     SPI_BAUDRATEPRESCALER_16
#elif (CFG_OLED_SPI_CLK_DIV == 32U)
  #define SPI_PRESCALER     SPI_BAUDRATEPRESCALER_32
#else
  #error "CFG_OLED_SPI_CLK_DIV must be 2/4/8/16/32"
#endif

bool hw_spi_init(void)
{
    /* 클럭 / PB13 SCK / PB15 MOSI(AF5) 는 HAL_SPI_MspInit() 이 이미 끝냈다.
     * SSD1306 은 Mode 0 (CPOL=0, CPHA=1Edge) 이고 .ioc 도 같은 값이라
     * 여기서는 프리스케일러만 확인한다. */
    if (hspi2.Instance != SPI2) {
        DBG_E("SPI2 handle not initialized (MX_SPI2_Init missing?)");
        return false;
    }

    if (hspi2.Init.BaudRatePrescaler != SPI_PRESCALER) {
        /* .ioc 값과 bms_cfg.h 값이 다르면 bms_cfg.h 를 따른다 */
        hspi2.Init.BaudRatePrescaler = SPI_PRESCALER;
        if (HAL_SPI_Init(&hspi2) != HAL_OK) {
            DBG_E("SPI2 re-init failed");
            return false;
        }
        DBG_W("SPI2 prescaler overridden by bms_cfg.h (/%u)", CFG_OLED_SPI_CLK_DIV);
    }

    DBG_I("SPI2 ok (PB13 SCK / PB15 MOSI, APB1/%u)", CFG_OLED_SPI_CLK_DIV);
    return true;
}

bool hw_spi_write(const uint8_t *p_data, uint16_t len)
{
    /* 송신 전용이라 Transmit 를 쓴다 (RX 버퍼 불필요).
     * 프레임버퍼 1페이지 = 128B, 5.25MHz 기준 약 195us -> 블로킹으로 충분. */
    return (HAL_SPI_Transmit(&hspi2, (uint8_t *)p_data, len, SPI_TIMEOUT_MS) == HAL_OK);
}
