/**
 * @file    hw_i2c.c
 * @brief   I2C1 (PB6 SCL / PB7 SDA, 400kHz) 래퍼
 * @note    HAL 은 7bit 주소를 1비트 시프트한 값을 요구한다 (addr7 << 1).
 *          팩 센서(0x40)와 SSD1306(0x3C)이 같은 버스를 공유하므로
 *          풀업 4.7k 는 버스 전체에 한 쌍만 남길 것.
 */
#include "hw_i2c.h"
#include "dbg.h"

extern I2C_HandleTypeDef hi2c1;         /* CubeMX 생성 (i2c.c) */

/* 이 값은 "정상 전송에 필요한 시간" 이 아니라 "죽은 버스를 얼마나 붙들고 있을지" 다.
 * 정상 최장 트랜잭션은 OLED 한 페이지 약 2.9ms 라 10ms 면 3배 여유다.
 * 동시에 super-loop 의 최악 블로킹을 결정한다 — 50ms 로 두면 버스가 물렸을 때
 * 한 바퀴가 300ms 멈추고 0x100 송신 간격이 EVSE 타임아웃(500~600ms)을 위협한다. */
#define I2C_TIMEOUT_MS      10U

void hw_i2c_init(void)
{
    /* CubeMX 가 MX_I2C1_Init() 에서 이미 초기화. 여기서는 상태만 확인. */
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        DBG_E("I2C1 not ready");
    }
}

bool hw_i2c_is_ready(uint8_t addr7)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1), 3, I2C_TIMEOUT_MS) == HAL_OK);
}

bool hw_i2c_write(uint8_t addr7, const uint8_t *p_data, uint16_t len)
{
    return (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1),
                                    (uint8_t *)p_data, len, I2C_TIMEOUT_MS) == HAL_OK);
}

bool hw_i2c_reg_write16(uint8_t addr7, uint8_t reg, uint16_t val)
{
    uint8_t tx[3];

    tx[0] = reg;
    tx[1] = (uint8_t)(val >> 8);        /* INA2xx 레지스터는 MSB first */
    tx[2] = (uint8_t)(val & 0xFFU);

    return hw_i2c_write(addr7, tx, 3);
}

bool hw_i2c_reg_read16(uint8_t addr7, uint8_t reg, uint16_t *p_val)
{
    uint8_t rx[2];

    /* 포인터 레지스터 write -> repeated start -> 2byte read */
    if (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, rx, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *p_val = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return true;
}

/**
 * @brief  I2C 버스 복구
 * @note   슬레이브가 ACK 도중 리셋되면 SDA 를 Low 로 물고 있어 버스가 잠긴다.
 *         SCL 을 9번 토글해 슬레이브 시프트 레지스터를 비우는 표준 복구 시퀀스다.
 */
bool hw_i2c_recover(void)
{
    GPIO_InitTypeDef gi = {0};
    uint8_t          i;

    __HAL_I2C_DISABLE(&hi2c1);

    gi.Pin   = GPIO_PIN_6;              /* SCL 을 수동 출력으로 */
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gi);

    for (i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* 페리페럴 소프트 리셋 후 재초기화 */
    __HAL_I2C_ENABLE(&hi2c1);
    HAL_I2C_DeInit(&hi2c1);
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        DBG_E("I2C recover failed");
        return false;
    }
    DBG_W("I2C bus recovered");
    return true;
}
