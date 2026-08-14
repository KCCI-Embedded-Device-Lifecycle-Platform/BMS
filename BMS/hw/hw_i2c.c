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
 * 정상 최장 트랜잭션은 OLED 한 페이지((128+1)byte x 9bit)이므로 그 3배를 준다.
 *   400kHz -> 페이지 2.9ms, 타임아웃 10ms
 *   100kHz -> 페이지 11.6ms, 타임아웃 36ms
 *
 * !! 상수로 못 박으면 안 된다. CFG_I2C_SPEED_HZ 를 100k 로 내려 진단할 때
 *    한 페이지가 11.6ms 로 늘어 10ms 고정 타임아웃에 그냥 걸린다. 그러면
 *    "속도를 내렸더니 OLED 가 죽었다" 로 보여서 진단이 정반대로 간다.
 *    그래서 속도에서 유도한다.
 *
 * 동시에 super-loop 의 최악 블로킹을 결정한다 — 100k 로 내리면 36ms 가 되므로
 * 진단이 끝나면 400k 로 되돌릴 것. (50ms 로 두면 버스가 물렸을 때 한 바퀴가
 * 300ms 멈추고 0x100 송신 간격이 EVSE 타임아웃 500~600ms 를 위협한다) */
#define I2C_TIMEOUT_MS      ((uint32_t)((((CFG_OLED_WIDTH + 1UL) * 9UL * 3UL * 1000UL) \
                                         / (uint32_t)CFG_I2C_SPEED_HZ) + 2UL))

/**
 * @brief  SCL/SDA 유휴 레벨 보고
 * @retval 둘 다 High(정상) 면 true
 * @note   "아무 주소도 ACK 하지 않는다" 는 증상은 원인이 최소 네 갈래인데,
 *         스캔 결과(장치 0개)만으로는 하나도 갈라지지 않는다. 라인 레벨은 그걸 가른다.
 *
 *         I2C 는 오픈드레인이라 **아무도 안 잡으면 풀업이 High 로 올려야 한다.**
 *         Low 로 남아 있다는 건 전송 이전에 이미 전기적으로 틀렸다는 뜻이다.
 *
 *         AF 모드에서도 IDR 은 핀 실제 레벨을 그대로 읽으므로(아날로그 모드만 예외)
 *         재설정 없이 그냥 읽으면 된다 — 버스를 건드리지 않는 무해한 관측이다.
 */
static bool i2c_report_lines(bool after_recover)
{
    bool scl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET);
    bool sda = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET);

    if (scl && sda) {
        DBG_I("I2C idle: SCL=1 SDA=1 (정상 - 풀업 살아 있음)");
        return true;
    }

    DBG_E("I2C idle: SCL(PB6)=%d SDA(PB7)=%d  <- 정상은 1/1 이다", (int)scl, (int)sda);

    if (!scl && !sda) {
        DBG_E("  둘 다 Low = 풀업이 없거나 모듈 VCC(3.3V)가 안 들어왔다.");
        DBG_E("  .ioc 가 PB6/PB7 을 NOPULL 로 잡으므로 **외부 풀업이 필수**다 (4.7k x2).");
    } else if (scl && !sda) {
        /* 9클럭 복구 뒤에도 Low 로 남는지가 "일시적 lock-up" 과 "전기적 고장" 을 가른다.
         * 진짜 lock-up 은 SCL 9번이면 거의 항상 풀린다 — 안 풀리면 원인이 다른 데 있다. */
        if (after_recover) {
            DBG_E("  9클럭 복구 뒤에도 SDA 가 Low = lock-up 이 **아니다**.");
            DBG_E("  전기적 문제다: SDA 가 GND 에 단락됐거나 슬레이브 드라이버가 손상됐다.");
            DBG_E("  절단 순서: (1) 팩(B+) 분리  (2) 모듈을 하나씩 떼면서");
            DBG_E("            SDA 가 1 로 돌아오는 지점을 찾을 것 - 그 모듈이 범인이다.");
        } else {
            DBG_E("  SDA 만 Low = 슬레이브가 버스를 물고 있다 (lock-up). 'b' 로 복구 시도.");
        }
    } else {
        DBG_E("  SCL 만 Low = SCL 이 GND 에 붙었거나 배선 단락. 배선을 먼저 볼 것.");
    }
    return false;
}

void hw_i2c_init(void)
{
    /* CubeMX 가 MX_I2C1_Init() 에서 이미 초기화했지만, 속도는 bms_cfg.h 를 단일 소스로
     * 삼는다 (hw_adc_init() 의 ADC_CR2_DDS 강제, bms_can_init() 의 모드 덮어쓰기와 같은 패턴).
     * .ioc 를 재생성해도 여기가 이긴다. */
    if (hi2c1.Init.ClockSpeed != (uint32_t)CFG_I2C_SPEED_HZ) {
        hi2c1.Init.ClockSpeed = (uint32_t)CFG_I2C_SPEED_HZ;
        if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
            DBG_E("I2C1 re-init failed (%ld Hz)", (long)CFG_I2C_SPEED_HZ);
            return;
        }
    }

    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        DBG_E("I2C1 not ready");
        return;
    }

    /* 속도를 부팅 로그에 남긴다 — "100k 로 내려서 됐다" 를 나중에 로그만 보고 알 수 있어야
     * 400k 로 되돌리는 것을 잊지 않는다. */
    DBG_I("I2C1 %ldkHz (PB6 SCL / PB7 SDA, 외부 풀업 필수)",
          (long)(CFG_I2C_SPEED_HZ / 1000L));

    /* 첫 트랜잭션 전에 라인 상태를 찍는다. 이게 없으면 부팅 로그가
     * "no ACK" 만 보여 주는데, 그건 배선/풀업/주소/전원 넷을 하나도 못 가른다.
     * 부팅 시점에는 아직 복구를 돌리지 않았으므로 after_recover = false. */
    (void)i2c_report_lines(false);
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
 * @brief  I2C1 버스 스캔 (콘솔 'b')
 * @retval 응답한 슬레이브 개수
 * @note   "OLED 도 안 되고 팩 센서도 안 된다" 를 만났을 때 원인을 세 갈래로 가르는 도구다:
 *           0개        -> 버스 자체가 죽었다 (풀업 없음 / SCL-SDA 뒤바뀜 / 모듈 전원 없음)
 *           일부만     -> 안 잡힌 모듈 하나의 배선·전원 문제
 *           낯선 주소  -> 점퍼 설정이 다르다 (OLED 0x3D, INA226 A0/A1 조합 0x41~0x4F)
 *         주소를 모른 채로는 "0x40 에 no ACK" 로그가 "모듈이 없다" 인지
 *         "다른 주소에 있다" 인지 구분해 주지 못한다.
 *
 *         !! 최대 약 0.5초 블로킹이다 (112주소 x 1회 x 4ms). 주기 슬롯이 그만큼 밀려
 *         EVSE 가 BMS 를 잠깐 offline 으로 볼 수 있다 — 수동 디버그 전용으로만 쓸 것.
 *         그래서 재시도 1회 / 짧은 타임아웃으로 hw_i2c_is_ready() 와 따로 둔다.
 */
uint8_t hw_i2c_scan(void)
{
    uint8_t addr;
    uint8_t found = 0;

    /* 라인이 Low 로 붙어 있으면 스캔은 무조건 0개다. 그 상태에서 0개를 보고하면
     * "장치가 없다" 로 읽혀서 엉뚱한 곳을 뒤지게 된다 — 먼저 전기적 상태를 알린다. */
    /* 'b' 핸들러가 이 함수 직전에 hw_i2c_recover() 를 돌린다 -> after_recover = true.
     * 그래야 "복구했는데도 Low" 를 lock-up 이 아니라 전기적 고장으로 보고할 수 있다. */
    if (!i2c_report_lines(true)) {
        DBG_E("I2C scan 생략 - 라인이 Low 다. 스캔해도 전부 0개로만 나온다.");
        return 0U;
    }

    /* 0x00~0x07 과 0x78~0x7F 는 예약 주소라 건너뛴다 (스캔하면 오검출이 난다) */
    for (addr = 0x08U; addr <= 0x77U; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 4U) == HAL_OK) {
            DBG_I("  I2C found 0x%02X%s", addr,
                  (addr == CFG_INA226_I2C_ADDR) ? "  <- INA226" :
                  (addr == CFG_OLED_I2C_ADDR)   ? "  <- OLED"   : "");
            found++;
        }
    }

    if (found == 0U) {
        /* 라인은 High 인데 아무도 ACK 하지 않는 경우다 = 풀업은 살아 있다.
         * 그러면 남는 원인은 "장치 쪽" 셋으로 좁혀진다. */
        DBG_E("I2C scan: 라인은 정상인데 응답 0개");
        DBG_E("  -> 풀업은 살아 있으므로 원인은 장치 쪽이다:");
        DBG_E("     (1) SCL/SDA 뒤바뀜 (PB6=SCL, PB7=SDA)");
        DBG_E("     (2) 모듈 VCC/GND 미결선 (풀업만 다른 데서 오고 있을 수 있다)");
        DBG_E("     (3) 모듈 GND 와 보드 GND 미공유");
    } else {
        DBG_I("I2C scan: %u device(s)", found);
    }
    return found;
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
