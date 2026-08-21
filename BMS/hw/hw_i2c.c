/**
 * @file    hw_i2c.c
 * @brief   I2C1 / I2C3 (400kHz) 래퍼 — 버스 2개를 매핑 테이블로 다룬다
 *
 * @note    HAL 은 7bit 주소를 1비트 시프트한 값을 요구한다 (addr7 << 1).
 *
 * @note    **버스가 2개인 이유와 왜 I2C2 가 아니라 I2C3 인지는 hw_i2c.h 의
 *          hw_i2c_bus_t 주석에 있다.** 한 줄 요약: 모듈 온보드 풀업이 병렬로 붙는 것을
 *          피하려고 나눴고, LQFP64 에서 두 번째 I2C 로 가능한 것이 I2C3 뿐이었다.
 *          -> 이제 **각 모듈의 풀업을 떼지 말 것.** 버스마다 한 쌍이 정상이다.
 *
 * @note    핀/핸들은 s_bus[] 한 곳에만 있다. hw_gpio.c 의 s_map[] 과 같은 패턴이라
 *          핀이 바뀌면 그 테이블 한 줄만 고치면 되고, 진단 로그(라인 레벨, 9클럭 복구)도
 *          자동으로 맞는 핀을 가리킨다 — 예전처럼 PB6/PB7 이 로그 문자열에 하드코딩되어
 *          "로그는 PB6 이라는데 실제로는 PA8 을 보고 있는" 상황이 생기지 않는다.
 */
#include "hw_i2c.h"
#include "dbg.h"

extern I2C_HandleTypeDef hi2c1;         /* CubeMX 생성 (i2c.c) */
extern I2C_HandleTypeDef hi2c3;

typedef struct {
    I2C_HandleTypeDef *p_h;
    GPIO_TypeDef      *scl_port;
    uint16_t           scl_pin;
    GPIO_TypeDef      *sda_port;        /* I2C3 은 SCL/SDA 가 다른 포트다 (PA8 / PC9) */
    uint16_t           sda_pin;
    const char        *name;            /* 로그용 */
} hw_i2c_map_t;

static const hw_i2c_map_t s_bus[HW_I2C_MAX] = {
    /* HW_I2C_SENSOR */ { &hi2c1, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, "I2C1 PB6/PB7" },
    /* HW_I2C_OLED   */ { &hi2c3, GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, "I2C3 PA8/PC9" },
};

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

#define BUS_OK(b)           ((b) < HW_I2C_MAX)

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
 *
 *         버스를 나눈 뒤로는 **한쪽만 Low 인 것이 원인을 크게 좁혀 준다.**
 *         공유 버스일 때는 어느 모듈이 물었는지 몰라 하나씩 떼야 했지만,
 *         이제 죽은 버스에 붙은 모듈이 곧 범인이다.
 */
static bool i2c_report_lines(hw_i2c_bus_t bus, bool after_recover)
{
    const hw_i2c_map_t *p_b = &s_bus[bus];
    bool scl = (HAL_GPIO_ReadPin(p_b->scl_port, p_b->scl_pin) == GPIO_PIN_SET);
    bool sda = (HAL_GPIO_ReadPin(p_b->sda_port, p_b->sda_pin) == GPIO_PIN_SET);

    if (scl && sda) {
        DBG_I("%s idle: SCL=1 SDA=1 (정상 - 풀업 살아 있음)", p_b->name);
        return true;
    }

    DBG_E("%s idle: SCL=%d SDA=%d  <- 정상은 1/1 이다", p_b->name, (int)scl, (int)sda);

    if (!scl && !sda) {
        DBG_E("  둘 다 Low = 풀업이 없거나 모듈 VCC(3.3V)가 안 들어왔다.");
        DBG_E("  .ioc 가 핀을 NOPULL 로 잡으므로 **외부 풀업이 필수**다 (4.7k x2).");
        DBG_E("  버스를 나눈 뒤로는 모듈 온보드 풀업을 떼지 않는다 - 뗐으면 되돌릴 것.");
    } else if (scl && !sda) {
        /* 9클럭 복구 뒤에도 Low 로 남는지가 "일시적 lock-up" 과 "전기적 고장" 을 가른다.
         * 진짜 lock-up 은 SCL 9번이면 거의 항상 풀린다 — 안 풀리면 원인이 다른 데 있다. */
        if (after_recover) {
            DBG_E("  9클럭 복구 뒤에도 SDA 가 Low = lock-up 이 **아니다**.");
            DBG_E("  전기적 문제다: SDA 가 GND 에 단락됐거나 슬레이브 드라이버가 손상됐다.");
            DBG_E("  이 버스에는 모듈이 하나뿐이므로 그 모듈이 범인이다 (팩 B+ 먼저 분리).");
        } else {
            DBG_E("  SDA 만 Low = 슬레이브가 버스를 물고 있다 (lock-up). 'b' 로 복구 시도.");
        }
    } else {
        DBG_E("  SCL 만 Low = SCL 이 GND 에 붙었거나 배선 단락. 배선을 먼저 볼 것.");
    }
    return false;
}

/**
 * @brief  버스 한 개 초기화
 * @note   CubeMX 가 MX_I2Cx_Init() 에서 이미 초기화했지만, 속도는 bms_cfg.h 를 단일 소스로
 *         삼는다 (hw_adc_init() 의 ADC_CR2_DDS 강제, bms_can_init() 의 모드 덮어쓰기와 같은 패턴).
 *         .ioc 를 재생성해도 여기가 이긴다.
 */
static void i2c_init_one(hw_i2c_bus_t bus)
{
    const hw_i2c_map_t *p_b = &s_bus[bus];

    if (p_b->p_h->Init.ClockSpeed != (uint32_t)CFG_I2C_SPEED_HZ) {
        p_b->p_h->Init.ClockSpeed = (uint32_t)CFG_I2C_SPEED_HZ;
        if (HAL_I2C_Init(p_b->p_h) != HAL_OK) {
            DBG_E("%s re-init failed (%ld Hz)", p_b->name, (long)CFG_I2C_SPEED_HZ);
            return;
        }
    }

    if (HAL_I2C_GetState(p_b->p_h) != HAL_I2C_STATE_READY) {
        DBG_E("%s not ready", p_b->name);
        return;
    }

    /* 속도를 부팅 로그에 남긴다 — "100k 로 내려서 됐다" 를 나중에 로그만 보고 알 수 있어야
     * 400k 로 되돌리는 것을 잊지 않는다. */
    DBG_I("%s %ldkHz (외부 풀업 필수)", p_b->name, (long)(CFG_I2C_SPEED_HZ / 1000L));

    /* 첫 트랜잭션 전에 라인 상태를 찍는다. 이게 없으면 부팅 로그가
     * "no ACK" 만 보여 주는데, 그건 배선/풀업/주소/전원 넷을 하나도 못 가른다.
     * 부팅 시점에는 아직 복구를 돌리지 않았으므로 after_recover = false. */
    (void)i2c_report_lines(bus, false);
}

void hw_i2c_init(void)
{
    uint8_t i;
    for (i = 0; i < (uint8_t)HW_I2C_MAX; i++) {
        i2c_init_one((hw_i2c_bus_t)i);
    }
}

bool hw_i2c_is_ready(hw_i2c_bus_t bus, uint8_t addr7)
{
    if (!BUS_OK(bus)) {
        return false;
    }
    return (HAL_I2C_IsDeviceReady(s_bus[bus].p_h, (uint16_t)(addr7 << 1),
                                  3, I2C_TIMEOUT_MS) == HAL_OK);
}

bool hw_i2c_write(hw_i2c_bus_t bus, uint8_t addr7, const uint8_t *p_data, uint16_t len)
{
    if (!BUS_OK(bus)) {
        return false;
    }
    return (HAL_I2C_Master_Transmit(s_bus[bus].p_h, (uint16_t)(addr7 << 1),
                                    (uint8_t *)p_data, len, I2C_TIMEOUT_MS) == HAL_OK);
}

bool hw_i2c_reg_write16(hw_i2c_bus_t bus, uint8_t addr7, uint8_t reg, uint16_t val)
{
    uint8_t tx[3];

    tx[0] = reg;
    tx[1] = (uint8_t)(val >> 8);        /* INA2xx 레지스터는 MSB first */
    tx[2] = (uint8_t)(val & 0xFFU);

    return hw_i2c_write(bus, addr7, tx, 3);
}

bool hw_i2c_reg_read16(hw_i2c_bus_t bus, uint8_t addr7, uint8_t reg, uint16_t *p_val)
{
    uint8_t rx[2];

    if (!BUS_OK(bus)) {
        return false;
    }

    /* 포인터 레지스터 write -> repeated start -> 2byte read */
    if (HAL_I2C_Mem_Read(s_bus[bus].p_h, (uint16_t)(addr7 << 1), reg,
                         I2C_MEMADD_SIZE_8BIT, rx, 2, I2C_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    *p_val = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return true;
}

/**
 * @brief  버스 스캔 (콘솔 'b')
 * @retval 응답한 슬레이브 개수
 * @note   "장치가 안 잡힌다" 를 만났을 때 원인을 세 갈래로 가르는 도구다:
 *           0개        -> 버스 자체가 죽었다 (풀업 없음 / SCL-SDA 뒤바뀜 / 모듈 전원 없음)
 *           일부만     -> 안 잡힌 모듈 하나의 배선·전원 문제
 *           낯선 주소  -> 점퍼 설정이 다르다 (OLED 0x3D, INA226 A0/A1 조합 0x41~0x4F)
 *         주소를 모른 채로는 "0x40 에 no ACK" 로그가 "모듈이 없다" 인지
 *         "다른 주소에 있다" 인지 구분해 주지 못한다.
 *
 *         !! 버스 하나당 최대 약 0.5초 블로킹이다 (112주소 x 1회 x 4ms).
 *         두 버스를 다 훑으면 1초다 — 주기 슬롯이 그만큼 밀려 EVSE 가 BMS 를 잠깐
 *         offline 으로 볼 수 있다. 수동 디버그 전용으로만 쓸 것.
 *         그래서 재시도 1회 / 짧은 타임아웃으로 hw_i2c_is_ready() 와 따로 둔다.
 */
uint8_t hw_i2c_scan(hw_i2c_bus_t bus)
{
    uint8_t addr;
    uint8_t found = 0;

    if (!BUS_OK(bus)) {
        return 0U;
    }

    /* 라인이 Low 로 붙어 있으면 스캔은 무조건 0개다. 그 상태에서 0개를 보고하면
     * "장치가 없다" 로 읽혀서 엉뚱한 곳을 뒤지게 된다 — 먼저 전기적 상태를 알린다. */
    /* 'b' 핸들러가 이 함수 직전에 hw_i2c_recover() 를 돌린다 -> after_recover = true.
     * 그래야 "복구했는데도 Low" 를 lock-up 이 아니라 전기적 고장으로 보고할 수 있다. */
    if (!i2c_report_lines(bus, true)) {
        DBG_E("%s scan 생략 - 라인이 Low 다. 스캔해도 전부 0개로만 나온다.", s_bus[bus].name);
        return 0U;
    }

    /* 0x00~0x07 과 0x78~0x7F 는 예약 주소라 건너뛴다 (스캔하면 오검출이 난다) */
    for (addr = 0x08U; addr <= 0x77U; addr++) {
        if (HAL_I2C_IsDeviceReady(s_bus[bus].p_h, (uint16_t)(addr << 1), 1, 4U) == HAL_OK) {
            DBG_I("  %s found 0x%02X%s", s_bus[bus].name, addr,
                  (addr == CFG_INA226_I2C_ADDR) ? "  <- INA226" :
                  (addr == CFG_OLED_I2C_ADDR)   ? "  <- OLED"   : "");
            found++;
        }
    }

    if (found == 0U) {
        /* 라인은 High 인데 아무도 ACK 하지 않는 경우다 = 풀업은 살아 있다.
         * 그러면 남는 원인은 "장치 쪽" 셋으로 좁혀진다. */
        DBG_E("%s scan: 라인은 정상인데 응답 0개", s_bus[bus].name);
        DBG_E("  -> 풀업은 살아 있으므로 원인은 장치 쪽이다:");
        DBG_E("     (1) SCL/SDA 뒤바뀜 (버스 이름의 핀 순서가 SCL/SDA 다)");
        DBG_E("     (2) 모듈 VCC/GND 미결선 (풀업만 다른 데서 오고 있을 수 있다)");
        DBG_E("     (3) 모듈 GND 와 보드 GND 미공유");
    } else {
        DBG_I("%s scan: %u device(s)", s_bus[bus].name, found);
    }
    return found;
}

/**
 * @brief  I2C 버스 복구
 * @note   슬레이브가 ACK 도중 리셋되면 SDA 를 Low 로 물고 있어 버스가 잠긴다.
 *         SCL 을 9번 토글해 슬레이브 시프트 레지스터를 비우는 표준 복구 시퀀스다.
 *
 *         복구 끝의 HAL_I2C_Init() 이 MspInit 을 다시 부르므로 SCL 핀이 AF 로 되돌아온다 —
 *         여기서 수동 출력으로 바꾼 것을 직접 복원할 필요가 없다.
 */
bool hw_i2c_recover(hw_i2c_bus_t bus)
{
    const hw_i2c_map_t *p_b;
    GPIO_InitTypeDef    gi = {0};
    uint8_t             i;

    if (!BUS_OK(bus)) {
        return false;
    }
    p_b = &s_bus[bus];

    __HAL_I2C_DISABLE(p_b->p_h);

    gi.Pin   = p_b->scl_pin;            /* SCL 을 수동 출력으로 */
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(p_b->scl_port, &gi);

    for (i = 0; i < 9U; i++) {
        HAL_GPIO_WritePin(p_b->scl_port, p_b->scl_pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(p_b->scl_port, p_b->scl_pin, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    /* 페리페럴 소프트 리셋 후 재초기화 */
    __HAL_I2C_ENABLE(p_b->p_h);
    HAL_I2C_DeInit(p_b->p_h);
    if (HAL_I2C_Init(p_b->p_h) != HAL_OK) {
        DBG_E("%s recover failed", p_b->name);
        return false;
    }
    DBG_W("%s bus recovered", p_b->name);
    return true;
}
