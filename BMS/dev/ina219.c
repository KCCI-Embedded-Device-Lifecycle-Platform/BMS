/**
 * @file    ina219.c
 * @brief   INA219 팩 전압/전류 측정 드라이버 (I2C)
 *
 * 최종 부품은 INA226 이지만 모듈 미입고라 현재 선택된 것은 이쪽이다(CFG_PACK_SENSOR).
 * 레지스터 번호만 같고 스케일이 전부 다르므로 ina226.c 와 파일을 나눠 둔다 —
 * 한 파일에 #if 로 섞으면 포팅 함정이 숨는다.
 *
 *    항목            INA226            INA219 (현재 사용)
 *    션트 풀스케일   +-81.92mV 고정    +-40/80/160/320mV (PGA 선택)
 *    션트 LSB        2.5uV             10uV
 *    버스 최대/LSB   36V / 1.25mV      26V / 4mV  (상위 13bit -> >>3 필요)
 *    평균화          하드웨어 AVG      없음 (12bit 532us 단발)
 *
 * -> PGA /8(320mV)이면 0.1옴 션트로 +-3.2A 라, 모듈 기본 R100 을 교체하지 않아도
 *    과전류 임계(1A)가 범위 안에 든다 (INA226 은 0.82A 에서 포화해 0.01옴 교체가 필수였다).
 *
 * @note  전류는 CALIB 레지스터가 아니라 션트 전압에서 직접 구한다:
 *        I[mA] = V_shunt[uV] / R_shunt[mohm].
 *        CALIB 설정 실수가 개입하지 않고 디버깅 때 원값을 그대로 볼 수 있다.
 *        (POWER 레지스터를 쓰려면 CALIB 이 필요하므로 설정은 해 둔다)
 */
#include "ina219.h"
#include "hw_i2c.h"
#include "dbg.h"

/* --- CONFIG 레지스터 비트 구성 ---
 *  [15]    RST         = 0
 *  [13]    BRNG        = 1  : 버스 32V 레인지
 *  [12:11] PG          = 3  : 게인 /8, 션트 +-320mV
 *  [10:7]  BADC        = 3  : 12bit, 532us
 *  [6:3]   SADC        = 3  : 12bit, 532us
 *  [2:0]   MODE        = 7  : Shunt+Bus, Continuous
 */
#define INA219_CFG_VALUE    0x399FU
#define INA219_CFG_RESET    0x8000U

/* CALIB = trunc(0.04096 / (Current_LSB * R_shunt)). Current_LSB=100uA -> 409600/R[mohm].
 * 션트를 바꿔도 따라가도록 상수 대신 식으로 둔다 (0.01옴이면 40960, uint16 범위 안). */
#define INA219_CALIB_VALUE  ((uint16_t)(409600L / CFG_INA219_SHUNT_MOHM))

/* 측정 가능한 최대 전류[mA]. 320000 은 PGA /8(+-320mV) 전제다 —
 * CONFIG 의 PG 필드를 낮추면 이 값도 같이 고쳐야 한다. */
#define INA219_FS_MA        (320000L / CFG_INA219_SHUNT_MOHM)

/* 풀스케일이 과전류 임계보다 낮으면 과전류 Fault 는 원리적으로 발생할 수 없다.
 * 런타임에는 "전류가 안 올라가네" 로만 보여 찾기 어려우므로 빌드에서 잡는다. */
#if (CFG_PACK_SENSOR == CFG_PACK_SENSOR_INA219)
  #if (INA219_FS_MA <= CFG_OVER_CURRENT_MA)
    #warning "INA219 shunt saturates below CFG_OVER_CURRENT_MA -- \
over-current fault can never be measured. Use a smaller shunt (CFG_INA219_SHUNT_MOHM) \
instead of lowering CFG_OVER_CURRENT_MA."
  #endif
#endif

/* INA226 에만 있는 제조사 ID 레지스터. INA219 는 이 포인터를 받지 않으므로
 * 값이 읽힌다면 0x40 에 꽂힌 것이 INA226 이라는 뜻이다 (기본 주소가 같아서 필요한 검사). */
#define INA226_REG_MFG_ID   0xFEU
#define INA226_MFG_ID_TI    0x5449U

static int32_t s_bus_mv;
static int32_t s_shunt_uv;
static int32_t s_current_ma;
static bool    s_ok;
static bool    s_sat_warned;        /* 포화/오버플로 경고는 1회만 (100ms 주기 로그 폭주 방지) */

bool ina219_init(void)
{
    uint16_t val = 0;

    s_ok         = false;
    s_sat_warned = false;

    if (!hw_i2c_is_ready(CFG_INA219_I2C_ADDR)) {
        DBG_E("INA219 no ACK at 0x%02X (A1/A0 배선 확인)", CFG_INA219_I2C_ADDR);
        return false;
    }

    /* 1) 잘못된 칩 확인. 여기서 TI ID 가 나오면 INA226 을 꽂아 놓고 이 드라이버를 빌드한
     *    것이고, 그대로 두면 전압 8배·전류 4배로 조용히 오측정된다.
     *    바로 뒤 소프트 리셋이 포인터를 되돌리므로 여기서 탐색해도 안전하다. */
    if (hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA226_REG_MFG_ID, &val)) {
        if (val == INA226_MFG_ID_TI) {
            DBG_E("0x%02X 에 INA226 이 꽂혀 있다 - CFG_PACK_SENSOR 를 INA226 으로 바꿀 것",
                  CFG_INA219_I2C_ADDR);
            return false;
        }
    }

    /* 2) 소프트 리셋 : 이전 세션 설정이 남아있을 수 있다 */
    (void)hw_i2c_reg_write16(CFG_INA219_I2C_ADDR, INA219_REG_CONFIG, INA219_CFG_RESET);
    HAL_Delay(5);

    /* 3) 설정 write */
    if (!hw_i2c_reg_write16(CFG_INA219_I2C_ADDR, INA219_REG_CONFIG, INA219_CFG_VALUE)) {
        DBG_E("INA219 config write fail");
        return false;
    }
    if (!hw_i2c_reg_write16(CFG_INA219_I2C_ADDR, INA219_REG_CALIB, INA219_CALIB_VALUE)) {
        DBG_E("INA219 calib write fail");
        return false;
    }

    /* 4) 읽어서 확인 (write-verify : 드라이버 초기화의 기본) */
    if (!hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA219_REG_CONFIG, &val)) {
        return false;
    }
    if (val != INA219_CFG_VALUE) {
        DBG_E("INA219 config mismatch: w=0x%04X r=0x%04X", INA219_CFG_VALUE, val);
        return false;
    }

    s_ok = true;
    DBG_I("INA219 ok (cfg=0x%04X, shunt=%ld mohm, FS=%ld mA)",
          val, (long)CFG_INA219_SHUNT_MOHM, (long)INA219_FS_MA);
    return true;
}

bool ina219_update(void)
{
    uint16_t raw;
    int16_t  sraw;

    if (!s_ok) {
        return false;
    }

    /* --- Bus Voltage --- */
    if (!hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA219_REG_BUS_V, &raw)) {
        s_ok = false;
        DBG_E("INA219 bus read fail");
        return false;
    }
    /* bit0=OVF, bit1=CNVR, bit[15:3]=전압, LSB=4mV.
     * INA226 과 달리 >>3 이 반드시 필요하다 (빼먹으면 전압이 8배로 읽힌다).
     * OVF 는 이 계산에 영향이 없지만 션트 포화의 동반 증상이라 1회만 기록한다. */
    if (((raw & 0x0001U) != 0U) && !s_sat_warned) {
        s_sat_warned = true;
        DBG_W("INA219 math overflow (OVF) - 션트 포화 여부 확인");
    }
    s_bus_mv = (int32_t)(raw >> 3) * 4;

    /* --- Shunt Voltage --- */
    if (!hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA219_REG_SHUNT_V, &raw)) {
        s_ok = false;
        return false;
    }
    /* 2의 보수 16bit, LSB = 10uV */
    sraw       = (int16_t)raw;
    s_shunt_uv = (int32_t)sraw * 10;

    /* PGA /8 풀스케일은 +-32000 카운트다 (int16 한계 32767 이 아니다).
     * 레일에 닿으면 클리핑이고, 과전류를 "임계 미달" 로 오판하는 조용한 실패가 된다. */
    if (((sraw >= 32000) || (sraw <= -32000)) && !s_sat_warned) {
        s_sat_warned = true;
        DBG_W("INA219 shunt saturated (>%ld mA) - 션트/PGA 재검토", (long)INA219_FS_MA);
    }

    /* --- 전류 : I[mA] = V[uV] / R[mohm] --- */
    s_current_ma = DIV_ROUND(s_shunt_uv, CFG_INA219_SHUNT_MOHM);

    return true;
}

int32_t ina219_get_bus_mv(void)      { return s_bus_mv; }
int32_t ina219_get_current_ma(void)  { return s_current_ma; }
int32_t ina219_get_shunt_uv(void)    { return s_shunt_uv; }
bool    ina219_is_ok(void)           { return s_ok; }
