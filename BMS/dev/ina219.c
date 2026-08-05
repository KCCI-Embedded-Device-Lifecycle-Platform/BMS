/**
 * @file    ina219.c
 * @brief   INA219 팩 전압/전류 측정 드라이버 (I2C)
 *
 * @details INA226 과의 차이 (팀 문서는 INA226 기준이므로 주의)
 *   -------------------------------------------------------------
 *    항목            INA226            INA219 (본 프로젝트)
 *   -------------------------------------------------------------
 *    션트 풀스케일   +-81.92mV 고정    +-40/80/160/320mV (PGA 선택)
 *    션트 LSB        2.5uV             10uV
 *    버스 전압 최대  36V               26V   (16.8V 팩은 여유 있음)
 *    버스 LSB        1.25mV            4mV   (레지스터 상위 13bit)
 *   -------------------------------------------------------------
 *   -> INA219 는 PGA /8(320mV) 를 쓰면 0.1옴 션트로 3.2A 까지 측정 가능하다.
 *      따라서 데모 전류가 3A 이하라면 모듈 기본 R100(0.1옴)을 교체할 필요가 없다.
 *      (INA226 은 0.82A 에서 포화하므로 0.01옴 교체가 필수였다)
 *
 * @note  전류 계산을 캘리브레이션 레지스터가 아니라 션트 전압에서 직접 한다.
 *        I[mA] = V_shunt[uV] / R_shunt[mohm]
 *        이렇게 하면 CALIB 레지스터 설정 실수로 인한 오차가 개입하지 않고,
 *        디버깅 시 션트 전압 원값을 그대로 볼 수 있다.
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

/* CALIB = trunc(0.04096 / (Current_LSB * R_shunt))
 * Current_LSB = 100uA, R = 0.1옴  ->  0.04096 / (0.0001 * 0.1) = 4096 */
#define INA219_CALIB_VALUE  4096U

static int32_t s_bus_mv;
static int32_t s_shunt_uv;
static int32_t s_current_ma;
static bool    s_ok;

bool ina219_init(void)
{
    uint16_t val = 0;

    s_ok = false;

    if (!hw_i2c_is_ready(CFG_INA219_I2C_ADDR)) {
        DBG_E("INA219 no ACK at 0x%02X", CFG_INA219_I2C_ADDR);
        return false;
    }

    /* 1) 소프트 리셋 : 이전 세션 설정이 남아있을 수 있다 */
    (void)hw_i2c_reg_write16(CFG_INA219_I2C_ADDR, INA219_REG_CONFIG, INA219_CFG_RESET);
    HAL_Delay(5);

    /* 2) 설정 write */
    if (!hw_i2c_reg_write16(CFG_INA219_I2C_ADDR, INA219_REG_CONFIG, INA219_CFG_VALUE)) {
        DBG_E("INA219 config write fail");
        return false;
    }
    if (!hw_i2c_reg_write16(CFG_INA219_I2C_ADDR, INA219_REG_CALIB, INA219_CALIB_VALUE)) {
        DBG_E("INA219 calib write fail");
        return false;
    }

    /* 3) 읽어서 확인 (write-verify : 드라이버 초기화의 기본) */
    if (!hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA219_REG_CONFIG, &val)) {
        return false;
    }
    if (val != INA219_CFG_VALUE) {
        DBG_E("INA219 config mismatch: w=0x%04X r=0x%04X", INA219_CFG_VALUE, val);
        return false;
    }

    s_ok = true;
    DBG_I("INA219 ok (cfg=0x%04X, shunt=%ld mohm)", val, (long)CFG_INA219_SHUNT_MOHM);
    return true;
}

bool ina219_update(void)
{
    uint16_t raw;

    if (!s_ok) {
        return false;
    }

    /* --- Bus Voltage --- */
    if (!hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA219_REG_BUS_V, &raw)) {
        s_ok = false;
        DBG_E("INA219 bus read fail");
        return false;
    }
    /* bit0=OVF, bit1=CNVR, bit[15:3]=전압, LSB=4mV */
    if ((raw & 0x0001U) != 0U) {
        DBG_W("INA219 math overflow");
    }
    s_bus_mv = (int32_t)(raw >> 3) * 4;

    /* --- Shunt Voltage --- */
    if (!hw_i2c_reg_read16(CFG_INA219_I2C_ADDR, INA219_REG_SHUNT_V, &raw)) {
        s_ok = false;
        return false;
    }
    /* 2의 보수 16bit, LSB = 10uV */
    s_shunt_uv = (int32_t)(int16_t)raw * 10;

    /* --- 전류 : I[mA] = V[uV] / R[mohm] --- */
    s_current_ma = DIV_ROUND(s_shunt_uv, CFG_INA219_SHUNT_MOHM);

    return true;
}

int32_t ina219_get_bus_mv(void)      { return s_bus_mv; }
int32_t ina219_get_current_ma(void)  { return s_current_ma; }
int32_t ina219_get_shunt_uv(void)    { return s_shunt_uv; }
bool    ina219_is_ok(void)           { return s_ok; }
