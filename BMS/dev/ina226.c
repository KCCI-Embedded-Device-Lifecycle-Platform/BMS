/**
 * @file    ina226.c
 * @brief   INA226 팩 전압/전류 측정 드라이버 (I2C)
 *
 * INA219 와 파일을 나눈 이유: 레지스터 이름만 같고 스케일이 전부 다르다.
 *
 *    항목            INA219                  INA226 (최종 부품)
 *    션트 풀스케일   +-40/80/160/320mV(PGA)  +-81.92mV 고정 (PGA 없음)
 *    션트 LSB        10uV                    2.5uV
 *    버스            26V / 4mV / >>3 필요    36V / 1.25mV / 시프트 없음
 *    평균화          없음                    하드웨어 AVG 1~1024회
 *
 * -> 분해능은 4배 좋지만 PGA 가 없어 "션트 선택 = 측정 범위" 다. 모듈 기본 R100(0.1옴)
 *    이면 +-0.82A 에서 포화하므로, 과전류 임계(1A)나 실팩 전류를 재려면 0.01옴 교체가
 *    필요하다. 런타임에는 "전류가 안 올라가네" 로만 보이므로 아래 검사로 빌드에서 잡는다.
 *
 * @note  전류를 CURRENT 레지스터가 아니라 션트 전압에서 직접 계산하는 이유는 ina219.c 와
 *        같다 — CALIB 설정 실수가 개입하지 않고 원값(uV)을 그대로 볼 수 있다.
 */
#include "ina226.h"
#include "hw_i2c.h"
#include "dbg.h"

/* --- CONFIG 레지스터 비트 구성 ---
 *  [15]    RST     = 0
 *  [14:12] 고정 100b (데이터시트가 이 값을 강제한다 -> 0x4000 을 항상 OR)
 *  [11:9]  AVG     = 2  : 16회 평균
 *  [8:6]   VBUSCT  = 4  : 1.1ms
 *  [5:3]   VSHCT   = 4  : 1.1ms
 *  [2:0]   MODE    = 7  : Shunt+Bus, Continuous
 *
 * 변환 1주기 = 16 x (1.1+1.1)ms = 35.2ms. 100ms 슬롯보다 짧아야 매번 새 값이 읽힌다
 * (평균 횟수를 더 올리면 값이 정체된다).
 */
#define INA226_CFG_VALUE    0x4527U
#define INA226_CFG_RESET    0x8000U

#define INA226_MFG_ID_TI    0x5449U

/* CALIB = trunc(0.00512 / (Current_LSB * R_shunt)). Current_LSB=100uA -> 51200/R[mohm].
 * 션트를 바꿔도 따라가도록 상수 대신 식으로 둔다. */
#define INA226_CALIB_VALUE  ((uint16_t)(51200L / CFG_INA226_SHUNT_MOHM))

/* 션트 풀스케일 +-81.92mV -> 측정 가능한 최대 전류[mA] */
#define INA226_FS_MA        (81920L / CFG_INA226_SHUNT_MOHM)

#if (CFG_PACK_SENSOR == CFG_PACK_SENSOR_INA226)
  #if (INA226_FS_MA <= CFG_OVER_CURRENT_MA)
    #warning "INA226 shunt saturates below CFG_OVER_CURRENT_MA -- \
over-current fault can never be measured. Replace the module shunt with 0.01ohm \
(CFG_INA226_SHUNT_MOHM=10) or lower CFG_OVER_CURRENT_MA."
  #endif
#endif

static int32_t s_bus_mv;
static int32_t s_shunt_uv;
static int32_t s_current_ma;
static bool    s_ok;
static bool    s_sat_warned;        /* 포화 경고는 1회만 (100ms 주기 로그 폭주 방지) */

bool ina226_init(void)
{
    uint16_t val = 0;

    s_ok         = false;
    s_sat_warned = false;

    if (!hw_i2c_is_ready(CFG_INA226_I2C_ADDR)) {
        DBG_E("INA226 no ACK at 0x%02X (A1/A0 배선 확인)", CFG_INA226_I2C_ADDR);
        return false;
    }

    /* 1) 제조사 ID 확인. INA219 를 같은 0x40 에 꽂아 두고 이 드라이버를 빌드하는 실수를
     *    걸러낸다 (INA219 에는 0xFE 가 없다). 클론 칩이 ID 를 안 채우기도 해서 경고만 한다. */
    if (hw_i2c_reg_read16(CFG_INA226_I2C_ADDR, INA226_REG_MFG_ID, &val)) {
        if (val != INA226_MFG_ID_TI) {
            DBG_W("INA226 mfg id = 0x%04X (expect 0x%04X) - INA219 아닌지 확인",
                  val, INA226_MFG_ID_TI);
        }
    }

    /* 2) 소프트 리셋 : 이전 세션 설정이 남아있을 수 있다 */
    (void)hw_i2c_reg_write16(CFG_INA226_I2C_ADDR, INA226_REG_CONFIG, INA226_CFG_RESET);
    HAL_Delay(5);

    /* 3) 설정 write */
    if (!hw_i2c_reg_write16(CFG_INA226_I2C_ADDR, INA226_REG_CONFIG, INA226_CFG_VALUE)) {
        DBG_E("INA226 config write fail");
        return false;
    }
    if (!hw_i2c_reg_write16(CFG_INA226_I2C_ADDR, INA226_REG_CALIB, INA226_CALIB_VALUE)) {
        DBG_E("INA226 calib write fail");
        return false;
    }

    /* 4) 읽어서 확인 (write-verify) */
    if (!hw_i2c_reg_read16(CFG_INA226_I2C_ADDR, INA226_REG_CONFIG, &val)) {
        return false;
    }
    if (val != INA226_CFG_VALUE) {
        DBG_E("INA226 config mismatch: w=0x%04X r=0x%04X", INA226_CFG_VALUE, val);
        return false;
    }

    s_ok = true;
    DBG_I("INA226 ok (cfg=0x%04X, shunt=%ld mohm, FS=%ld mA)",
          val, (long)CFG_INA226_SHUNT_MOHM, (long)INA226_FS_MA);
    return true;
}

bool ina226_update(void)
{
    uint16_t raw;
    int16_t  sraw;

    if (!s_ok) {
        return false;
    }

    /* --- Bus Voltage : 16bit 전체가 전압, LSB 1.25mV ---
     * INA219 와 달리 시프트가 없다 (>>3 을 남겨두면 전압이 1/8 로 읽힌다 - 포팅 1순위 함정) */
    if (!hw_i2c_reg_read16(CFG_INA226_I2C_ADDR, INA226_REG_BUS_V, &raw)) {
        s_ok = false;
        DBG_E("INA226 bus read fail");
        return false;
    }
    s_bus_mv = ((int32_t)raw * 5) / 4;

    /* --- Shunt Voltage : 2의 보수 16bit, LSB = 2.5uV --- */
    if (!hw_i2c_reg_read16(CFG_INA226_I2C_ADDR, INA226_REG_SHUNT_V, &raw)) {
        s_ok = false;
        return false;
    }
    sraw       = (int16_t)raw;
    s_shunt_uv = ((int32_t)sraw * 5) / 2;

    /* 레일에 닿으면 측정이 아니라 클리핑이고, 과전류를 "임계 미달" 로 오판하게 된다. */
    if (((sraw >= 32767) || (sraw <= -32768)) && !s_sat_warned) {
        s_sat_warned = true;
        DBG_W("INA226 shunt saturated (>%ld mA) - 션트 교체 필요", (long)INA226_FS_MA);
    }

    /* --- 전류 : I[mA] = V[uV] / R[mohm] --- */
    s_current_ma = DIV_ROUND(s_shunt_uv, CFG_INA226_SHUNT_MOHM);

    return true;
}

int32_t ina226_get_bus_mv(void)      { return s_bus_mv; }
int32_t ina226_get_current_ma(void)  { return s_current_ma; }
int32_t ina226_get_shunt_uv(void)    { return s_shunt_uv; }
bool    ina226_is_ok(void)           { return s_ok; }
