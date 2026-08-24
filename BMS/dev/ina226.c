/**
 * @file    ina226.c
 * @brief   INA226 팩 전류/EVSE측 버스 전압 측정 드라이버 (I2C)
 *
 * 이 프로젝트의 팩 전류 센서는 INA226 하나다 (2026-08-14, INA219 대체 경로 제거).
 *
 * 핀 호환인 INA219 와 헷갈리기 쉬운 지점이 세 곳 있다 — 잘못 포팅하면 컴파일은 되고
 * 값만 조용히 틀어지므로 미리 적어 둔다:
 *
 *    항목            INA219                  INA226 (이 파일)
 *    션트 풀스케일   +-40/80/160/320mV(PGA)  +-81.92mV 고정 (PGA 없음)
 *    션트 LSB        10uV                    2.5uV
 *    버스            26V / 4mV / >>3 필요    36V / 1.25mV / 시프트 없음
 *    평균화          없음                    하드웨어 AVG 1~1024회
 *
 * -> 분해능은 4배 좋지만 PGA 가 없어 "션트 선택 = 측정 범위" 다. 모듈 기본 R100(0.1옴)
 *    이면 +-0.82A 에서 포화하므로, 과전류 임계(1A)나 실팩 전류를 재려면 0.01옴 교체가
 *    필요하다. 런타임에는 "전류가 안 올라가네" 로만 보이므로 아래 검사로 빌드에서 잡는다.
 *
 * @note  전류를 CURRENT 레지스터가 아니라 션트 전압에서 직접 계산한다 —
 *        CALIB 설정 실수가 개입하지 않고 원값(uV)을 그대로 볼 수 있다.
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

/* 0.1ohm 션트를 유지하면 INA226 은 819mA에서 포화한다. 이 구성에서는 ACS712가
 * 포화 이후 전류와 과전류 보호를 이어받는다. ACS 경로까지 꺼진 빌드는 안전하지 않다. */
#if (INA226_FS_MA <= CFG_OVER_CURRENT_MA) && (BRINGUP_S4_NTC_ACS == 0)
  #error "INA226 range is below over-current threshold; enable ACS712 fallback"
#endif

/* 버스 글리치로 죽었을 때 재설정을 몇 번까지 시도할지 (oled.c 와 같은 이유·같은 형태).
 * 무제한이면 버스가 진짜 죽었을 때 100ms 슬롯이 매번 I2C 타임아웃만큼 밀려
 * "Fault 후 100ms 내 permit=0" 이 무너진다. 20회(약 2초) 뒤 포기한다. */
#define INA226_RECFG_MAX    20U

static int32_t s_bus_mv;
static int32_t s_shunt_uv;
static int32_t s_current_ma;
static bool    s_ok;
static bool    s_sat_warned;        /* 포화 경고는 1회만 (100ms 주기 로그 폭주 방지) */
static bool    s_saturated;
static uint8_t s_recfg_cnt;         /* 버스 글리치 후 재설정 시도 횟수 */

/**
 * @brief  ACK 확인 + 소프트리셋 + CONFIG/CALIB write + write-verify
 * @note   init() 과 "주기 읽기 실패 후 자동 재설정" 이 공유한다.
 *         재시도 카운터를 건드리지 않는 것이 init() 과의 유일한 차이다.
 */
static bool ina226_configure(void)
{
    uint16_t val = 0;

    if (!hw_i2c_is_ready(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR)) {
        DBG_E("INA226 no ACK at 0x%02X (A1/A0 배선 확인)", CFG_INA226_I2C_ADDR);
        return false;
    }

    /* 1) 제조사 ID 확인. 모듈이 정말 INA226 인지 보는 유일한 수단이다 —
     *    핀·주소가 같은 INA219 를 꽂아도 ACK 는 똑같이 오고, 그대로 두면 전압 8배·
     *    전류 4배로 조용히 오측정된다 (INA219 에는 0xFE 레지스터가 없다).
     *    클론 칩이 ID 를 안 채우는 경우가 있어 거부하지 않고 경고만 한다. */
    if (hw_i2c_reg_read16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_MFG_ID, &val)) {
        if (val != INA226_MFG_ID_TI) {
            DBG_W("INA226 mfg id = 0x%04X (expect 0x%04X) - INA219 를 꽂은 것 아닌지 확인",
                  val, INA226_MFG_ID_TI);
        }
    }

    /* 2) 소프트 리셋 : 이전 세션 설정이 남아있을 수 있다 */
    (void)hw_i2c_reg_write16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_CONFIG, INA226_CFG_RESET);
    HAL_Delay(5);

    /* 3) 설정 write */
    if (!hw_i2c_reg_write16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_CONFIG, INA226_CFG_VALUE)) {
        DBG_E("INA226 config write fail");
        return false;
    }
    if (!hw_i2c_reg_write16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_CALIB, INA226_CALIB_VALUE)) {
        DBG_E("INA226 calib write fail");
        return false;
    }

    /* 4) 읽어서 확인 (write-verify) */
    if (!hw_i2c_reg_read16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_CONFIG, &val)) {
        /* 여기가 조용하면 init 이 아무 흔적 없이 false 를 돌려줘서, 호출부 로그에
         * INA226 줄이 통째로 사라진다 ('b' 출력에서 실제로 그랬다).
         * write 는 되는데 read 만 실패하는 패턴은 SDA 상승시간(풀업) 문제를 시사한다 —
         * read 는 슬레이브가 SDA 를 놓고 풀업이 끌어올려야 하는 구간이 더 많다. */
        DBG_E("INA226 config read-back fail (write 는 됐는데 read 가 실패 = 풀업/상승시간 의심)");
        return false;
    }
    if (val != INA226_CFG_VALUE) {
        DBG_E("INA226 config mismatch: w=0x%04X r=0x%04X", INA226_CFG_VALUE, val);
        return false;
    }

    return true;
}

bool ina226_init(void)
{
    s_ok         = false;
    s_sat_warned = false;
    s_saturated  = false;
    s_recfg_cnt  = 0U;

    if (!ina226_configure()) {
        return false;
    }

    s_ok = true;
    DBG_I("INA226 ok (cfg=0x%04X, shunt=%ld mohm, FS=%ld mA)",
          INA226_CFG_VALUE, (long)CFG_INA226_SHUNT_MOHM, (long)INA226_FS_MA);
    return true;
}

bool ina226_update(void)
{
    uint16_t raw;
    int16_t  sraw;

    /* 전송 실패 한 번으로 리셋까지 영구히 죽는 것을 막는다.
     * PACK 전압은 셀 ADC B+가 계속 제공하므로, 재시도가 없으면 INA226 전류 보호 경로만
     * 죽은 상태가 전압 뒤에 가려진다. */
    if (!s_ok) {
        if (s_recfg_cnt >= INA226_RECFG_MAX) {
            return false;               /* 포기 (아래 경고를 이미 냈다) */
        }
        s_recfg_cnt++;
        if (!ina226_configure()) {
            if (s_recfg_cnt == INA226_RECFG_MAX) {
                DBG_E("INA226 재설정 %u회 실패 - 포기. 'i' 로 수동 재시도", s_recfg_cnt);
            }
            return false;
        }
        s_ok = true;
        DBG_W("INA226 re-init ok (%u회째) - I2C 버스가 불안정하다", s_recfg_cnt);
    }

    /* --- Bus Voltage : 16bit 전체가 전압, LSB 1.25mV ---
     * INA219 와 달리 시프트가 없다 (>>3 을 남겨두면 전압이 1/8 로 읽힌다 - 포팅 1순위 함정) */
    if (!hw_i2c_reg_read16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_BUS_V, &raw)) {
        s_ok = false;
        DBG_E("INA226 bus read fail");
        return false;
    }
    s_bus_mv = ((int32_t)raw * 5) / 4;

    /* --- Shunt Voltage : 2의 보수 16bit, LSB = 2.5uV --- */
    if (!hw_i2c_reg_read16(HW_I2C_SENSOR, CFG_INA226_I2C_ADDR, INA226_REG_SHUNT_V, &raw)) {
        s_ok = false;
        /* 이 로그가 없어서 오래 헤맸다 — 버스 읽기는 로그를 내는데 션트 읽기만
         * 조용히 죽어서, "INA226 ok 인데 값이 안 들어온다" 로만 보였다.
         * 실패 경로에는 반드시 흔적을 남길 것. */
        DBG_E("INA226 shunt read fail");
        return false;
    }
    sraw       = (int16_t)raw;
    s_shunt_uv = ((int32_t)sraw * 5) / 2;

    /* 레일에 닿으면 측정이 아니라 클리핑이고, 과전류를 "임계 미달" 로 오판하게 된다. */
    s_saturated = ((sraw >= 32760) || (sraw <= -32760));
    if (s_saturated && !s_sat_warned) {
        s_sat_warned = true;
        DBG_W("INA226 shunt saturated (>%ldmA) -> current source ACS712", (long)INA226_FS_MA);
    }

    /* --- 전류 : I[mA] = V[uV] / R[mohm] --- */
    s_current_ma = DIV_ROUND(s_shunt_uv, CFG_INA226_SHUNT_MOHM);

    return true;
}

int32_t ina226_get_bus_mv(void)      { return s_bus_mv; }
int32_t ina226_get_current_ma(void)  { return s_current_ma; }
int32_t ina226_get_shunt_uv(void)    { return s_shunt_uv; }
bool    ina226_is_ok(void)           { return s_ok; }
bool    ina226_is_saturated(void)    { return s_saturated; }
int32_t ina226_get_full_scale_ma(void) { return INA226_FS_MA; }
