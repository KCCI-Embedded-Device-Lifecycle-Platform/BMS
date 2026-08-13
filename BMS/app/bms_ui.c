/**
 * @file    bms_ui.c
 * @brief   OLED 로컬 상태 표시
 * @note    128x64 / 5x7 폰트 -> 가로 21자, 세로 8줄.
 *          I2C 트래픽을 아끼려고 500ms 주기로만 갱신한다.
 */
#include "bms_ui.h"
#include "bms_state.h"
#include "bms_fault.h"
#include "oled.h"

void bms_ui_init(void)
{
    if (oled_is_ok()) {
        oled_clear();
        oled_draw_str(0, 0, "  EV BMS  STAGE1");
        oled_draw_str(0, 2, "   booting ...");
        oled_flush();
    }
}

void bms_ui_update(const bms_data_t *p_d)
{
    if (!oled_is_ok()) {
        return;
    }

    oled_clear();

    /* 0행 : 상태 + permit + 릴레이 지령을 나란히.
     * 둘이 다르면 EVSE 요청이 없거나 FSM 이 충전 구간에 없다는 뜻이다. */
    oled_printf(0, 0, "%-8s P:%d R:%d",
                bms_fsm_state_name(p_d->state),
                (int)p_d->charge_permit, (int)p_d->relay_on);

    /* 1~2행 : 셀 전압 4개 (2개씩) */
    oled_printf(0, 1, "C1 %ld.%03ld  C2 %ld.%03ld",
                (long)(p_d->cell_mv[0] / 1000), (long)(p_d->cell_mv[0] % 1000),
                (long)(p_d->cell_mv[1] / 1000), (long)(p_d->cell_mv[1] % 1000));
    oled_printf(0, 2, "C3 %ld.%03ld  C4 %ld.%03ld",
                (long)(p_d->cell_mv[2] / 1000), (long)(p_d->cell_mv[2] % 1000),
                (long)(p_d->cell_mv[3] / 1000), (long)(p_d->cell_mv[3] % 1000));

    /* 3행 : 팩 전압/전류 */
    oled_printf(0, 3, "PACK %ld.%02ldV %ldmA",
                (long)(p_d->pack_mv / 1000), (long)((p_d->pack_mv % 1000) / 10),
                (long)p_d->pack_ma);

    /* 4행 : 온도 / SOC */
    if (p_d->temp_c10 == BMS_TEMP_INVALID) {
        oled_printf(0, 4, "TEMP --.-C  SOC %u%%", p_d->soc);
    } else {
        oled_printf(0, 4, "TEMP %ld.%ldC  SOC %u%%",
                    (long)(p_d->temp_c10 / 10),
                    (long)((p_d->temp_c10 < 0 ? -p_d->temp_c10 : p_d->temp_c10) % 10),
                    p_d->soc);
    }

    /* 5행 : 셀 편차 */
    oled_printf(0, 5, "IMBAL %ldmV", (long)p_d->imbalance_mv);

    /* 6행 : Fault */
    oled_printf(0, 6, "FLT %s (0x%04X)", bms_fault_name(p_d->fault), p_d->fault);

    /* 7행 : VDDA (캘리브레이션 확인용) */
    oled_printf(0, 7, "VDDA %ldmV", (long)p_d->vdda_mv);

    oled_flush();
}
