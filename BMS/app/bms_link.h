#ifndef __BMS_LINK_H_
#define __BMS_LINK_H_
#include "common_def.h"

void bms_link_init(void);
void bms_link_send_main(const bms_data_t *p_d);     /* 0x100 + 0x103, 100ms */
void bms_link_send_cell(const bms_data_t *p_d);     /* 0x101 + 0x102, 500ms */
void bms_link_send_version(void);                   /* 0x104,         1s    */
void bms_link_poll_rx(void);                        /* EVSE 수신 처리        */
#endif
