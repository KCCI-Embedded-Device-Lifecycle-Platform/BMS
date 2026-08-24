#ifndef __HW_ADC_H_
#define __HW_ADC_H_
#include "common_def.h"

/**
 * @brief  ADC1 변환 랭크 인덱스
 * @warning CubeMX 의 Rank 순서와 반드시 1:1 로 일치해야 한다.
 *          Rank1=IN0, Rank2=IN1, Rank3=IN4, Rank4=IN8,
 *          Rank5=IN11, Rank6=IN10, Rank7=IN13, Rank8=IN14,
 *          Rank9=IN12, Rank10=Vrefint
 */
typedef enum {
    ADC_IDX_NODE_B1 = 0,    /* PA0  IN0  : 셀1 누적    */
    ADC_IDX_NODE_B2,        /* PA1  IN1  : 셀1+2 누적  */
    ADC_IDX_NODE_B3,        /* PA4  IN4  : 셀1+2+3     */
    ADC_IDX_NODE_BP,        /* PB0  IN8  : 팩 전압     */
    ADC_IDX_NTC1,           /* PC1  IN11 : 온도1       */
    ADC_IDX_NTC2,           /* PC0  IN10 : 온도2       */
    ADC_IDX_NTC3,           /* PC3  IN13 : 온도3       */
    ADC_IDX_NTC4,           /* PC4  IN14 : 온도4       */
    ADC_IDX_ACS712,         /* PC2  IN12 : 홀 전류     */
    ADC_IDX_VREFINT,        /*      IN17 : VDDA 보정   */
    ADC_IDX_MAX
} adc_idx_t;

bool     hw_adc_init(void);
bool     hw_adc_is_ready(void);                 /* 첫 평균 완료 여부 */
bool     hw_adc_is_fresh(uint32_t max_age_ms);  /* DMA 평균이 최근에도 갱신됐는가 */
void     hw_adc_snapshot(uint16_t *p_out);      /* ADC_IDX_MAX 개 원자적 복사 */
uint16_t hw_adc_get_raw(adc_idx_t idx);
int32_t  hw_adc_get_vdda_mv(void);              /* Vrefint 기반 실측 VDDA */
int32_t  hw_adc_raw_to_uv(uint16_t raw);        /* 핀 전압 [uV] */
#endif
