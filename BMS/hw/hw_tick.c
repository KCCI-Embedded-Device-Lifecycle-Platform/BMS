#include "hw_tick.h"

uint32_t hw_tick_ms(void)
{
    return HAL_GetTick();
}

void hw_tick_delay(uint32_t ms)
{
    HAL_Delay(ms);
}

bool hw_tick_due(uint32_t *p_last, uint32_t period)
{
    uint32_t now = HAL_GetTick();

    /* (now - *p_last) 는 unsigned 연산이므로 now 가 wrap 되어도 결과가 옳다 */
    if ((uint32_t)(now - *p_last) >= period) {
        *p_last = now;
        return true;
    }
    return false;
}

bool hw_tick_elapsed(uint32_t start, uint32_t period)
{
    return ((uint32_t)(HAL_GetTick() - start) >= period);
}
