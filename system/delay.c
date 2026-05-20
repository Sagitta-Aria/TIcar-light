#include "delay.h"

#include "ti_msp_dl_config.h"

void delay_ms(uint32_t ms)
{
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);
}
