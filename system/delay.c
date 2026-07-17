#include "delay.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

#if (CAR_RECOVERY_SAFE_BUILD != 0U)
#define DELAY_CPUCLK_FREQ (32000000U)
#else
#define DELAY_CPUCLK_FREQ (CPUCLK_FREQ)
#endif

void delay_ms(uint32_t ms)
{
    delay_cycles((DELAY_CPUCLK_FREQ / 1000U) * ms);
}
