#include "interrupt.h"

#include "encoder.h"
#include "jy61p.h"
#include "key.h"

void Interrupt_Init(void)
{
}

void GPIOA_IRQHandler(void)
{
    Encoder_HandleGPIOInterrupt();
}

void GPIOB_IRQHandler(void)
{
    Key_HandleGPIOInterrupt();
}

void UART0_IRQHandler(void)
{
    JY61P_HandleUARTInterrupt();
}
