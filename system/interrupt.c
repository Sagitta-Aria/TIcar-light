#include "interrupt.h"

#include "encoder.h"
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
