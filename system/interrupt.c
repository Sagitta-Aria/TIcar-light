#include "interrupt.h"

#include "encoder_motor.h"
#include "jy61p.h"
#include "key.h"
#include "link.h"
#include "log_uart.h"
#include "motor_no_yaw.h"
#include "rtos_app.h"
#include "stepper_pulse.h"
#include "ti_msp_dl_config.h"

void Interrupt_Init(void)
{
    NVIC_SetPriority(TIMG0_INT_IRQn, 0U);
    NVIC_SetPriority(GPIOB_INT_IRQn, 0U);
    NVIC_SetPriority(UART3_INT_IRQn, 1U);
    NVIC_SetPriority(UART1_INT_IRQn, 2U);
    NVIC_SetPriority(UART0_INT_IRQn, 2U);
}

/*
 * 作用：处理 GROUP1 里的 GPIO 中断。
 * 使用场景：MSPM0G3507 的 GPIOA/GPIOB 都挂在 GROUP1 向量上。
 * 说明：启动文件真正调用的是 GROUP1_IRQHandler，不是 GPIOA/B_IRQHandler。
 */
static void Interrupt_HandleGroup1(void)
{
    EncoderMotor_HandleGPIOInterrupt();
    if (Key_HandleGPIOInterrupt() != 0U) {
        RtosApp_NotifyInputFromISR();
    }
}

void GROUP1_IRQHandler(void)
{
    Interrupt_HandleGroup1();
}

void GPIOA_IRQHandler(void)
{
    EncoderMotor_HandleGPIOInterrupt();
}

void GPIOB_IRQHandler(void)
{
    EncoderMotor_HandleGPIOInterrupt();
    if (Key_HandleGPIOInterrupt() != 0U) {
        RtosApp_NotifyInputFromISR();
    }
}

void UART0_IRQHandler(void)
{
    LogUart_HandleUARTInterrupt();
}

void UART1_IRQHandler(void)
{
    JY61P_HandleUARTInterrupt();
}

void UART3_IRQHandler(void)
{
    if (Link_HandleUARTInterrupt() != 0U) {
        RtosApp_NotifyGimbalFromISR();
    }
}

void TIMG0_IRQHandler(void)
{
    StepperPulse_HandleTimerInterrupt();
    if (MotorNoYaw_TimerSample() != 0U) {
        RtosApp_NotifyControlFromISR();
    }
}
