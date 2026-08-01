/*
 * 全部手写中断入口与NVIC优先级配置。
 * ISR只调用对应硬件分发函数并发送FromISR任务通知，不执行显示、日志或控制器主循环。
 * FreeRTOS可调用中断优先级必须保持符合configMAX_SYSCALL_INTERRUPT_PRIORITY约束。
 */
#include "interrupt.h"

#include "board_config.h"
#include "bluetooth_uart.h"
#include "resource_config.h"
#include "encoder_motor.h"
#if CAR_PROFILE_IS_FULL || CAR_LIBRARY_H7_IMU_ENABLED
#include "h7_gyro_link.h"
#endif
#if CAR_JY61P_ENABLED
#include "jy61p.h"
#endif
#include "key.h"
#if CAR_PROFILE_IS_FULL
#include "link.h"
#elif CAR_M0_ATTITUDE_UART_REQUIRED
#include "m0_attitude_uart.h"
#endif
#include "log_uart.h"
#if CAR_PROFILE_IS_FULL && CAR_LIBRARY_LINE_FOLLOW_ENABLED
#include "motor_no_yaw.h"
#endif
#include "rtos_app.h"
#if CAR_PROFILE_IS_FULL
#include "stepper_pulse.h"
#endif
#include "ti_msp_dl_config.h"

void Interrupt_Init(void)
{
#if CAR_PROFILE_IS_FULL
    NVIC_SetPriority(TIMG6_INT_IRQn, 0U);
#endif
#if CAR_PROFILE_IS_FULL && CAR_LIBRARY_LINE_FOLLOW_ENABLED
    NVIC_SetPriority(TIMG0_INT_IRQn, 1U);
#endif
    NVIC_SetPriority(GPIOB_INT_IRQn, 0U);
#if CAR_BLUETOOTH_ENABLED
    NVIC_SetPriority(CAR_BLUETOOTH_UART_INST_INT_IRQN, 2U);
#endif
#if CAR_PROFILE_IS_FULL
    NVIC_SetPriority(UART3_INT_IRQn, 1U);
    NVIC_SetPriority(H7GyroLink_INST_INT_IRQN, 2U);
#elif CAR_M0_ATTITUDE_UART_REQUIRED
    NVIC_SetPriority(CAR_M0_ATTITUDE_UART_INST_INT_IRQN, 2U);
#endif
#if CAR_JY61P_ENABLED
    NVIC_SetPriority(JY61P_INST_INT_IRQN, 2U);
#endif
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
#if CAR_PROFILE_IS_FULL && CAR_LIBRARY_H7_IMU_ENABLED
    H7GyroLink_HandleUARTInterrupt();
#elif CAR_ENABLE_LOG_UART_RX
    LogUart_HandleUARTInterrupt();
#endif
}

void UART1_IRQHandler(void)
{
#if CAR_JY61P_ENABLED
    JY61P_HandleUARTInterrupt();
#endif
}

void UART2_IRQHandler(void)
{
#if CAR_BLUETOOTH_ENABLED && CAR_BLUETOOTH_UART_IS_UART2
    BluetoothUart_HandleUARTInterrupt();
#endif
}

void UART3_IRQHandler(void)
{
#if CAR_BLUETOOTH_ENABLED && CAR_BLUETOOTH_UART_IS_UART3
    BluetoothUart_HandleUARTInterrupt();
#elif CAR_M0_ATTITUDE_UART_REQUIRED
    M0AttitudeUart_HandleUARTInterrupt();
#else
    if (Link_HandleUARTInterrupt() != 0U) {
        RtosApp_NotifyGimbalFromISR();
    }
#endif
}

void TIMG6_IRQHandler(void)
{
#if CAR_PROFILE_IS_FULL
    StepperPulse_HandleTimerInterrupt();
#endif
}

void TIMG0_IRQHandler(void)
{
#if CAR_PROFILE_IS_FULL && CAR_LIBRARY_LINE_FOLLOW_ENABLED
    switch (DL_TimerG_getPendingInterrupt(GRAY_SAMPLE_TIMER_INST)) {
    case DL_TIMER_IIDX_ZERO:
        if (MotorNoYaw_TimerSample() != 0U) {
            RtosApp_NotifyControlFromISR();
        }
        break;
    default:
        break;
    }
#endif
}
