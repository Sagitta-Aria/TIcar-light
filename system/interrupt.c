#include "interrupt.h"

#include "board_config.h"
#include "encoder.h"
#include "jy61p.h"
#include "key.h"
#include "ti_msp_dl_config.h"

/* UART 兜底处理只清理有限数量，避免异常 RX 噪声导致中断里死循环。 */
#define INTERRUPT_UART_SERVICE_LIMIT    (16U)
#define INTERRUPT_UART_RX_DRAIN_LIMIT   (32U)

void Interrupt_Init(void)
{
}

/*
 * 作用：处理 GROUP1 里的 GPIO 中断。
 * 使用场景：MSPM0G3507 的 GPIOA/GPIOB 都挂在 GROUP1 向量上。
 * 说明：启动文件真正调用的是 GROUP1_IRQHandler，不是 GPIOA/B_IRQHandler。
 */
static void Interrupt_HandleGroup1(void)
{
#if CAR_ENABLE_ENCODER_INPUTS
    Encoder_HandleGPIOInterrupt();
#endif
    Key_HandleGPIOInterrupt();
}

/*
 * 作用：兜底清掉暂未使用的 UART 中断源。
 * 使用场景：某个 UART 只打开 NVIC 但暂时没有接收业务时，避免落入默认死循环。
 */
static void Interrupt_ClearUART(UART_Regs *uart)
{
    DL_UART_IIDX pending;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;
    uint8_t data;

    do {
        pending = DL_UART_Main_getPendingInterrupt(uart);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < INTERRUPT_UART_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(uart, &data)) {
                ++rxCount;
            }
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < INTERRUPT_UART_SERVICE_LIMIT));
}

void GROUP1_IRQHandler(void)
{
    Interrupt_HandleGroup1();
}

void GPIOA_IRQHandler(void)
{
#if CAR_ENABLE_ENCODER_INPUTS
    Encoder_HandleGPIOInterrupt();
#endif
}

void GPIOB_IRQHandler(void)
{
    Key_HandleGPIOInterrupt();
}

void UART0_IRQHandler(void)
{
    JY61P_HandleUARTInterrupt();
}

void UART1_IRQHandler(void)
{
    Interrupt_ClearUART(JQ8400_INST);
}

void UART3_IRQHandler(void)
{
    Interrupt_ClearUART(Exchange_INST);
}
