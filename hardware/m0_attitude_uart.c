/*
 * GMR外部M0姿态UART封装。串口层负责UART3收发和硬件错误统计，
 * 帧同步、CRC、命令编码和yaw处理全部由m0_attitude_link负责。
 */
#include "resource_config.h"

#if CAR_M0_ATTITUDE_UART_REQUIRED

#include "m0_attitude_uart.h"

#include "m0_attitude_link.h"
#include "ti_msp_dl_config.h"

#define M0_ATTITUDE_UART_SERVICE_LIMIT  (16U)
#define M0_ATTITUDE_UART_RX_DRAIN_LIMIT (64U)
#define M0_ATTITUDE_UART_STARTUP_DELAY_MS (500U)

static volatile uint32_t g_m0AttitudeUartRxByteCount;
static volatile uint32_t g_m0AttitudeUartErrorCount;
static volatile uint32_t g_m0AttitudeUartFramingErrorCount;
static volatile uint32_t g_m0AttitudeUartNoiseErrorCount;
static volatile uint32_t g_m0AttitudeUartOverrunErrorCount;
static volatile uint32_t g_m0AttitudeUartTxCommandCount;
static uint32_t g_m0AttitudeUartStartupElapsedMs;
static uint8_t g_m0AttitudeUartCommandSequence;
static uint8_t g_m0AttitudeUartReportRateSent;

void M0AttitudeUart_Init(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(
        CAR_M0_ATTITUDE_UART_RX_IOMUX,
        CAR_M0_ATTITUDE_UART_RX_IOMUX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    g_m0AttitudeUartRxByteCount = 0U;
    g_m0AttitudeUartErrorCount = 0U;
    g_m0AttitudeUartFramingErrorCount = 0U;
    g_m0AttitudeUartNoiseErrorCount = 0U;
    g_m0AttitudeUartOverrunErrorCount = 0U;
    g_m0AttitudeUartTxCommandCount = 0U;
    g_m0AttitudeUartStartupElapsedMs = 0U;
    g_m0AttitudeUartCommandSequence = 0U;
    g_m0AttitudeUartReportRateSent = 0U;

    DL_UART_Main_setRXFIFOThreshold(CAR_M0_ATTITUDE_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(CAR_M0_ATTITUDE_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(CAR_M0_ATTITUDE_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CAR_M0_ATTITUDE_UART_INST_INT_IRQN);
}

void M0AttitudeUart_Task(uint32_t elapsedMs)
{
    uint8_t frame[M0_ATTITUDE_LINK_COMMAND_FRAME_SIZE];
    uint8_t index;

    if (g_m0AttitudeUartReportRateSent != 0U) {
        return;
    }
    if (elapsedMs >= (M0_ATTITUDE_UART_STARTUP_DELAY_MS -
        g_m0AttitudeUartStartupElapsedMs)) {
        g_m0AttitudeUartStartupElapsedMs =
            M0_ATTITUDE_UART_STARTUP_DELAY_MS;
    } else {
        g_m0AttitudeUartStartupElapsedMs += elapsedMs;
    }
    if (g_m0AttitudeUartStartupElapsedMs <
        M0_ATTITUDE_UART_STARTUP_DELAY_MS) {
        return;
    }
    if (M0AttitudeLink_BuildSetReportRate100Hz(
        g_m0AttitudeUartCommandSequence, frame) == 0U) {
        return;
    }
    ++g_m0AttitudeUartCommandSequence;
    for (index = 0U; index < M0_ATTITUDE_LINK_COMMAND_FRAME_SIZE;
        ++index) {
        DL_UART_Main_transmitDataBlocking(
            CAR_M0_ATTITUDE_UART_INST, frame[index]);
    }
    g_m0AttitudeUartReportRateSent = 1U;
    ++g_m0AttitudeUartTxCommandCount;
}

void M0AttitudeUart_HandleUARTInterrupt(void)
{
    DL_UART_IIDX pending;
    uint8_t data;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;

    do {
        pending = DL_UART_Main_getPendingInterrupt(
            CAR_M0_ATTITUDE_UART_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < M0_ATTITUDE_UART_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(
                    CAR_M0_ATTITUDE_UART_INST, &data)) {
                ++g_m0AttitudeUartRxByteCount;
                (void)M0AttitudeLink_ConsumeByte(data);
                ++rxCount;
            }
        } else if (pending == DL_UART_MAIN_IIDX_FRAMING_ERROR) {
            ++g_m0AttitudeUartErrorCount;
            ++g_m0AttitudeUartFramingErrorCount;
        } else if (pending == DL_UART_MAIN_IIDX_NOISE_ERROR) {
            ++g_m0AttitudeUartErrorCount;
            ++g_m0AttitudeUartNoiseErrorCount;
        } else if (pending == DL_UART_MAIN_IIDX_OVERRUN_ERROR) {
            ++g_m0AttitudeUartErrorCount;
            ++g_m0AttitudeUartOverrunErrorCount;
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < M0_ATTITUDE_UART_SERVICE_LIMIT));
}

#endif
