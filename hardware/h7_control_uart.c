#include "h7_control_uart.h"

#include "library_config.h"
#include "resource_config.h"

#if CAR_PROFILE_IS_GMR && CAR_H7_UART_REQUIRED

#include "pin_map.h"
#include "ti_msp_dl_config.h"

#define H7_CONTROL_LINE_SIZE          (16U)
#define H7_CONTROL_IRQ_SERVICE_LIMIT  (16U)
#define H7_CONTROL_RX_DRAIN_LIMIT     (64U)
#define H7_CONTROL_TX_TIMEOUT_COUNT   (100000U)

static char g_h7ControlLine[H7_CONTROL_LINE_SIZE];
static uint8_t g_h7ControlLineLength;
static volatile H7ControlCommand g_h7ControlPendingCommand;
static volatile uint8_t g_h7ControlTxBusy;

static const DL_UART_Main_ClockConfig g_h7ControlClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config g_h7ControlConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

static uint32_t H7ControlUart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void H7ControlUart_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint8_t H7ControlUart_LineEquals(const char *expected)
{
    uint8_t index = 0U;

    while ((index < g_h7ControlLineLength) &&
        (expected[index] != '\0') &&
        (g_h7ControlLine[index] == expected[index])) {
        ++index;
    }
    return (uint8_t)(((index == g_h7ControlLineLength) &&
        (expected[index] == '\0')) ? 1U : 0U);
}

static void H7ControlUart_CompleteLine(void)
{
    if (H7ControlUart_LineEquals("@START=1") != 0U) {
        g_h7ControlPendingCommand = H7_CONTROL_COMMAND_START_ATTITUDE;
    } else if (H7ControlUart_LineEquals("@STOP") != 0U) {
        g_h7ControlPendingCommand = H7_CONTROL_COMMAND_STOP;
    }
    g_h7ControlLineLength = 0U;
}

static void H7ControlUart_ConsumeByte(uint8_t data)
{
    if (data == (uint8_t)'\r') {
        return;
    }
    if (data == (uint8_t)'\n') {
        H7ControlUart_CompleteLine();
        return;
    }
    if (g_h7ControlLineLength < (H7_CONTROL_LINE_SIZE - 1U)) {
        g_h7ControlLine[g_h7ControlLineLength++] = (char)data;
    } else {
        g_h7ControlLineLength = 0U;
    }
}

void H7ControlUart_Init(void)
{
    g_h7ControlLineLength = 0U;
    g_h7ControlPendingCommand = H7_CONTROL_COMMAND_NONE;
    g_h7ControlTxBusy = 0U;

    DL_UART_Main_reset(CAR_H7_UART_INST);
    DL_UART_Main_enablePower(CAR_H7_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(PIN_H7_CONTROL_UART_TX_IOMUX,
        PIN_H7_CONTROL_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        PIN_H7_CONTROL_UART_RX_IOMUX,
        PIN_H7_CONTROL_UART_RX_FUNC,
        DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE,
        DL_GPIO_WAKEUP_DISABLE);

    DL_UART_Main_setClockConfig(CAR_H7_UART_INST,
        (DL_UART_Main_ClockConfig *)&g_h7ControlClockConfig);
    DL_UART_Main_init(CAR_H7_UART_INST,
        (DL_UART_Main_Config *)&g_h7ControlConfig);
    DL_UART_Main_configBaudRate(CAR_H7_UART_INST,
        CAR_H7_UART_FREQUENCY, CAR_H7_UART_BAUD_RATE);
    DL_UART_Main_setRXFIFOThreshold(CAR_H7_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enable(CAR_H7_UART_INST);
    DL_UART_Main_enableInterrupt(CAR_H7_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(CAR_H7_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CAR_H7_UART_INST_INT_IRQN);
}

uint8_t H7ControlUart_TrySendBytes(const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint16_t index;
    uint8_t success = 1U;

    if (data == 0) {
        return 0U;
    }
    primask = H7ControlUart_EnterCritical();
    if (g_h7ControlTxBusy != 0U) {
        H7ControlUart_ExitCritical(primask);
        return 0U;
    }
    g_h7ControlTxBusy = 1U;
    H7ControlUart_ExitCritical(primask);

    for (index = 0U; index < length; ++index) {
        uint32_t timeout = H7_CONTROL_TX_TIMEOUT_COUNT;

        while ((timeout > 0U) &&
            !DL_UART_Main_transmitDataCheck(CAR_H7_UART_INST,
                data[index])) {
            --timeout;
        }
        if (timeout == 0U) {
            success = 0U;
            break;
        }
    }

    primask = H7ControlUart_EnterCritical();
    g_h7ControlTxBusy = 0U;
    H7ControlUart_ExitCritical(primask);
    return success;
}

H7ControlCommand H7ControlUart_TakeCommand(void)
{
    uint32_t primask = H7ControlUart_EnterCritical();
    H7ControlCommand command = g_h7ControlPendingCommand;

    g_h7ControlPendingCommand = H7_CONTROL_COMMAND_NONE;
    H7ControlUart_ExitCritical(primask);
    return command;
}

void H7ControlUart_HandleUARTInterrupt(void)
{
    DL_UART_IIDX pending;
    uint8_t data;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;

    do {
        pending = DL_UART_Main_getPendingInterrupt(CAR_H7_UART_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < H7_CONTROL_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(CAR_H7_UART_INST, &data)) {
                H7ControlUart_ConsumeByte(data);
                ++rxCount;
            }
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < H7_CONTROL_IRQ_SERVICE_LIMIT));
}

#else

void H7ControlUart_Init(void)
{
}

uint8_t H7ControlUart_TrySendBytes(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return 0U;
}

H7ControlCommand H7ControlUart_TakeCommand(void)
{
    return H7_CONTROL_COMMAND_NONE;
}

void H7ControlUart_HandleUARTInterrupt(void)
{
}

#endif
