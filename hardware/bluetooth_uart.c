#include "bluetooth_uart.h"

#include <stdint.h>

#include "bluetooth_config.h"

#if CAR_BLUETOOTH_ENABLED

#include "bluetooth_service.h"
#include "pin_map.h"
#include "resource_config.h"
#include "ti_msp_dl_config.h"

#define BLUETOOTH_UART_IRQ_SERVICE_LIMIT    (16U)
#define BLUETOOTH_UART_RX_DRAIN_LIMIT       (64U)

static volatile uint8_t g_bluetoothTxInterruptEnabled;

static const DL_UART_Main_ClockConfig g_bluetoothUartClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config g_bluetoothUartConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

void BluetoothUart_Init(void)
{
    g_bluetoothTxInterruptEnabled = 0U;

    DL_UART_Main_reset(CAR_BLUETOOTH_UART_INST);
    DL_UART_Main_enablePower(CAR_BLUETOOTH_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(PIN_BLUETOOTH_UART_TX_IOMUX,
        PIN_BLUETOOTH_UART_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(PIN_BLUETOOTH_UART_RX_IOMUX,
        PIN_BLUETOOTH_UART_RX_FUNC);

    DL_UART_Main_setClockConfig(CAR_BLUETOOTH_UART_INST,
        (DL_UART_Main_ClockConfig *)&g_bluetoothUartClockConfig);
    DL_UART_Main_init(CAR_BLUETOOTH_UART_INST,
        (DL_UART_Main_Config *)&g_bluetoothUartConfig);
    DL_UART_Main_configBaudRate(CAR_BLUETOOTH_UART_INST,
        CAR_BLUETOOTH_UART_FREQUENCY, CAR_BLUETOOTH_UART_BAUD_RATE);
    DL_UART_Main_setRXFIFOThreshold(CAR_BLUETOOTH_UART_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(CAR_BLUETOOTH_UART_INST,
        DL_UART_MAIN_TX_FIFO_LEVEL_EMPTY);
    DL_UART_Main_enable(CAR_BLUETOOTH_UART_INST);

    DL_UART_Main_enableInterrupt(CAR_BLUETOOTH_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(CAR_BLUETOOTH_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(CAR_BLUETOOTH_UART_INST_INT_IRQN);
}

void BluetoothUart_Task(void)
{
    uint8_t data;

    if ((g_bluetoothTxInterruptEnabled == 0U) &&
        !DL_UART_Main_isTXFIFOFull(CAR_BLUETOOTH_UART_INST) &&
        (BluetoothService_TakeTxByteFromISR(&data) != 0U)) {
        DL_UART_Main_transmitData(CAR_BLUETOOTH_UART_INST, data);
        g_bluetoothTxInterruptEnabled = 1U;
        DL_UART_Main_enableInterrupt(CAR_BLUETOOTH_UART_INST,
            DL_UART_MAIN_INTERRUPT_TX);
    }
}

void BluetoothUart_HandleUARTInterrupt(void)
{
    DL_UART_IIDX pending;
    uint8_t data;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;
    uint8_t txQueueEmpty;

    do {
        pending = DL_UART_Main_getPendingInterrupt(CAR_BLUETOOTH_UART_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < BLUETOOTH_UART_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(
                    CAR_BLUETOOTH_UART_INST, &data)) {
                BluetoothService_InputByteFromISR(data);
                ++rxCount;
            }
        } else if (pending == DL_UART_MAIN_IIDX_TX) {
            txQueueEmpty = 0U;
            while (!DL_UART_Main_isTXFIFOFull(CAR_BLUETOOTH_UART_INST)) {
                if (BluetoothService_TakeTxByteFromISR(&data) == 0U) {
                    txQueueEmpty = 1U;
                    break;
                }
                DL_UART_Main_transmitData(CAR_BLUETOOTH_UART_INST, data);
            }
            if (txQueueEmpty != 0U) {
                DL_UART_Main_disableInterrupt(CAR_BLUETOOTH_UART_INST,
                    DL_UART_MAIN_INTERRUPT_TX);
                g_bluetoothTxInterruptEnabled = 0U;
            }
        } else if ((pending == DL_UART_MAIN_IIDX_FRAMING_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_NOISE_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_OVERRUN_ERROR)) {
            /* Reading IIDX acknowledges the hardware error source. */
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < BLUETOOTH_UART_IRQ_SERVICE_LIMIT));
}

#else

void BluetoothUart_Init(void)
{
}

void BluetoothUart_Task(void)
{
}

void BluetoothUart_HandleUARTInterrupt(void)
{
}

#endif
