#include "log_uart.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

#define LOG_UART_TX_TIMEOUT_COUNT    (100000U)
#define LOG_UART_RX_BUFFER_SIZE      (128U)
#define LOG_UART_RX_BUFFER_MASK      (LOG_UART_RX_BUFFER_SIZE - 1U)
#define LOG_UART_IRQ_SERVICE_LIMIT   (16U)
#define LOG_UART_IRQ_RX_DRAIN_LIMIT  (64U)

#if ((LOG_UART_RX_BUFFER_SIZE & LOG_UART_RX_BUFFER_MASK) != 0U)
#error "LOG_UART_RX_BUFFER_SIZE must be a power of two"
#endif

static volatile uint8_t g_logRxBuffer[LOG_UART_RX_BUFFER_SIZE];
static volatile uint8_t g_logRxWriteIndex;
static volatile uint8_t g_logRxReadIndex;
static volatile uint32_t g_logRxDropCount;
static volatile uint32_t g_logRxErrorCount;
static volatile uint8_t g_logTxBusy;

static uint32_t LogUart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void LogUart_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint8_t LogUart_TryLockTx(void)
{
    uint32_t primask = LogUart_EnterCritical();

    if (g_logTxBusy != 0U) {
        LogUart_ExitCritical(primask);
        return 0U;
    }
    g_logTxBusy = 1U;
    LogUart_ExitCritical(primask);
    return 1U;
}

static void LogUart_UnlockTx(void)
{
    uint32_t primask = LogUart_EnterCritical();

    g_logTxBusy = 0U;
    LogUart_ExitCritical(primask);
}

static void LogUart_PushRxByte(uint8_t data)
{
    uint8_t next = (uint8_t)((g_logRxWriteIndex + 1U) &
        LOG_UART_RX_BUFFER_MASK);

    if (next == g_logRxReadIndex) {
        ++g_logRxDropCount;
        return;
    }
    g_logRxBuffer[g_logRxWriteIndex] = data;
    g_logRxWriteIndex = next;
}

/*
 * 作用：带超时发送 1 字节日志。
 * 使用场景：启动日志、状态机日志、菜单监视数据。
 * 说明：即使 CH340 未连接或 UART 状态异常，也不能因为打印卡死主循环。
 */
static uint8_t LogUart_TrySendByte(uint8_t data)
{
#if CAR_ENABLE_LOG_UART
    uint32_t timeout = LOG_UART_TX_TIMEOUT_COUNT;

    while (timeout > 0U) {
        if (DL_UART_Main_transmitDataCheck(LogUart_INST, data)) {
            return 1U;
        }
        --timeout;
    }
#else
    (void)data;
#endif
    return 0U;
}

void LogUart_Init(void)
{
#if CAR_ENABLE_LOG_UART
    g_logRxWriteIndex = 0U;
    g_logRxReadIndex = 0U;
    g_logRxDropCount = 0U;
    g_logRxErrorCount = 0U;
    g_logTxBusy = 0U;
#if CAR_ENABLE_LOG_UART_RX
    DL_UART_Main_setRXFIFOThreshold(LogUart_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(LogUart_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(LogUart_INST_INT_IRQN);
    NVIC_EnableIRQ(LogUart_INST_INT_IRQN);
#endif
#endif
}

void LogUart_Task(void)
{
}

void LogUart_HandleUARTInterrupt(void)
{
#if CAR_ENABLE_LOG_UART && CAR_ENABLE_LOG_UART_RX
    DL_UART_IIDX pending;
    uint8_t data;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;

    do {
        pending = DL_UART_Main_getPendingInterrupt(LogUart_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < LOG_UART_IRQ_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(LogUart_INST, &data)) {
                LogUart_PushRxByte(data);
                ++rxCount;
            }
        } else if ((pending == DL_UART_MAIN_IIDX_FRAMING_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_NOISE_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_OVERRUN_ERROR)) {
            ++g_logRxErrorCount;
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < LOG_UART_IRQ_SERVICE_LIMIT));
#endif
}

uint8_t LogUart_TryReadByte(uint8_t *data)
{
#if CAR_ENABLE_LOG_UART && CAR_ENABLE_LOG_UART_RX
    uint32_t primask;

    if (data == 0) {
        return 0U;
    }
    primask = LogUart_EnterCritical();
    if (g_logRxReadIndex == g_logRxWriteIndex) {
        LogUart_ExitCritical(primask);
        return 0U;
    }
    *data = g_logRxBuffer[g_logRxReadIndex];
    g_logRxReadIndex = (uint8_t)((g_logRxReadIndex + 1U) &
        LOG_UART_RX_BUFFER_MASK);
    LogUart_ExitCritical(primask);
    return 1U;
#else
    (void)data;
    return 0U;
#endif
}

void LogUart_ClearRx(void)
{
#if CAR_ENABLE_LOG_UART
    uint8_t data;
    uint8_t count = 0U;
    uint32_t primask = LogUart_EnterCritical();

    g_logRxWriteIndex = 0U;
    g_logRxReadIndex = 0U;
    g_logRxDropCount = 0U;
    g_logRxErrorCount = 0U;
#if CAR_ENABLE_LOG_UART_RX
    while ((count < LOG_UART_IRQ_RX_DRAIN_LIMIT) &&
        DL_UART_Main_receiveDataCheck(LogUart_INST, &data)) {
        ++count;
    }
#else
    (void)data;
    (void)count;
#endif
    LogUart_ExitCritical(primask);
#endif
}

uint32_t LogUart_GetRxDropCount(void)
{
    uint32_t value;
    uint32_t primask = LogUart_EnterCritical();

    value = g_logRxDropCount;
    LogUart_ExitCritical(primask);
    return value;
}

uint32_t LogUart_GetRxErrorCount(void)
{
    uint32_t value;
    uint32_t primask = LogUart_EnterCritical();

    value = g_logRxErrorCount;
    LogUart_ExitCritical(primask);
    return value;
}

void LogUart_SendByte(uint8_t data)
{
    if (LogUart_TryLockTx() != 0U) {
        (void)LogUart_TrySendByte(data);
        LogUart_UnlockTx();
    }
}

uint8_t LogUart_TrySendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint8_t success = 1U;

    if (data == 0) {
        return 0U;
    }
    if (LogUart_TryLockTx() == 0U) {
        return 0U;
    }

    for (i = 0U; i < length; ++i) {
        if (!LogUart_TrySendByte(data[i])) {
            success = 0U;
            break;
        }
    }
    LogUart_UnlockTx();
    return success;
}

void LogUart_SendBytes(const uint8_t *data, uint16_t length)
{
    (void)LogUart_TrySendBytes(data, length);
}

void LogUart_SendString(const char *text)
{
    if ((text == 0) || (LogUart_TryLockTx() == 0U)) {
        return;
    }
    while (*text != '\0') {
        if (!LogUart_TrySendByte((uint8_t)*text)) {
            break;
        }
        ++text;
    }
    LogUart_UnlockTx();
}

void LogUart_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    if (LogUart_TryLockTx() == 0U) {
        return;
    }
    do {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        ++count;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while (count > 0U) {
        --count;
        if (!LogUart_TrySendByte((uint8_t)digits[count])) {
            break;
        }
    }
    LogUart_UnlockTx();
}

void LogUart_SendSigned(int32_t value)
{
    char digits[10];
    uint8_t count = 0U;
    uint32_t magnitude;

    if (LogUart_TryLockTx() == 0U) {
        return;
    }
    if (value < 0) {
        if (!LogUart_TrySendByte((uint8_t)'-')) {
            LogUart_UnlockTx();
            return;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    do {
        digits[count] = (char)('0' + (magnitude % 10U));
        magnitude /= 10U;
        ++count;
    } while ((magnitude != 0U) && (count < (uint8_t)sizeof(digits)));
    while (count > 0U) {
        --count;
        if (!LogUart_TrySendByte((uint8_t)digits[count])) {
            break;
        }
    }
    LogUart_UnlockTx();
}

void LogUart_SendHex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    int8_t shift;

    if (LogUart_TryLockTx() == 0U) {
        return;
    }
    if (!LogUart_TrySendByte((uint8_t)'0') ||
        !LogUart_TrySendByte((uint8_t)'x')) {
        LogUart_UnlockTx();
        return;
    }
    for (shift = 28; shift >= 0; shift -= 4) {
        if (!LogUart_TrySendByte(
            (uint8_t)hex[(value >> (uint8_t)shift) & 0x0FU])) {
            break;
        }
    }
    LogUart_UnlockTx();
}
