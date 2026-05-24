#include "link.h"

#include "ti_msp_dl_config.h"

#define LINK_UART_TX_TIMEOUT_COUNT    (100000U)
#define LINK_IRQ_SERVICE_LIMIT        (16U)
#define LINK_IRQ_RX_DRAIN_LIMIT       (64U)
#define LINK_RX_LINE_SIZE             (64U)
#define LINK_RX_LINE_QUEUE_COUNT      (4U)

static char g_linkRxLines[LINK_RX_LINE_QUEUE_COUNT][LINK_RX_LINE_SIZE];
static volatile uint8_t g_linkRxLengths[LINK_RX_LINE_QUEUE_COUNT];
static volatile uint8_t g_linkRxReadIndex;
static volatile uint8_t g_linkRxWriteIndex;
static volatile uint8_t g_linkRxCount;
static char g_linkRxBuild[LINK_RX_LINE_SIZE];
static volatile uint8_t g_linkRxBuildLength;
static volatile uint32_t g_linkRxLineCount;
static volatile uint32_t g_linkRxDropCount;

static uint32_t Link_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Link_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/*
 * 作用：带超时发送 1 字节到视觉模块串口。
 * 使用场景：后续 Exchange/视觉模块协议发送。
 * 说明：不能使用永久阻塞发送，避免串口时钟或 FIFO 异常时卡死主流程。
 */
static uint8_t Link_TrySendByte(uint8_t data)
{
    uint32_t timeout = LINK_UART_TX_TIMEOUT_COUNT;

    while (timeout > 0U) {
        if (DL_UART_Main_transmitDataCheck(Exchange_INST, data)) {
            return 1U;
        }
        --timeout;
    }
    return 0U;
}

static void Link_ResetRxState(void)
{
    uint8_t i;

    g_linkRxReadIndex = 0U;
    g_linkRxWriteIndex = 0U;
    g_linkRxCount = 0U;
    g_linkRxBuildLength = 0U;
    g_linkRxLineCount = 0U;
    g_linkRxDropCount = 0U;
    for (i = 0U; i < LINK_RX_LINE_QUEUE_COUNT; ++i) {
        g_linkRxLengths[i] = 0U;
    }
}

static void Link_FinishRxLine(void)
{
    uint8_t i;
    uint8_t writeIndex;

    if (g_linkRxBuildLength == 0U) {
        return;
    }

    if (g_linkRxCount >= LINK_RX_LINE_QUEUE_COUNT) {
        g_linkRxBuildLength = 0U;
        ++g_linkRxDropCount;
        return;
    }

    writeIndex = g_linkRxWriteIndex;
    for (i = 0U; i < g_linkRxBuildLength; ++i) {
        g_linkRxLines[writeIndex][i] = g_linkRxBuild[i];
    }
    g_linkRxLines[writeIndex][g_linkRxBuildLength] = '\0';
    g_linkRxLengths[writeIndex] = g_linkRxBuildLength;

    g_linkRxWriteIndex = (uint8_t)((g_linkRxWriteIndex + 1U) %
        LINK_RX_LINE_QUEUE_COUNT);
    ++g_linkRxCount;
    ++g_linkRxLineCount;
    g_linkRxBuildLength = 0U;
}

/*
 * 作用：在 UART3 中断里拼接一行视觉数据。
 * 说明：以 '\n' 或 '\r' 结束一帧，超长行直接丢弃，避免阻塞和越界。
 */
static void Link_ParseRxByte(uint8_t data)
{
    if ((data == '\n') || (data == '\r')) {
        Link_FinishRxLine();
        return;
    }

    if (g_linkRxBuildLength >= (uint8_t)(LINK_RX_LINE_SIZE - 1U)) {
        g_linkRxBuildLength = 0U;
        ++g_linkRxDropCount;
        return;
    }

    g_linkRxBuild[g_linkRxBuildLength] = (char)data;
    ++g_linkRxBuildLength;
}

void Link_Init(void)
{
    Link_ResetRxState();
    DL_UART_Main_setRXFIFOThreshold(
        Exchange_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(Exchange_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(Exchange_INST_INT_IRQN);
    NVIC_EnableIRQ(Exchange_INST_INT_IRQN);
}

void Link_Task(void)
{
}

void Link_HandleUARTInterrupt(void)
{
    uint8_t data;
    DL_UART_IIDX pending;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;

    do {
        pending = DL_UART_Main_getPendingInterrupt(Exchange_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < LINK_IRQ_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(Exchange_INST, &data)) {
                Link_ParseRxByte(data);
                ++rxCount;
            }
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < LINK_IRQ_SERVICE_LIMIT));
}

void Link_SendByte(uint8_t data)
{
    (void)Link_TrySendByte(data);
}

void Link_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        if (!Link_TrySendByte(data[i])) {
            return;
        }
    }
}

void Link_SendString(const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        if (!Link_TrySendByte((uint8_t)*text)) {
            return;
        }
        ++text;
    }
}

uint8_t Link_PopLine(char *buffer, uint16_t bufferSize)
{
    uint8_t readIndex;
    uint8_t length;
    uint8_t i;
    uint32_t primask;

    if ((buffer == 0) || (bufferSize == 0U)) {
        return 0U;
    }

    primask = Link_EnterCritical();
    if (g_linkRxCount == 0U) {
        Link_ExitCritical(primask);
        buffer[0] = '\0';
        return 0U;
    }

    readIndex = g_linkRxReadIndex;
    length = g_linkRxLengths[readIndex];
    if (length >= bufferSize) {
        length = (uint8_t)(bufferSize - 1U);
    }
    for (i = 0U; i < length; ++i) {
        buffer[i] = g_linkRxLines[readIndex][i];
    }
    buffer[length] = '\0';

    g_linkRxLengths[readIndex] = 0U;
    g_linkRxReadIndex = (uint8_t)((g_linkRxReadIndex + 1U) %
        LINK_RX_LINE_QUEUE_COUNT);
    --g_linkRxCount;
    Link_ExitCritical(primask);
    return 1U;
}

void Link_ClearRx(void)
{
    uint32_t primask = Link_EnterCritical();
    Link_ResetRxState();
    Link_ExitCritical(primask);
}

uint32_t Link_GetRxLineCount(void)
{
    return g_linkRxLineCount;
}

uint32_t Link_GetRxDropCount(void)
{
    return g_linkRxDropCount;
}
