#include "link.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

#define LINK_UART_TX_TIMEOUT_COUNT    (100000U)
#define LINK_IRQ_SERVICE_LIMIT        (16U)
#define LINK_IRQ_RX_DRAIN_LIMIT       (64U)
#define LINK_RX_LINE_SIZE             (64U)

static char g_linkRxLatest[LINK_RX_LINE_SIZE];
static volatile uint8_t g_linkRxLatestLength;
static volatile uint8_t g_linkRxHasLine;
static char g_linkRxBuild[LINK_RX_LINE_SIZE];
static volatile uint8_t g_linkRxBuildLength;
static volatile uint8_t g_linkRxDiscardingLine;
static volatile uint32_t g_linkRxLineCount;
static volatile uint32_t g_linkRxOverwriteCount;
static volatile uint32_t g_linkRxLongLineDropCount;
static volatile uint32_t g_linkRxByteCount;
static volatile uint8_t g_linkRxLastByte;
static volatile uint32_t g_linkRxPinChangeCount;
static volatile uint8_t g_linkRxPinLevel;
static volatile uint32_t g_linkRxErrorCount;
static volatile uint32_t g_linkRxFrameErrorCount;
static volatile uint32_t g_linkRxNoiseErrorCount;

/* 作用：读取 PB3/UART3 RX 引脚当前原始电平。 */
static uint8_t Link_ReadRxPinLevel(void)
{
    return ((DL_GPIO_readPins(GPIO_Exchange_RX_PORT,
        GPIO_Exchange_RX_PIN) & GPIO_Exchange_RX_PIN) != 0U) ? 1U : 0U;
}

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
    g_linkRxLatestLength = 0U;
    g_linkRxHasLine = 0U;
    g_linkRxBuildLength = 0U;
    g_linkRxDiscardingLine = 0U;
    g_linkRxLineCount = 0U;
    g_linkRxOverwriteCount = 0U;
    g_linkRxLongLineDropCount = 0U;
    g_linkRxByteCount = 0U;
    g_linkRxLastByte = 0U;
    g_linkRxPinChangeCount = 0U;
    g_linkRxPinLevel = Link_ReadRxPinLevel();
    g_linkRxErrorCount = 0U;
    g_linkRxFrameErrorCount = 0U;
    g_linkRxNoiseErrorCount = 0U;
}

static uint8_t Link_FinishRxLine(void)
{
    uint8_t i;

    if (g_linkRxBuildLength == 0U) {
        return 0U;
    }

    if (g_linkRxHasLine != 0U) {
        ++g_linkRxOverwriteCount;
    }

    for (i = 0U; i < g_linkRxBuildLength; ++i) {
        g_linkRxLatest[i] = g_linkRxBuild[i];
    }
    g_linkRxLatest[g_linkRxBuildLength] = '\0';
    g_linkRxLatestLength = g_linkRxBuildLength;
    g_linkRxHasLine = 1U;

    ++g_linkRxLineCount;
    g_linkRxBuildLength = 0U;
    return 1U;
}

/*
 * 作用：在 UART3 中断里拼接一行视觉数据。
 * 说明：以 '\n' 或 '\r' 结束一帧，超长行直接丢弃，避免阻塞和越界。
 */
static uint8_t Link_ParseRxByte(uint8_t data)
{
    ++g_linkRxByteCount;
    g_linkRxLastByte = data;

    if ((data == '\n') || (data == '\r')) {
        if (g_linkRxDiscardingLine != 0U) {
            g_linkRxDiscardingLine = 0U;
            g_linkRxBuildLength = 0U;
            return 0U;
        }
        return Link_FinishRxLine();
    }

    if (g_linkRxDiscardingLine != 0U) {
        return 0U;
    }

    if (g_linkRxBuildLength >= (uint8_t)(LINK_RX_LINE_SIZE - 1U)) {
        g_linkRxBuildLength = 0U;
        g_linkRxDiscardingLine = 1U;
        ++g_linkRxLongLineDropCount;
        return 0U;
    }

    g_linkRxBuild[g_linkRxBuildLength] = (char)data;
    ++g_linkRxBuildLength;
    return 0U;
}

void Link_Init(void)
{
    Link_ResetRxState();
    DL_UART_Main_setRXFIFOThreshold(
        Exchange_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(Exchange_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(Exchange_INST_INT_IRQN);
    NVIC_EnableIRQ(Exchange_INST_INT_IRQN);
}

void Link_Task(void)
{
    uint8_t level = Link_ReadRxPinLevel();

    if (level != g_linkRxPinLevel) {
        g_linkRxPinLevel = level;
        ++g_linkRxPinChangeCount;
    }
}

uint8_t Link_HandleUARTInterrupt(void)
{
    uint8_t data;
    DL_UART_IIDX pending;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;
    uint8_t lineCompleted = 0U;

    do {
        pending = DL_UART_Main_getPendingInterrupt(Exchange_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < LINK_IRQ_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(Exchange_INST, &data)) {
                if (Link_ParseRxByte(data) != 0U) {
                    lineCompleted = 1U;
                }
                ++rxCount;
            }
        } else if (pending == DL_UART_MAIN_IIDX_FRAMING_ERROR) {
            ++g_linkRxErrorCount;
            ++g_linkRxFrameErrorCount;
        } else if (pending == DL_UART_MAIN_IIDX_NOISE_ERROR) {
            ++g_linkRxErrorCount;
            ++g_linkRxNoiseErrorCount;
        } else if (pending == DL_UART_MAIN_IIDX_OVERRUN_ERROR) {
            ++g_linkRxErrorCount;
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < LINK_IRQ_SERVICE_LIMIT));
    return lineCompleted;
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
    uint8_t length;
    uint8_t i;
    uint32_t primask;

    if ((buffer == 0) || (bufferSize == 0U)) {
        return 0U;
    }

    primask = Link_EnterCritical();
    if (g_linkRxHasLine == 0U) {
        Link_ExitCritical(primask);
        buffer[0] = '\0';
        return 0U;
    }

    length = g_linkRxLatestLength;
    if (length >= bufferSize) {
        length = (uint8_t)(bufferSize - 1U);
    }
    for (i = 0U; i < length; ++i) {
        buffer[i] = g_linkRxLatest[i];
    }
    buffer[length] = '\0';

    g_linkRxLatestLength = 0U;
    g_linkRxHasLine = 0U;
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
    return g_linkRxLongLineDropCount;
}

uint32_t Link_GetRxOverwriteCount(void)
{
    return g_linkRxOverwriteCount;
}

uint32_t Link_GetRxLongLineDropCount(void)
{
    return g_linkRxLongLineDropCount;
}

uint32_t Link_GetRxByteCount(void)
{
    return g_linkRxByteCount;
}

uint8_t Link_GetRxLastByte(void)
{
    return g_linkRxLastByte;
}

uint8_t Link_GetRxBuildLength(void)
{
    return g_linkRxBuildLength;
}

uint8_t Link_GetRxPinLevel(void)
{
    return g_linkRxPinLevel;
}

uint32_t Link_GetRxPinChangeCount(void)
{
    return g_linkRxPinChangeCount;
}

uint32_t Link_GetRxErrorCount(void)
{
    return g_linkRxErrorCount;
}

uint32_t Link_GetRxFrameErrorCount(void)
{
    return g_linkRxFrameErrorCount;
}

uint32_t Link_GetRxNoiseErrorCount(void)
{
    return g_linkRxNoiseErrorCount;
}
