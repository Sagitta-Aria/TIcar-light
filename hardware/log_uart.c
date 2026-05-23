#include "log_uart.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

#define LOG_UART_TX_TIMEOUT_COUNT    (100000U)

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
    NVIC_ClearPendingIRQ(LogUart_INST_INT_IRQN);
#endif
}

void LogUart_Task(void)
{
}

void LogUart_SendByte(uint8_t data)
{
    (void)LogUart_TrySendByte(data);
}

void LogUart_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }

    for (i = 0U; i < length; ++i) {
        if (!LogUart_TrySendByte(data[i])) {
            return;
        }
    }
}

void LogUart_SendString(const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        if (!LogUart_TrySendByte((uint8_t)*text)) {
            return;
        }
        ++text;
    }
}

void LogUart_SendUnsigned(uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        ++count;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while (count > 0U) {
        --count;
        if (!LogUart_TrySendByte((uint8_t)digits[count])) {
            return;
        }
    }
}

void LogUart_SendSigned(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (!LogUart_TrySendByte((uint8_t)'-')) {
            return;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }

    LogUart_SendUnsigned(magnitude);
}

void LogUart_SendHex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    int8_t shift;

    LogUart_SendString("0x");
    for (shift = 28; shift >= 0; shift -= 4) {
        if (!LogUart_TrySendByte(
            (uint8_t)hex[(value >> (uint8_t)shift) & 0x0FU])) {
            return;
        }
    }
}
