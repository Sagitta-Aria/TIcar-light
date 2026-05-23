#include "link.h"

#include "ti_msp_dl_config.h"

#define LINK_UART_TX_TIMEOUT_COUNT    (100000U)

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

void Link_Init(void)
{
    NVIC_EnableIRQ(Exchange_INST_INT_IRQN);
}

void Link_Task(void)
{
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
