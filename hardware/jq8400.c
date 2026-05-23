#include "jq8400.h"

#include "board_config.h"
#include "ti_msp_dl_config.h"

#if CAR_ENABLE_JQ8400

#define JQ8400_UART_TX_TIMEOUT_COUNT    (100000U)

/*
 * 作用：带超时发送语音模块串口数据。
 * 使用场景：播放提示音、后续语音业务命令。
 * 说明：语音模块未接或 UART 状态异常时不能卡死整车。
 */
static uint8_t JQ8400_TrySendByte(uint8_t data)
{
    uint32_t timeout = JQ8400_UART_TX_TIMEOUT_COUNT;

    while (timeout > 0U) {
        if (DL_UART_Main_transmitDataCheck(JQ8400_INST, data)) {
            return 1U;
        }
        --timeout;
    }
    return 0U;
}

void JQ8400_Init(void)
{
    NVIC_EnableIRQ(JQ8400_INST_INT_IRQN);
}

void JQ8400_Task(void)
{
}

void JQ8400_SendByte(uint8_t data)
{
    (void)JQ8400_TrySendByte(data);
}

void JQ8400_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        if (!JQ8400_TrySendByte(data[i])) {
            return;
        }
    }
}

#else

/*
 * 当前硬件资源分配中，UART1 PB6/PB7 已给 JY61P。
 * JQ8400 框架保留为空实现，避免误占串口或误初始化引脚。
 */
void JQ8400_Init(void)
{
}

void JQ8400_Task(void)
{
}

void JQ8400_SendByte(uint8_t data)
{
    (void)data;
}

void JQ8400_SendBytes(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
}

#endif
