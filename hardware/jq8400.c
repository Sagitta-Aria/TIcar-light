#include "jq8400.h"

#include "ti_msp_dl_config.h"

void JQ8400_Init(void)
{
    NVIC_EnableIRQ(JQ8400_INST_INT_IRQN);
}

void JQ8400_Task(void)
{
}

void JQ8400_SendByte(uint8_t data)
{
    DL_UART_transmitDataBlocking(JQ8400_INST, data);
}

void JQ8400_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        JQ8400_SendByte(data[i]);
    }
}
