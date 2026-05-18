#include "jy61p.h"

#include "ti_msp_dl_config.h"

void JY61P_Init(void)
{
    NVIC_EnableIRQ(JY61P_INST_INT_IRQN);
}

void JY61P_Task(void)
{
}

void JY61P_SendByte(uint8_t data)
{
    DL_UART_transmitDataBlocking(JY61P_INST, data);
}

void JY61P_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        JY61P_SendByte(data[i]);
    }
}
