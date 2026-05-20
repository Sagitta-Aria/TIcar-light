#include "link.h"

#include "ti_msp_dl_config.h"

void Link_Init(void)
{
    NVIC_EnableIRQ(Exchange_INST_INT_IRQN);
}

void Link_Task(void)
{
}

void Link_SendByte(uint8_t data)
{
    DL_UART_transmitDataBlocking(Exchange_INST, data);
}

void Link_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        Link_SendByte(data[i]);
    }
}

void Link_SendString(const char *text)
{
    while ((text != 0) && (*text != '\0')) {
        Link_SendByte((uint8_t)*text);
        ++text;
    }
}
