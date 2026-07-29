#ifndef H7_CONTROL_UART_H
#define H7_CONTROL_UART_H

#include <stdint.h>

typedef enum {
    H7_CONTROL_COMMAND_NONE = 0,
    H7_CONTROL_COMMAND_START_ATTITUDE,
    H7_CONTROL_COMMAND_STOP
} H7ControlCommand;

/* Configure Tianmeng UART2/PB15-PB16 for the H7 LCD/control link. */
void H7ControlUart_Init(void);

/* Send one complete LCD command without interleaving another sender. */
uint8_t H7ControlUart_TrySendBytes(const uint8_t *data, uint16_t length);

/* Return and clear the latest complete H7 control command. */
H7ControlCommand H7ControlUart_TakeCommand(void);

/* Drain UART2 RX in interrupt context and parse line-oriented commands. */
void H7ControlUart_HandleUARTInterrupt(void);

#endif
