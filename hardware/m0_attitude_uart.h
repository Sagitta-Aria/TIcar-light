#ifndef M0_ATTITUDE_UART_H
#define M0_ATTITUDE_UART_H

#include <stdint.h>

/* 初始化GMR外部M0姿态串口的RX中断；UART实例和引脚由resource_config.h映射。 */
void M0AttitudeUart_Init(void);

/* 每5ms调用；上电500ms后通过PB2发送一次CY-Z 100Hz上报命令。 */
void M0AttitudeUart_Task(uint32_t elapsedMs);

/* UART3中断入口调用；只排空RX FIFO并把字节交给m0_attitude_link解析器。 */
void M0AttitudeUart_HandleUARTInterrupt(void);

#endif
