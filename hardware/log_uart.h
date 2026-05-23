#ifndef LOG_UART_H
#define LOG_UART_H

#include <stdint.h>

/* LogUart_Init：初始化 Type-C CH340 日志通道的软件状态。 */
void LogUart_Init(void);

/* LogUart_Task：日志串口后台任务入口，当前发送为同步有限等待。 */
void LogUart_Task(void);

/* LogUart_SendByte：向 Type-C 日志串口发送 1 个字节，内部带超时。 */
void LogUart_SendByte(uint8_t data);

/* LogUart_SendBytes：向 Type-C 日志串口发送连续数据，内部带超时。 */
void LogUart_SendBytes(const uint8_t *data, uint16_t length);

/* LogUart_SendString：向 Type-C 日志串口发送 C 字符串，内部带超时。 */
void LogUart_SendString(const char *text);

#endif
