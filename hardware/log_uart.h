#ifndef LOG_UART_H
#define LOG_UART_H

#include "board_config.h"
#include <stdint.h>

/* LogUart_Init：初始化 Type-C CH340 日志通道的软件状态。 */
void LogUart_Init(void);

/* LogUart_Task：日志串口后台任务入口，当前发送为同步有限等待。 */
void LogUart_Task(void);

/* LogUart_HandleUARTInterrupt：UART0 ISR入口，只缓存接收字节。 */
void LogUart_HandleUARTInterrupt(void);

/* LogUart_TryReadByte：从UART0接收环形缓冲取一个字节。 */
uint8_t LogUart_TryReadByte(uint8_t *data);

/* LogUart_ClearRx：清空软件接收缓冲和累计错误计数。 */
void LogUart_ClearRx(void);

uint32_t LogUart_GetRxDropCount(void);
uint32_t LogUart_GetRxErrorCount(void);

/* LogUart_SendByte：向 Type-C 日志串口发送 1 个字节，内部带超时。 */
void LogUart_SendByte(uint8_t data);

/* LogUart_SendBytes：向 Type-C 日志串口发送连续数据，内部带超时。 */
void LogUart_SendBytes(const uint8_t *data, uint16_t length);

/* LogUart_SendString：向 Type-C 日志串口发送 C 字符串，内部带超时。 */
void LogUart_SendString(const char *text);

/* LogUart_SendUnsigned：发送无符号十进制数，内部只使用小栈缓冲。 */
void LogUart_SendUnsigned(uint32_t value);

/* LogUart_SendSigned：发送有符号十进制数。 */
void LogUart_SendSigned(int32_t value);

/* LogUart_SendHex32：发送 0xXXXXXXXX 格式十六进制数。 */
void LogUart_SendHex32(uint32_t value);

/*
 * LOG_*：工程统一日志宏。
 * 使用场景：应用层、板级层记录行为事件。
 * 关闭方法：把 config/board_config.h 里的 CAR_ENABLE_LOG_UART 改成 0。
 */
#if CAR_ENABLE_LOG_UART
#define LOG_RAW(text) \
    do { LogUart_SendString((text)); } while (0)
#define LOG_LINE(text) \
    do { LogUart_SendString((text)); LogUart_SendString("\r\n"); } while (0)
#define LOG_U32(label, value) \
    do { LogUart_SendString((label)); LogUart_SendUnsigned((uint32_t)(value)); \
        LogUart_SendString("\r\n"); } while (0)
#define LOG_I32(label, value) \
    do { LogUart_SendString((label)); LogUart_SendSigned((int32_t)(value)); \
        LogUart_SendString("\r\n"); } while (0)
#define LOG_HEX32(label, value) \
    do { LogUart_SendString((label)); LogUart_SendHex32((uint32_t)(value)); \
        LogUart_SendString("\r\n"); } while (0)
#else
#define LOG_RAW(text) \
    do { (void)sizeof(text); } while (0)
#define LOG_LINE(text) \
    do { (void)sizeof(text); } while (0)
#define LOG_U32(label, value) \
    do { (void)sizeof(label); (void)sizeof(value); } while (0)
#define LOG_I32(label, value) \
    do { (void)sizeof(label); (void)sizeof(value); } while (0)
#define LOG_HEX32(label, value) \
    do { (void)sizeof(label); (void)sizeof(value); } while (0)
#endif

#endif
