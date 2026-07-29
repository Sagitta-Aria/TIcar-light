#ifndef LINK_H
#define LINK_H

#include <stdint.h>

/*
 * Link：K230视觉UART3行缓冲层。
 * ISR只收字节并在换行时提交完整帧；Vision任务再解析字段。
 * 本模块不是UART0日志口，也不能用于H7姿态帧。
 */

/* 清空UART3收发统计和行缓冲，打开接收中断。 */
void Link_Init(void);

/* 任务上下文低频维护入口；当前协议主要由中断接收驱动。 */
void Link_Task(void);

/* UART3 ISR入口；完整行形成时返回1，用于唤醒Gimbal任务。 */
uint8_t Link_HandleUARTInterrupt(void);

/* 同步发送接口，供给K230发送单字节/字节串/字符串命令；均带有限等待。 */
void Link_SendByte(uint8_t data);
void Link_SendBytes(const uint8_t *data, uint16_t length);
void Link_SendString(const char *text);

/* 弹出最新完整视觉行；bufferSize含结尾'\0'，成功返回1。 */
uint8_t Link_PopLine(char *buffer, uint16_t bufferSize);

/* 丢弃正在拼接和已经完成的接收行，同时清零诊断统计。 */
void Link_ClearRx(void);

/* 以下接口只读UART3诊断计数，不会清零或消费接收数据。 */
uint32_t Link_GetRxLineCount(void);
uint32_t Link_GetRxDropCount(void);
uint32_t Link_GetRxOverwriteCount(void);
uint32_t Link_GetRxLongLineDropCount(void);
uint32_t Link_GetRxByteCount(void);
uint8_t Link_GetRxLastByte(void);
uint8_t Link_GetRxBuildLength(void);
uint8_t Link_GetRxPinLevel(void);
uint32_t Link_GetRxPinChangeCount(void);
uint32_t Link_GetRxErrorCount(void);
uint32_t Link_GetRxFrameErrorCount(void);
uint32_t Link_GetRxNoiseErrorCount(void);

#endif
