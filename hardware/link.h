#ifndef LINK_H
#define LINK_H

#include <stdint.h>

void Link_Init(void);
void Link_Task(void);
void Link_HandleUARTInterrupt(void);
void Link_SendByte(uint8_t data);
void Link_SendBytes(const uint8_t *data, uint16_t length);
void Link_SendString(const char *text);
uint8_t Link_PopLine(char *buffer, uint16_t bufferSize);
void Link_ClearRx(void);
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
