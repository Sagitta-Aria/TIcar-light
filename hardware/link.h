#ifndef LINK_H
#define LINK_H

#include <stdint.h>

void Link_Init(void);
void Link_Task(void);
void Link_SendByte(uint8_t data);
void Link_SendBytes(const uint8_t *data, uint16_t length);
void Link_SendString(const char *text);

#endif
