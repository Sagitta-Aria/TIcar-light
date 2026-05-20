#ifndef JQ8400_H
#define JQ8400_H

#include <stdint.h>

void JQ8400_Init(void);
void JQ8400_Task(void);
void JQ8400_SendByte(uint8_t data);
void JQ8400_SendBytes(const uint8_t *data, uint16_t length);

#endif
