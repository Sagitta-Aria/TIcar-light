#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>

void JY61P_Init(void);
void JY61P_Task(void);
void JY61P_SendByte(uint8_t data);
void JY61P_SendBytes(const uint8_t *data, uint16_t length);

#endif
