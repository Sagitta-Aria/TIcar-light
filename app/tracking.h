#ifndef TRACKING_H
#define TRACKING_H

#include <stdint.h>

void Tracking_Init(void);
void Tracking_Task(void);
void Tracking_SetEnabled(uint8_t enabled);
uint8_t Tracking_IsEnabled(void);

#endif
