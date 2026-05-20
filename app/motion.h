#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

void Motion_Stop(void);
void Motion_SetSpeed(int16_t left, int16_t right);
void Motion_Forward(uint16_t duty);
void Motion_Backward(uint16_t duty);
void Motion_TurnLeft(uint16_t duty);
void Motion_TurnRight(uint16_t duty);

#endif
