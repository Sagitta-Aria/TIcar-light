#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

void Motion_Stop(void);
void Motion_SetChassisCommand(int16_t leftCommand, int16_t rightCommand);
void Motion_Forward(uint16_t command);
void Motion_Backward(uint16_t command);
void Motion_TurnLeft(uint16_t command);
void Motion_TurnRight(uint16_t command);

#endif
