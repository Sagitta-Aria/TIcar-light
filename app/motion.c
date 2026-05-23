#include "motion.h"

#include "motor.h"

static int16_t Motion_ToSignedCommand(uint16_t command)
{
    return (command > 32767U) ? 32767 : (int16_t)command;
}

void Motion_Stop(void)
{
    Motor_Stop();
}

void Motion_SetChassisCommand(int16_t leftCommand, int16_t rightCommand)
{
    Motor_SetChassisCommand(leftCommand, rightCommand);
}

void Motion_Forward(uint16_t command)
{
    int16_t speed = Motion_ToSignedCommand(command);
    Motion_SetChassisCommand(speed, speed);
}

void Motion_Backward(uint16_t command)
{
    int16_t speed = Motion_ToSignedCommand(command);
    Motion_SetChassisCommand((int16_t)-speed, (int16_t)-speed);
}

void Motion_TurnLeft(uint16_t command)
{
    int16_t speed = Motion_ToSignedCommand(command);
    Motion_SetChassisCommand((int16_t)-speed, speed);
}

void Motion_TurnRight(uint16_t command)
{
    int16_t speed = Motion_ToSignedCommand(command);
    Motion_SetChassisCommand(speed, (int16_t)-speed);
}
