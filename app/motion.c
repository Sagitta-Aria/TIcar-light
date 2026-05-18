#include "motion.h"

#include "motor.h"

static int16_t Motion_ToSignedDuty(uint16_t duty)
{
    return (duty > 32767U) ? 32767 : (int16_t)duty;
}

void Motion_Stop(void)
{
    Motor_Stop();
}

void Motion_SetSpeed(int16_t left, int16_t right)
{
    Motor_SetSpeed(left, right);
}

void Motion_Forward(uint16_t duty)
{
    int16_t speed = Motion_ToSignedDuty(duty);
    Motion_SetSpeed(speed, speed);
}

void Motion_Backward(uint16_t duty)
{
    int16_t speed = Motion_ToSignedDuty(duty);
    Motion_SetSpeed((int16_t)-speed, (int16_t)-speed);
}

void Motion_TurnLeft(uint16_t duty)
{
    int16_t speed = Motion_ToSignedDuty(duty);
    Motion_SetSpeed((int16_t)-speed, speed);
}

void Motion_TurnRight(uint16_t duty)
{
    int16_t speed = Motion_ToSignedDuty(duty);
    Motion_SetSpeed(speed, (int16_t)-speed);
}
