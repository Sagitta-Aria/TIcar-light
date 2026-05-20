#include "motion.h"

#include "board_config.h"
#include "motor.h"
#include "speed_control.h"

static int16_t Motion_ToSignedDuty(uint16_t duty)
{
    return (duty > 32767U) ? 32767 : (int16_t)duty;
}

void Motion_Stop(void)
{
#if CAR_ENABLE_SPEED_CONTROL
    SpeedControl_Stop();
#else
    Motor_Stop();
#endif
}

void Motion_SetSpeed(int16_t left, int16_t right)
{
#if CAR_ENABLE_SPEED_CONTROL
    SpeedControl_SetTarget(left, right);
#else
    Motor_SetSpeed(left, right);
#endif
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
