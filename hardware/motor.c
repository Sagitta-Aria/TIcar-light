#include "motor.h"

#include "board_config.h"
#include "pin_map.h"

static uint16_t Motor_ClampDuty(uint16_t duty)
{
    return (duty > CAR_MOTOR_PWM_MAX_COUNTS) ? CAR_MOTOR_PWM_MAX_COUNTS : duty;
}

static uint16_t Motor_DutyFromSigned(int16_t speed)
{
    if (speed < 0) {
        speed = (int16_t)(-speed);
    }
    return Motor_ClampDuty((uint16_t)speed);
}

static void Motor_SetDirPins(uint32_t in1, uint32_t in2, MotorDir dir)
{
    switch (dir) {
    case MOTOR_FORWARD:
        DL_GPIO_setPins(PIN_MOTOR_DIR_PORT, in1);
        DL_GPIO_clearPins(PIN_MOTOR_DIR_PORT, in2);
        break;
    case MOTOR_REVERSE:
        DL_GPIO_clearPins(PIN_MOTOR_DIR_PORT, in1);
        DL_GPIO_setPins(PIN_MOTOR_DIR_PORT, in2);
        break;
    case MOTOR_BRAKE:
        DL_GPIO_setPins(PIN_MOTOR_DIR_PORT, in1 | in2);
        break;
    case MOTOR_COAST:
    default:
        DL_GPIO_clearPins(PIN_MOTOR_DIR_PORT, in1 | in2);
        break;
    }
}

void Motor_Init(void)
{
    Motor_Stop();
    DL_Timer_startCounter(PIN_MOTOR_PWM_TIMER);
}

void Motor_Set(MotorId motor, MotorDir dir, uint16_t duty)
{
    duty = Motor_ClampDuty(duty);

    if (motor == MOTOR_LEFT) {
        Motor_SetDirPins(PIN_MOTOR_LEFT_IN1, PIN_MOTOR_LEFT_IN2, dir);
        DL_Timer_setCaptureCompareValue(
            PIN_MOTOR_PWM_TIMER, duty, PIN_MOTOR_LEFT_PWM_CC);
    } else {
        Motor_SetDirPins(PIN_MOTOR_RIGHT_IN1, PIN_MOTOR_RIGHT_IN2, dir);
        DL_Timer_setCaptureCompareValue(
            PIN_MOTOR_PWM_TIMER, duty, PIN_MOTOR_RIGHT_PWM_CC);
    }
}

void Motor_SetSpeed(int16_t left, int16_t right)
{
    Motor_Set(MOTOR_LEFT,
        (left >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE,
        Motor_DutyFromSigned(left));
    Motor_Set(MOTOR_RIGHT,
        (right >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE,
        Motor_DutyFromSigned(right));
}

void Motor_Stop(void)
{
    Motor_Set(MOTOR_LEFT, MOTOR_COAST, 0U);
    Motor_Set(MOTOR_RIGHT, MOTOR_COAST, 0U);
}
