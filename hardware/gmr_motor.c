#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "motor.h"

#include "encoder_motor.h"
#include "motor_enable.h"

static uint8_t Motor_IsChassis(MotorId motor)
{
    return (uint8_t)(((motor == MOTOR_CHASSIS_LEFT) ||
        (motor == MOTOR_CHASSIS_RIGHT)) ? 1U : 0U);
}

static int16_t Motor_SignedCommand(MotorDir dir, uint16_t speed)
{
    int16_t value = (speed > 32767U) ? 32767 : (int16_t)speed;

    return (dir == MOTOR_REVERSE) ? (int16_t)-value : value;
}

void Motor_Init(void)
{
    MotorEnable_Init();
    EncoderMotor_Init();
    EncoderMotor_Stop();
}

void Motor_Task(void)
{
}

void Motor_RunChassisControl(void)
{
    EncoderMotor_RunControlPeriod();
}

void Motor_Set(MotorId motor, MotorDir dir, uint16_t speedSps)
{
    if (Motor_IsChassis(motor) == 0U) {
        return;
    }
    EncoderMotor_SetTarget((uint8_t)motor,
        ((dir == MOTOR_COAST) || (dir == MOTOR_BRAKE)) ? 0 :
            Motor_SignedCommand(dir, speedSps));
}

void Motor_MoveSteps(MotorId motor, MotorDir dir, uint16_t speedSps,
    uint32_t stepCount)
{
    (void)motor;
    (void)dir;
    (void)speedSps;
    (void)stepCount;
}

uint8_t Motor_IsStepMoveActive(MotorId motor)
{
    (void)motor;
    return 0U;
}

void Motor_SetRampStep(MotorId motor, uint16_t accelStepSps,
    uint16_t decelStepSps)
{
    (void)motor;
    (void)accelStepSps;
    (void)decelStepSps;
}

void Motor_ResetRampStep(MotorId motor)
{
    (void)motor;
}

void Motor_SetChassisCommand(int16_t leftCps, int16_t rightCps)
{
    EncoderMotor_SetTargets(leftCps, rightCps);
}

void Motor_SetChassisPeriodCommand(int16_t leftCounts, int16_t rightCounts)
{
    EncoderMotor_SetPeriodTargets(leftCounts, rightCounts);
}

void Motor_SetChassisCrossCoupledPwm(int16_t leftPwm, int16_t rightPwm,
    int32_t syncGainQ1024, uint16_t syncLimitPwm)
{
    EncoderMotor_SetCrossCoupledPwm(leftPwm, rightPwm, syncGainQ1024,
        syncLimitPwm);
}

void Motor_SetChassisZeroTargetBrake(uint8_t leftEnabled,
    uint8_t rightEnabled)
{
    EncoderMotor_SetZeroTargetBrake(ENCODER_MOTOR_LEFT, leftEnabled);
    EncoderMotor_SetZeroTargetBrake(ENCODER_MOTOR_RIGHT, rightEnabled);
}

void Motor_SetAllStop(void)
{
    EncoderMotor_Stop();
}

void Motor_Stop(void)
{
    EncoderMotor_Stop();
}

int16_t Motor_GetCommand(MotorId motor)
{
    return (Motor_IsChassis(motor) != 0U) ?
        EncoderMotor_GetTarget((uint8_t)motor) : 0;
}

int32_t Motor_GetStepCount(MotorId motor)
{
    return (Motor_IsChassis(motor) != 0U) ?
        EncoderMotor_GetTotalCount((uint8_t)motor) : 0;
}

int16_t Motor_GetGimbalStepRate(MotorId motor)
{
    (void)motor;
    return 0;
}

void Motor_ResetStepCount(MotorId motor)
{
    if (Motor_IsChassis(motor) != 0U) {
        EncoderMotor_ResetTotalCount((uint8_t)motor);
    }
}

void Motor_ResetAllStepCounts(void)
{
    EncoderMotor_ResetAllCounts();
}

#endif
