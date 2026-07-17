#include "motor.h"

#include "board_config.h"
#include "encoder_motor.h"
#include "motor_enable.h"
#include "pin_map.h"
#include "stepper_pulse.h"

typedef struct {
    GPIO_Regs *dirPort;
    uint32_t dirPin;
    uint16_t speedSps;
    int8_t directionSign;
} GimbalStepper;

static GimbalStepper g_gimbalMotors[2] = {
    { PIN_STEPPER_GIMBAL_YAW_DIR_PORT, PIN_STEPPER_GIMBAL_YAW_DIR, 0U, 1 },
    { PIN_STEPPER_GIMBAL_PITCH_DIR_PORT, PIN_STEPPER_GIMBAL_PITCH_DIR,
        0U, 1 }
};

static uint8_t Motor_IsValid(MotorId motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

static uint8_t Motor_IsChassis(MotorId motor)
{
    return ((motor == MOTOR_CHASSIS_LEFT) ||
        (motor == MOTOR_CHASSIS_RIGHT)) ? 1U : 0U;
}

static uint16_t Motor_ClampStepperSpeed(uint16_t speedSps)
{
    return (speedSps > CAR_STEPPER_SPEED_MAX_SPS) ?
        CAR_STEPPER_SPEED_MAX_SPS : speedSps;
}

static int16_t Motor_SignedCommand(MotorDir dir, uint16_t speed)
{
    int32_t value = (speed > 32767U) ? 32767 : (int32_t)speed;
    return (dir == MOTOR_REVERSE) ? (int16_t)-value : (int16_t)value;
}

static GimbalStepper *Motor_GetGimbal(MotorId motor)
{
    return &g_gimbalMotors[(uint32_t)motor - (uint32_t)MOTOR_GIMBAL_1];
}

static void Motor_SetGimbal(MotorId motor, MotorDir dir, uint16_t speedSps)
{
    GimbalStepper *stepper = Motor_GetGimbal(motor);
    int8_t directionSign;

    if ((dir == MOTOR_COAST) || (dir == MOTOR_BRAKE) || (speedSps == 0U)) {
        stepper->speedSps = 0U;
        StepperPulse_SetTarget(motor, stepper->directionSign, 0U);
        return;
    }

    directionSign = (dir == MOTOR_REVERSE) ? -1 : 1;
    if (directionSign != stepper->directionSign) {
        StepperPulse_SetTarget(motor, stepper->directionSign, 0U);
    }
    if (dir == MOTOR_REVERSE) {
        DL_GPIO_setPins(stepper->dirPort, stepper->dirPin);
    } else {
        DL_GPIO_clearPins(stepper->dirPort, stepper->dirPin);
    }
    stepper->directionSign = directionSign;
    stepper->speedSps = Motor_ClampStepperSpeed(speedSps);
    StepperPulse_SetTarget(motor, stepper->directionSign, stepper->speedSps);
}

void Motor_Init(void)
{
    MotorEnable_Init();
    EncoderMotor_Init();
    StepperPulse_Init();
    Motor_SetAllStop();
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
    if (Motor_IsValid(motor) == 0U) {
        return;
    }
    if (Motor_IsChassis(motor) != 0U) {
        EncoderMotor_SetTarget((uint8_t)motor,
            ((dir == MOTOR_COAST) || (dir == MOTOR_BRAKE)) ? 0 :
                Motor_SignedCommand(dir, speedSps));
        return;
    }
    Motor_SetGimbal(motor, dir, speedSps);
}

void Motor_SetRampStep(MotorId motor, uint16_t accelStepSps,
    uint16_t decelStepSps)
{
    if ((motor == MOTOR_GIMBAL_1) || (motor == MOTOR_GIMBAL_2)) {
        StepperPulse_SetRampStep(motor, accelStepSps, decelStepSps);
    }
}

void Motor_ResetRampStep(MotorId motor)
{
    Motor_SetRampStep(motor, (uint16_t)CAR_STEPPER_ACCEL_STEP_SPS,
        (uint16_t)CAR_STEPPER_DECEL_STEP_SPS);
}

void Motor_SetChassisCommand(int16_t leftCps, int16_t rightCps)
{
    EncoderMotor_SetTargets(leftCps, rightCps);
}

void Motor_SetChassisPeriodCommand(int16_t leftCounts, int16_t rightCounts)
{
    EncoderMotor_SetPeriodTargets(leftCounts, rightCounts);
}

void Motor_SetAllStop(void)
{
    uint8_t index;

    EncoderMotor_Stop();
    for (index = 0U; index < 2U; ++index) {
        g_gimbalMotors[index].speedSps = 0U;
        g_gimbalMotors[index].directionSign = 1;
        DL_GPIO_clearPins(g_gimbalMotors[index].dirPort,
            g_gimbalMotors[index].dirPin);
    }
    StepperPulse_StopAll();
}

void Motor_Stop(void)
{
    Motor_SetAllStop();
}

int16_t Motor_GetCommand(MotorId motor)
{
    GimbalStepper *stepper;

    if (Motor_IsValid(motor) == 0U) {
        return 0;
    }
    if (Motor_IsChassis(motor) != 0U) {
        return EncoderMotor_GetTarget((uint8_t)motor);
    }
    stepper = Motor_GetGimbal(motor);
    return (stepper->directionSign >= 0) ? (int16_t)stepper->speedSps :
        (int16_t)(-(int32_t)stepper->speedSps);
}

int32_t Motor_GetStepCount(MotorId motor)
{
    if (Motor_IsValid(motor) == 0U) {
        return 0;
    }
    if (Motor_IsChassis(motor) != 0U) {
        return EncoderMotor_GetTotalCount((uint8_t)motor);
    }
    return StepperPulse_GetStepCount(motor);
}

void Motor_ResetStepCount(MotorId motor)
{
    if (Motor_IsValid(motor) == 0U) {
        return;
    }
    if (Motor_IsChassis(motor) != 0U) {
        EncoderMotor_ResetTotalCount((uint8_t)motor);
    } else {
        StepperPulse_ResetStepCount(motor);
    }
}

void Motor_ResetAllStepCounts(void)
{
    EncoderMotor_ResetAllCounts();
    StepperPulse_ResetAllStepCounts();
}
