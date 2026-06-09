#include "motor.h"

#include "board_config.h"
#include "motor_enable.h"
#include "pin_map.h"
#include "stepper_pulse.h"

typedef struct {
    GPIO_Regs *dirPort;
    uint32_t dirPin;
    uint16_t speedSps;
    int8_t directionSign;
} MotorStepper;

static MotorStepper g_motors[MOTOR_COUNT] = {
    {
        PIN_STEPPER_CHASSIS_LEFT_DIR_PORT,
        PIN_STEPPER_CHASSIS_LEFT_DIR,
        0U, 1
    },
    {
        PIN_STEPPER_CHASSIS_RIGHT_DIR_PORT,
        PIN_STEPPER_CHASSIS_RIGHT_DIR,
        0U, 1
    },
    {
        PIN_STEPPER_GIMBAL_1_DIR_PORT,
        PIN_STEPPER_GIMBAL_1_DIR,
        0U, 1
    },
    {
        PIN_STEPPER_GIMBAL_2_DIR_PORT,
        PIN_STEPPER_GIMBAL_2_DIR,
        0U, 1
    }
};

/*
 * 作用：把上层 SPS 限制到步进调度可接受范围。
 * 使用场景：Motor_Set 接收上层速度后先做保护。
 */
static uint16_t Motor_ClampSpeed(uint16_t speedSps)
{
    return (speedSps > CAR_STEPPER_SPEED_MAX_SPS) ?
        CAR_STEPPER_SPEED_MAX_SPS : speedSps;
}

/*
 * 作用：把 int16_t 有符号 SPS 转成正向幅值。
 * 使用场景：Motor_SetChassisCommand 接收左右底盘有符号速度。
 */
static uint16_t Motor_SpeedFromSigned(int16_t speedSps)
{
    int32_t value = (int32_t)speedSps;

    if (value < 0) {
        value = -value;
    }
    if (value > (int32_t)CAR_STEPPER_SPEED_MAX_SPS) {
        value = (int32_t)CAR_STEPPER_SPEED_MAX_SPS;
    }
    return (uint16_t)value;
}

static uint8_t Motor_IsValid(MotorId motor)
{
    return ((uint32_t)motor < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

/*
 * 作用：根据底盘左右电机安装方向修正 DIR 电平。
 * 使用场景：Motor_Set 收到逻辑方向后，写实际 DIR 引脚前调用。
 * 说明：只改变物理 DIR 电平，不改变 STEP 计数的逻辑正负号。
 */
static MotorDir Motor_ApplyDirectionReverse(MotorId motor, MotorDir dir)
{
    uint8_t reverse = 0U;

    if (motor == MOTOR_CHASSIS_LEFT) {
        reverse = CAR_CHASSIS_LEFT_REVERSE;
    } else if (motor == MOTOR_CHASSIS_RIGHT) {
        reverse = CAR_CHASSIS_RIGHT_REVERSE;
    }

    if (reverse == 0U) {
        return dir;
    }

    return (dir == MOTOR_REVERSE) ? MOTOR_FORWARD : MOTOR_REVERSE;
}

/* 作用：把逻辑方向转换成 STEP 计数使用的符号。 */
static int8_t Motor_GetDirectionSign(MotorDir dir)
{
    return (dir == MOTOR_REVERSE) ? -1 : 1;
}

/*
 * 作用：写实际 DIR 管脚。
 * 说明：默认 DIR 低为正转，高为反转；是否需要反相由 Motor_ApplyDirectionReverse 先处理。
 */
static void Motor_WriteDirectionPin(MotorStepper *motor, MotorDir physicalDir)
{
    if (physicalDir == MOTOR_REVERSE) {
        DL_GPIO_setPins(motor->dirPort, motor->dirPin);
    } else {
        DL_GPIO_clearPins(motor->dirPort, motor->dirPin);
    }
}

void Motor_Init(void)
{
    MotorEnable_Init();
    Motor_SetAllStop();
    StepperPulse_Init();
}

void Motor_Task(void)
{
    /* ccs1.2 起 STEP 由 TIMG0 中断输出，主循环保留该接口便于后续扩展。 */
}

void Motor_Set(MotorId motor, MotorDir dir, uint16_t speedSps)
{
    MotorStepper *stepper;
    MotorDir physicalDir;
    int8_t logicalDirectionSign;

    if (!Motor_IsValid(motor)) {
        return;
    }

    stepper = &g_motors[motor];
    if ((dir == MOTOR_COAST) || (dir == MOTOR_BRAKE) || (speedSps == 0U)) {
        stepper->speedSps = 0U;
        StepperPulse_SetTarget(motor, stepper->directionSign, 0U);
        return;
    }

    logicalDirectionSign = Motor_GetDirectionSign(dir);
    if (logicalDirectionSign != stepper->directionSign) {
        StepperPulse_SetTarget(motor, stepper->directionSign, 0U);
    }
    physicalDir = Motor_ApplyDirectionReverse(motor, dir);
    Motor_WriteDirectionPin(stepper, physicalDir);
    stepper->directionSign = logicalDirectionSign;
    stepper->speedSps = Motor_ClampSpeed(speedSps);
    StepperPulse_SetTarget(motor, stepper->directionSign,
        stepper->speedSps);
}

void Motor_SetChassisCommand(int16_t leftSpeedSps, int16_t rightSpeedSps)
{
    Motor_Set(MOTOR_CHASSIS_LEFT,
        (leftSpeedSps >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE,
        Motor_SpeedFromSigned(leftSpeedSps));
    Motor_Set(MOTOR_CHASSIS_RIGHT,
        (rightSpeedSps >= 0) ? MOTOR_FORWARD : MOTOR_REVERSE,
        Motor_SpeedFromSigned(rightSpeedSps));
}

void Motor_SetAllStop(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)MOTOR_COUNT; ++i) {
        g_motors[i].speedSps = 0U;
        g_motors[i].directionSign = 1;
        DL_GPIO_clearPins(g_motors[i].dirPort, g_motors[i].dirPin);
    }
    StepperPulse_StopAll();
}

void Motor_Stop(void)
{
    Motor_SetAllStop();
}

int16_t Motor_GetCommand(MotorId motor)
{
    MotorStepper *stepper;

    if (!Motor_IsValid(motor)) {
        return 0;
    }
    stepper = &g_motors[motor];
    return (int16_t)((stepper->directionSign >= 0) ?
        (int16_t)stepper->speedSps : -(int16_t)stepper->speedSps);
}

int32_t Motor_GetStepCount(MotorId motor)
{
    if (!Motor_IsValid(motor)) {
        return 0;
    }
    return StepperPulse_GetStepCount(motor);
}

void Motor_ResetStepCount(MotorId motor)
{
    if (!Motor_IsValid(motor)) {
        return;
    }
    StepperPulse_ResetStepCount(motor);
}

void Motor_ResetAllStepCounts(void)
{
    StepperPulse_ResetAllStepCounts();
}
