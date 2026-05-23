#include "motor_test.h"

#include "board_config.h"
#include "log_uart.h"
#include "motor.h"

typedef enum {
    MOTOR_TEST_STEP_IDLE = 0,
    MOTOR_TEST_STEP_CHASSIS_LEFT_FORWARD,
    MOTOR_TEST_STEP_CHASSIS_LEFT_REVERSE,
    MOTOR_TEST_STEP_CHASSIS_RIGHT_FORWARD,
    MOTOR_TEST_STEP_CHASSIS_RIGHT_REVERSE,
    MOTOR_TEST_STEP_GIMBAL_1_FORWARD,
    MOTOR_TEST_STEP_GIMBAL_1_REVERSE,
    MOTOR_TEST_STEP_GIMBAL_2_FORWARD,
    MOTOR_TEST_STEP_GIMBAL_2_REVERSE,
    MOTOR_TEST_STEP_DONE
} MotorTestStep;

static MotorTestStep g_motorTestStep = MOTOR_TEST_STEP_IDLE;
static uint16_t g_motorTestTicks;
static uint8_t g_motorTestRunning;

/*
 * 作用：执行单个步进电机方向测试。
 * 使用场景：MotorTest_ApplyStep 根据当前步骤调用。
 * 说明：只给一个电机输出低速 STEP，方便逐个核对接线和方向。
 */
static void MotorTest_RunOne(MotorId motor, MotorDir dir, const char *text)
{
    Motor_Stop();
    Motor_Set(motor, dir, CAR_STEPPER_TEST_COMMAND);
    LogUart_SendString(text);
    LogUart_SendString("\r\n");
}

/*
 * 作用：执行当前测试步骤的步进输出。
 * 使用场景：MotorTest_Start 和 MotorTest_Next 内部调用。
 * 不要用于：正式循迹或云台控制，正式控制应该走 motion/tracking 或后续 gimbal 模块。
 */
static void MotorTest_ApplyStep(void)
{
    switch (g_motorTestStep) {
    case MOTOR_TEST_STEP_CHASSIS_LEFT_FORWARD:
        MotorTest_RunOne(MOTOR_CHASSIS_LEFT, MOTOR_FORWARD,
            "stepper test: chassis left forward");
        break;

    case MOTOR_TEST_STEP_CHASSIS_LEFT_REVERSE:
        MotorTest_RunOne(MOTOR_CHASSIS_LEFT, MOTOR_REVERSE,
            "stepper test: chassis left reverse");
        break;

    case MOTOR_TEST_STEP_CHASSIS_RIGHT_FORWARD:
        MotorTest_RunOne(MOTOR_CHASSIS_RIGHT, MOTOR_FORWARD,
            "stepper test: chassis right forward");
        break;

    case MOTOR_TEST_STEP_CHASSIS_RIGHT_REVERSE:
        MotorTest_RunOne(MOTOR_CHASSIS_RIGHT, MOTOR_REVERSE,
            "stepper test: chassis right reverse");
        break;

    case MOTOR_TEST_STEP_GIMBAL_1_FORWARD:
        MotorTest_RunOne(MOTOR_GIMBAL_1, MOTOR_FORWARD,
            "stepper test: gimbal 1 forward");
        break;

    case MOTOR_TEST_STEP_GIMBAL_1_REVERSE:
        MotorTest_RunOne(MOTOR_GIMBAL_1, MOTOR_REVERSE,
            "stepper test: gimbal 1 reverse");
        break;

    case MOTOR_TEST_STEP_GIMBAL_2_FORWARD:
        MotorTest_RunOne(MOTOR_GIMBAL_2, MOTOR_FORWARD,
            "stepper test: gimbal 2 forward");
        break;

    case MOTOR_TEST_STEP_GIMBAL_2_REVERSE:
        MotorTest_RunOne(MOTOR_GIMBAL_2, MOTOR_REVERSE,
            "stepper test: gimbal 2 reverse");
        break;

    case MOTOR_TEST_STEP_IDLE:
    case MOTOR_TEST_STEP_DONE:
    default:
        Motor_Stop();
        LogUart_SendString("stepper test: stop\r\n");
        break;
    }

    g_motorTestTicks = CAR_MOTOR_TEST_RUN_TICKS;
}

void MotorTest_Init(void)
{
    g_motorTestStep = MOTOR_TEST_STEP_IDLE;
    g_motorTestTicks = 0U;
    g_motorTestRunning = 0U;
}

void MotorTest_Start(void)
{
    MotorTest_Init();
    g_motorTestRunning = 1U;
    g_motorTestStep = MOTOR_TEST_STEP_CHASSIS_LEFT_FORWARD;
    MotorTest_ApplyStep();
}

void MotorTest_Stop(void)
{
    g_motorTestStep = MOTOR_TEST_STEP_IDLE;
    g_motorTestTicks = 0U;
    g_motorTestRunning = 0U;
    Motor_Stop();
}

uint8_t MotorTest_Next(void)
{
    if (!g_motorTestRunning) {
        MotorTest_Start();
        return 1U;
    }

    if (g_motorTestStep >= MOTOR_TEST_STEP_GIMBAL_2_REVERSE) {
        g_motorTestStep = MOTOR_TEST_STEP_DONE;
        MotorTest_Stop();
        LogUart_SendString("stepper test: done\r\n");
        return 0U;
    }

    g_motorTestStep = (MotorTestStep)((uint8_t)g_motorTestStep + 1U);
    MotorTest_ApplyStep();
    return 1U;
}

void MotorTest_Task(void)
{
    if (!g_motorTestRunning || (g_motorTestTicks == 0U)) {
        return;
    }

    --g_motorTestTicks;
    if (g_motorTestTicks == 0U) {
        Motor_Stop();
        LogUart_SendString("stepper test: auto stop\r\n");
    }
}

uint8_t MotorTest_IsRunning(void)
{
    return g_motorTestRunning;
}

const char *MotorTest_GetStepName(void)
{
    switch (g_motorTestStep) {
    case MOTOR_TEST_STEP_CHASSIS_LEFT_FORWARD:
        return "CL forward";
    case MOTOR_TEST_STEP_CHASSIS_LEFT_REVERSE:
        return "CL reverse";
    case MOTOR_TEST_STEP_CHASSIS_RIGHT_FORWARD:
        return "CR forward";
    case MOTOR_TEST_STEP_CHASSIS_RIGHT_REVERSE:
        return "CR reverse";
    case MOTOR_TEST_STEP_GIMBAL_1_FORWARD:
        return "G1 forward";
    case MOTOR_TEST_STEP_GIMBAL_1_REVERSE:
        return "G1 reverse";
    case MOTOR_TEST_STEP_GIMBAL_2_FORWARD:
        return "G2 forward";
    case MOTOR_TEST_STEP_GIMBAL_2_REVERSE:
        return "G2 reverse";
    case MOTOR_TEST_STEP_DONE:
        return "done";
    case MOTOR_TEST_STEP_IDLE:
    default:
        return "idle";
    }
}
