#include "motor_test.h"

#include "board_config.h"
#include "link.h"
#include "motor.h"

typedef enum {
    MOTOR_TEST_STEP_IDLE = 0,
    MOTOR_TEST_STEP_LEFT_FORWARD,
    MOTOR_TEST_STEP_LEFT_REVERSE,
    MOTOR_TEST_STEP_RIGHT_FORWARD,
    MOTOR_TEST_STEP_RIGHT_REVERSE,
    MOTOR_TEST_STEP_BOTH_FORWARD,
    MOTOR_TEST_STEP_BOTH_REVERSE,
    MOTOR_TEST_STEP_DONE
} MotorTestStep;

static MotorTestStep g_motorTestStep = MOTOR_TEST_STEP_IDLE;
static uint16_t g_motorTestTicks;
static uint8_t g_motorTestRunning;

/*
 * 作用：执行当前测试步骤的电机输出。
 * 使用场景：MotorTest_Start 和 MotorTest_Next 内部调用。
 * 不要用于：正式循迹或速度闭环，那里应该走 motion/tracking。
 * 说明：只用低速短时间输出，便于人工观察轮子方向。
 */
static void MotorTest_ApplyStep(void)
{
    Motor_Stop();

    switch (g_motorTestStep) {
    case MOTOR_TEST_STEP_LEFT_FORWARD:
        Motor_Set(MOTOR_LEFT, MOTOR_FORWARD, CAR_MOTOR_TEST_DUTY);
        Link_SendString("motor test: left forward\r\n");
        break;

    case MOTOR_TEST_STEP_LEFT_REVERSE:
        Motor_Set(MOTOR_LEFT, MOTOR_REVERSE, CAR_MOTOR_TEST_DUTY);
        Link_SendString("motor test: left reverse\r\n");
        break;

    case MOTOR_TEST_STEP_RIGHT_FORWARD:
        Motor_Set(MOTOR_RIGHT, MOTOR_FORWARD, CAR_MOTOR_TEST_DUTY);
        Link_SendString("motor test: right forward\r\n");
        break;

    case MOTOR_TEST_STEP_RIGHT_REVERSE:
        Motor_Set(MOTOR_RIGHT, MOTOR_REVERSE, CAR_MOTOR_TEST_DUTY);
        Link_SendString("motor test: right reverse\r\n");
        break;

    case MOTOR_TEST_STEP_BOTH_FORWARD:
        Motor_Set(MOTOR_LEFT, MOTOR_FORWARD, CAR_MOTOR_TEST_DUTY);
        Motor_Set(MOTOR_RIGHT, MOTOR_FORWARD, CAR_MOTOR_TEST_DUTY);
        Link_SendString("motor test: both forward\r\n");
        break;

    case MOTOR_TEST_STEP_BOTH_REVERSE:
        Motor_Set(MOTOR_LEFT, MOTOR_REVERSE, CAR_MOTOR_TEST_DUTY);
        Motor_Set(MOTOR_RIGHT, MOTOR_REVERSE, CAR_MOTOR_TEST_DUTY);
        Link_SendString("motor test: both reverse\r\n");
        break;

    case MOTOR_TEST_STEP_IDLE:
    case MOTOR_TEST_STEP_DONE:
    default:
        Link_SendString("motor test: stop\r\n");
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
    g_motorTestStep = MOTOR_TEST_STEP_LEFT_FORWARD;
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

    if (g_motorTestStep >= MOTOR_TEST_STEP_BOTH_REVERSE) {
        g_motorTestStep = MOTOR_TEST_STEP_DONE;
        MotorTest_Stop();
        Link_SendString("motor test: done\r\n");
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
        Link_SendString("motor test: auto stop\r\n");
    }
}

uint8_t MotorTest_IsRunning(void)
{
    return g_motorTestRunning;
}

const char *MotorTest_GetStepName(void)
{
    switch (g_motorTestStep) {
    case MOTOR_TEST_STEP_LEFT_FORWARD:
        return "left_forward";
    case MOTOR_TEST_STEP_LEFT_REVERSE:
        return "left_reverse";
    case MOTOR_TEST_STEP_RIGHT_FORWARD:
        return "right_forward";
    case MOTOR_TEST_STEP_RIGHT_REVERSE:
        return "right_reverse";
    case MOTOR_TEST_STEP_BOTH_FORWARD:
        return "both_forward";
    case MOTOR_TEST_STEP_BOTH_REVERSE:
        return "both_reverse";
    case MOTOR_TEST_STEP_DONE:
        return "done";
    case MOTOR_TEST_STEP_IDLE:
    default:
        return "idle";
    }
}
