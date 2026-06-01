#include "motor_enable_test.h"

#include "log_uart.h"
#include "motor.h"
#include "motor_enable.h"

static uint8_t g_motorEnableTestRunning;

void MotorEnableTest_Init(void)
{
    g_motorEnableTestRunning = 0U;
}

void MotorEnableTest_Start(void)
{
    Motor_SetAllStop();
    MotorEnable_SetAll(1U);
    g_motorEnableTestRunning = 1U;
    LOG_LINE("motor enable test: all enabled");
}

void MotorEnableTest_Stop(void)
{
    if (g_motorEnableTestRunning == 0U) {
        return;
    }

    MotorEnable_SetAll(0U);
    g_motorEnableTestRunning = 0U;
    LOG_LINE("motor enable test: all disabled");
}

void MotorEnableTest_Task(void)
{
}

uint8_t MotorEnableTest_IsRunning(void)
{
    return g_motorEnableTestRunning;
}

const char *MotorEnableTest_GetEnableLine(void)
{
    return (MotorEnable_IsAllEnabled() != 0U) ?
        MotorEnable_GetActiveLevelName() : "EN OFF";
}
