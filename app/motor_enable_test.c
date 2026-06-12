#include "motor_enable_test.h"

#include "log_uart.h"
#include "motor.h"
#include "motor_enable.h"

static uint8_t g_motorEnableTestRunning;

/* 作用：初始化使能测试状态，默认不拉 EN。 */
void MotorEnableTest_Init(void)
{
    g_motorEnableTestRunning = 0U;
}

/*
 * 作用：进入四电机使能测试。
 * 使用场景：菜单 Gimbal Test / Enable Test。
 * 说明：只拉 EN，不发 STEP；用于确认驱动器供电、EN 极性和电机抱死是否正确。
 */
void MotorEnableTest_Start(void)
{
    Motor_SetAllStop();
    MotorEnable_SetAll(1U);
    g_motorEnableTestRunning = 1U;
}

/*
 * 作用：退出四电机使能测试。
 * 说明：只关闭 EN 测试输出，不改其它菜单状态；重复调用是安全的。
 */
void MotorEnableTest_Stop(void)
{
    if (g_motorEnableTestRunning == 0U) {
        return;
    }

    MotorEnable_SetAll(0U);
    g_motorEnableTestRunning = 0U;
    LOG_LINE("motor enable test: all disabled");
}

/*
 * 作用：使能测试的周期任务。
 * 说明：当前测试是静态拉 EN，因此这里保持为空，避免主循环里出现多余阻塞。
 */
void MotorEnableTest_Task(void)
{
}

/* 作用：返回使能测试是否正在运行。 */
uint8_t MotorEnableTest_IsRunning(void)
{
    return g_motorEnableTestRunning;
}

/* 作用：生成 OLED 状态文字，显示 EN 当前是否处于有效电平。 */
const char *MotorEnableTest_GetEnableLine(void)
{
    return (MotorEnable_IsAllEnabled() != 0U) ?
        MotorEnable_GetActiveLevelName() : "EN OFF";
}
