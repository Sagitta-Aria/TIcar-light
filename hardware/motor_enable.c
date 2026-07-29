/*
 * 电机使能兼容层：保存状态机的使能请求，但当前底盘无独立EN、云台EN由硬件固定有效。
 * 调用这些接口不会翻转MCU GPIO；真正停车必须使用Motor_Stop/Motion_Stop清零输出。
 */
#include "motor_enable.h"

/*
 * PA31/PB19 已改作底盘方向和编码器输入，新板云台EN必须硬件固定有效。
 * 保留这些接口是为了让原任务1~4状态机无需知道板级EN接线变化。
 */
static uint8_t g_enableRequested;

void MotorEnable_Init(void)
{
    g_enableRequested = 0U;
}

void MotorEnable_SetAll(uint8_t enabled)
{
    g_enableRequested = (enabled != 0U) ? 1U : 0U;
}

void MotorEnable_SetChassis(uint8_t enabled)
{
    (void)enabled;
}

void MotorEnable_SetGimbal(uint8_t enabled)
{
    g_enableRequested = (enabled != 0U) ? 1U : 0U;
}

uint8_t MotorEnable_IsAllEnabled(void)
{
    return g_enableRequested;
}

const char *MotorEnable_GetActiveLevelName(void)
{
    return "EN HW";
}
