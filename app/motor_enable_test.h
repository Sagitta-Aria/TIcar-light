#ifndef MOTOR_ENABLE_TEST_H
#define MOTOR_ENABLE_TEST_H

#include <stdint.h>

/* MotorEnableTest_Init：初始化四电机使能测试状态。 */
void MotorEnableTest_Init(void);

/*
 * MotorEnableTest_Start：进入四电机使能测试。
 * 说明：只拉 EN，不输出 STEP；用于检查驱动器是否上电抱死。
 */
void MotorEnableTest_Start(void);

/* MotorEnableTest_Stop：退出测试并关闭四路 EN。 */
void MotorEnableTest_Stop(void);

/* MotorEnableTest_Task：预留周期任务，目前不做阻塞动作。 */
void MotorEnableTest_Task(void);

/* MotorEnableTest_IsRunning：读取当前是否处于四电机使能测试状态。 */
uint8_t MotorEnableTest_IsRunning(void);

/* MotorEnableTest_GetEnableLine：返回 OLED 上显示的 EN 状态行。 */
const char *MotorEnableTest_GetEnableLine(void);

#endif
