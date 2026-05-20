#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include <stdint.h>

/* MotorTest_Init：初始化电机方向确认模块，不会让电机转动。 */
void MotorTest_Init(void);

/* MotorTest_Start：进入方向确认流程，并执行第一个低速测试步骤。 */
void MotorTest_Start(void);

/* MotorTest_Stop：立即停止电机方向确认流程和电机输出。 */
void MotorTest_Stop(void);

/* MotorTest_Next：切到下一步方向确认，返回 0 表示测试流程已结束。 */
uint8_t MotorTest_Next(void);

/* MotorTest_Task：处理自动停车计时，应在主循环里周期调用。 */
void MotorTest_Task(void);

/* MotorTest_IsRunning：返回方向确认流程是否正在进行。 */
uint8_t MotorTest_IsRunning(void);

/* MotorTest_GetStepName：返回当前测试步骤名称，用于串口/OLED 调试。 */
const char *MotorTest_GetStepName(void);

#endif
