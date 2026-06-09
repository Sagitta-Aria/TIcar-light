#ifndef GIMBAL_MOTOR_TEST_H
#define GIMBAL_MOTOR_TEST_H

#include <stdint.h>

typedef enum {
    GIMBAL_MOTOR_TEST_STAGE_IDLE = 0,
    GIMBAL_MOTOR_TEST_STAGE_SPEED
} GimbalMotorTestStage;

/* GimbalMotorTest_Init：初始化云台电机测试状态。 */
void GimbalMotorTest_Init(void);

/*
 * GimbalMotorTest_Start：开始云台电机 SPS 测试。
 * 说明：yaw/pitch 使用独立默认速度，菜单里可按 500 SPS 同时加减。
 */
void GimbalMotorTest_Start(void);

/* GimbalMotorTest_Stop：停止云台电机测试，只停止云台两个轴。 */
void GimbalMotorTest_Stop(void);

/*
 * GimbalMotorTest_Task：周期保持当前 SPS 输出。
 * 说明：驱动器断线、报警或未使能时，MCU 仍会继续发 STEP。
 */
void GimbalMotorTest_Task(void);

/* GimbalMotorTest_IncreaseSpeed/DecreaseSpeed：按 500 SPS 同时调整 yaw/pitch。 */
void GimbalMotorTest_IncreaseSpeed(void);
void GimbalMotorTest_DecreaseSpeed(void);

/* GimbalMotorTest_GetSpeedSps：兼容旧接口，返回 yaw 当前速度，单位 step/s。 */
uint16_t GimbalMotorTest_GetSpeedSps(void);

/* GimbalMotorTest_GetYaw/PitchSpeedSps：读取两轴当前速度，单位 step/s。 */
uint16_t GimbalMotorTest_GetYawSpeedSps(void);
uint16_t GimbalMotorTest_GetPitchSpeedSps(void);

uint8_t GimbalMotorTest_IsRunning(void);
GimbalMotorTestStage GimbalMotorTest_GetStage(void);
const char *GimbalMotorTest_GetStageName(void);
uint8_t GimbalMotorTest_GetProgressPercent(void);

#endif
