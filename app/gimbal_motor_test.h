#ifndef GIMBAL_MOTOR_TEST_H
#define GIMBAL_MOTOR_TEST_H

#include <stdint.h>

typedef enum {
    GIMBAL_MOTOR_TEST_STAGE_IDLE = 0,
    GIMBAL_MOTOR_TEST_STAGE_ZERO,
    GIMBAL_MOTOR_TEST_STAGE_LEFT_RIGHT_90,
    GIMBAL_MOTOR_TEST_STAGE_UP_DOWN_90,
    GIMBAL_MOTOR_TEST_STAGE_SIM_TRACK,
    GIMBAL_MOTOR_TEST_STAGE_CIRCLE,
    GIMBAL_MOTOR_TEST_STAGE_DONE
} GimbalMotorTestStage;

/* GimbalMotorTest_Init：初始化云台电机测试状态。 */
void GimbalMotorTest_Init(void);

/*
 * GimbalMotorTest_Start：开始云台综合测试。
 * 说明：先做 90 度发脉冲，再跑仿真视觉跟踪和开环画圆。
 */
void GimbalMotorTest_Start(void);

/* GimbalMotorTest_Stop：停止云台电机测试，只停止云台两个轴。 */
void GimbalMotorTest_Stop(void);

/*
 * GimbalMotorTest_Task：周期检查 STEP 输出计数并切换测试阶段。
 * 说明：驱动器断线、报警或未使能时，软件进度仍可能到 100%。
 */
void GimbalMotorTest_Task(void);

uint8_t GimbalMotorTest_IsRunning(void);
GimbalMotorTestStage GimbalMotorTest_GetStage(void);
const char *GimbalMotorTest_GetStageName(void);
uint8_t GimbalMotorTest_GetProgressPercent(void);

#endif
