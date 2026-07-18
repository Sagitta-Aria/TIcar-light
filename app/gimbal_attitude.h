#ifndef GIMBAL_ATTITUDE_H
#define GIMBAL_ATTITUDE_H

#include <stdint.h>

#include "body_motion.h"

typedef struct {
    uint16_t stepsPerRevolution;
    int8_t directionSign;
    uint16_t kffQ1024;
    uint16_t kpQ1024;
    uint16_t maxSpeedSps;
    uint16_t accelStepSps;
    uint32_t positionLimitSteps;
} GimbalAttitudeConfig;

typedef struct {
    BodyMotionSnapshot motion;
    GimbalAttitudeConfig config;
    int32_t referenceStep;
    int32_t currentStep;
    int32_t stepError;
    int16_t feedForwardSps;
    int16_t commandSps;
    int16_t stepOutputSps;
    uint8_t active;
    uint8_t holdEnabled;
    uint8_t feedForwardEnabled;
    uint8_t hasReference;
} GimbalAttitudeSnapshot;

/* 初始化独立于视觉云台的 yaw 姿态保持模块，默认不输出。 */
void GimbalAttitude_Init(void);

/* 启动 Task8/Task5 云台姿态保持，并先执行静止零偏校准。 */
void GimbalAttitude_Start(void);

/* 停止 yaw 输出并恢复该轴的默认斜坡参数。 */
void GimbalAttitude_Stop(void);

/* 10 ms 周期控制入口；只消费 BodyMotion 的共享估计结果。 */
void GimbalAttitude_Task(void);

/* 重新采集 JY61 零偏，校准期间立即停止 yaw。 */
void GimbalAttitude_StartCalibration(void);

/* 在线启停姿态保持；模块保持 active 以便继续观察姿态。 */
void GimbalAttitude_SetHoldEnabled(uint8_t enabled);

/* 上层按任务策略控制角速度前馈；通用启动默认关闭，Task8入口会显式开启。 */
void GimbalAttitude_SetFeedForwardEnabled(uint8_t enabled);

uint8_t GimbalAttitude_IsActive(void);
void GimbalAttitude_GetSnapshot(GimbalAttitudeSnapshot *snapshot);
void GimbalAttitude_GetConfig(GimbalAttitudeConfig *config);
uint8_t GimbalAttitude_SetConfig(const GimbalAttitudeConfig *config);

#endif
