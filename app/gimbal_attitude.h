#ifndef GIMBAL_ATTITUDE_H
#define GIMBAL_ATTITUDE_H

#include <stdint.h>

#include "body_motion.h"
#include "h7_gyro_link.h"

typedef struct {
    uint16_t stepsPerRevolution;
    int8_t motorDirectionSign;
    int8_t h7FeedbackSign;
    int8_t jy61FeedForwardSign;
    uint16_t jy61KffQ1024;
    uint16_t h7AngleKpQ1024;
    uint16_t h7RateKpQ1024;
    uint16_t maxSpeedSps;
    uint16_t accelStepSps;
    uint32_t positionLimitSteps;
} GimbalAttitudeConfig;

typedef struct {
    BodyMotionSnapshot motion;
    H7GyroLinkFeedback feedback;
    GimbalAttitudeConfig config;
    int32_t referenceYawX100;
    int32_t feedbackYawX100;
    int32_t feedbackRateX100PerSec;
    int32_t angleErrorX100;
    int32_t rateReferenceX100PerSec;
    int32_t rateErrorX100PerSec;
    int32_t currentStep;
    int16_t feedForwardSps;
    int16_t rateFeedbackSps;
    int16_t commandSps;
    int16_t stepOutputSps;
    uint32_t feedbackAngleAgeMs;
    uint32_t feedbackGyroAgeMs;
    uint8_t active;
    uint8_t holdEnabled;
    uint8_t feedForwardEnabled;
    uint8_t feedbackFresh;
    uint8_t feedForwardFresh;
    uint8_t hasReference;
    uint8_t referenceTracking;
    uint8_t directOutput;
} GimbalAttitudeSnapshot;

/* 初始化 H7 反馈、JY61前馈的 yaw 姿态保持模块，默认不输出。 */
void GimbalAttitude_Init(void);

/* 启动 Task8/Task5 云台姿态保持，并锁存首个有效H7 yaw为目标。 */
void GimbalAttitude_Start(void);

/* 启动 Task4 姿态辅助；只计算补偿命令，由视觉云台统一输出电机速度。 */
void GimbalAttitude_StartAssist(void);

/* 停止 yaw 输出并恢复该轴的默认斜坡参数。 */
void GimbalAttitude_Stop(void);

/* 固定10ms或视觉帧提前唤醒入口：H7反馈，板载JY61提供底座前馈。 */
void GimbalAttitude_Task(void);

/* 重新采集板载JY61前馈零偏；H7反馈不在M0端重复校准。 */
void GimbalAttitude_StartCalibration(void);

/* 在线启停姿态保持；模块保持 active 以便继续观察姿态。 */
void GimbalAttitude_SetHoldEnabled(uint8_t enabled);

/* 上层按任务策略控制板载JY61角速度前馈；H7反馈始终生效。 */
void GimbalAttitude_SetFeedForwardEnabled(uint8_t enabled);

/* 明确请求时才重抓H7参考；Task4固定跟车矫正期间必须保持关闭。 */
void GimbalAttitude_SetReferenceTracking(uint8_t enabled);

uint8_t GimbalAttitude_IsActive(void);
uint8_t GimbalAttitude_DrivesMotorDirectly(void);
int16_t GimbalAttitude_GetCommandSps(void);
void GimbalAttitude_GetSnapshot(GimbalAttitudeSnapshot *snapshot);
void GimbalAttitude_GetConfig(GimbalAttitudeConfig *config);
uint8_t GimbalAttitude_SetConfig(const GimbalAttitudeConfig *config);

#endif
