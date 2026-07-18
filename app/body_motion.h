#ifndef BODY_MOTION_H
#define BODY_MOTION_H

#include <stdint.h>

typedef enum {
    BODY_MOTION_WAITING = 0,
    BODY_MOTION_CALIBRATING,
    BODY_MOTION_READY,
    BODY_MOTION_STALE
} BodyMotionState;

typedef struct {
    uint16_t gyroAlphaQ1024;
    uint16_t yawBetaQ1024;
    uint16_t predictionMs;
    uint16_t staleMs;
    uint16_t calibrationSamples;
} BodyMotionConfig;

typedef struct {
    BodyMotionState state;
    int16_t yawRawX100;
    int32_t yawUnwrappedX100;
    int32_t yawEstimateX100;
    int32_t yawControlX100;
    int32_t yawRateRawX100PerSec;
    int32_t yawRateFilteredX100PerSec;
    int32_t gyroBiasX100PerSec;
    uint32_t angleFrameCount;
    uint32_t gyroFrameCount;
    uint32_t angleAgeMs;
    uint32_t gyroAgeMs;
    uint16_t calibrationCount;
    uint16_t calibrationTarget;
} BodyMotionSnapshot;

/* 初始化唯一一份 JY61 yaw 估计状态；底盘和云台都读取这一份结果。 */
void BodyMotion_Init(void);

/* 以 10 ms 固定周期消费 JY61 快照并执行预测、低通和角度校正。 */
void BodyMotion_Task(void);

/* 清空零偏累计并重新采集静止角速度，校准期间控制输出应保持为零。 */
void BodyMotion_StartCalibration(void);

/* 原子读取当前共享姿态快照。 */
void BodyMotion_GetSnapshot(BodyMotionSnapshot *snapshot);

/* 读取或更新滤波参数；非法范围返回 0 且保持原参数。 */
void BodyMotion_GetConfig(BodyMotionConfig *config);
uint8_t BodyMotion_SetConfig(const BodyMotionConfig *config);

#endif
