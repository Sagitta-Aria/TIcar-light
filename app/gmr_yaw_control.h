#ifndef GMR_YAW_CONTROL_H
#define GMR_YAW_CONTROL_H

#include <stdint.h>

typedef struct {
    uint8_t active;
    uint8_t sensorFresh;
    uint8_t referenceLocked;
    uint8_t targetReached;
    int32_t yawX100;
    int32_t targetYawX100;
    int32_t errorX100;
    int32_t yawRateX100PerSec;
    uint32_t sensorAgeMs;
    int16_t baseCommandCounts;
    int16_t wheelCommandCounts;
    int16_t leftTargetCounts;
    int16_t rightTargetCounts;
} GmrYawControlSnapshot;

/* 初始化外部M0姿态航向跟踪；不启动底盘输出。 */
void GmrYawControl_Init(void);

/* 读取最新姿态并在上电后的第一帧锁存航向基准；不写电机。 */
void GmrYawControl_Observe(void);

/* 请求以前述上电航向为目标进行前进保持；无帧时等待第一帧。 */
void GmrYawControl_StartHold(void);

/* 以当前yaw为基准修改相对目标；传感器无效时返回0。 */
uint8_t GmrYawControl_StartRelative(int16_t angleDegrees);

/* 退出航向保持并立即清零左右轮目标。 */
void GmrYawControl_Stop(void);

/* Task5运行时由10ms底盘控制任务调用一次并更新差速目标。 */
void GmrYawControl_RunControlPeriod(void);

/* 原子读取航向外环状态，供Task5串口和Menu2显示。 */
void GmrYawControl_GetSnapshot(GmrYawControlSnapshot *snapshot);

#endif
