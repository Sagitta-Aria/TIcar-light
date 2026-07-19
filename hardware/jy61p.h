#ifndef JY61P_H
#define JY61P_H

#include <stdint.h>

typedef struct {
    int16_t rollX100;
    int16_t pitchX100;
    int16_t yawX100;
    int32_t rollRateX100PerSec;
    int32_t pitchRateX100PerSec;
    int32_t yawRateX100PerSec;
    uint32_t angleFrameCount;
    uint32_t gyroFrameCount;
    uint32_t angleFrameTick;
    uint32_t gyroFrameTick;
    uint32_t badFrameCount;
} JY61P_Attitude;

/* 板载JY61P使用UART1/PB7，提供底座yaw角度和角速度前馈。 */
/* JY61P_Init：清空姿态解析状态，并打开UART1接收中断。 */
void JY61P_Init(void);

/* JY61P_HandleUARTInterrupt：UART1中断入口，只收字节和更新姿态缓存。 */
void JY61P_HandleUARTInterrupt(void);

/* JY61P_GetAttitude：读取最近的欧拉角和角速度；角速度单位为 0.01 度/秒。 */
uint8_t JY61P_GetAttitude(JY61P_Attitude *attitude);

/* JY61P_PrintTask：人工调试姿态串口时调用，比赛RTOS任务不周期调用。 */
void JY61P_PrintTask(void);

#endif
