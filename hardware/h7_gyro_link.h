#ifndef H7_GYRO_LINK_H
#define H7_GYRO_LINK_H

#include <stdint.h>

typedef struct {
    int16_t rollX100;
    int16_t pitchX100;
    int16_t yawRawX100;
    int32_t yawUnwrappedX100;
    int32_t rollRateX100PerSec;
    int32_t pitchRateX100PerSec;
    int32_t yawRateX100PerSec;
    uint32_t angleFrameCount;
    uint32_t gyroFrameCount;
    uint32_t angleFrameTick;
    uint32_t gyroFrameTick;
    uint32_t badFrameCount;
} H7GyroLinkFeedback;

/* 初始化 UART0/PA11 上的 H7 姿态反馈解析器并打开接收中断。 */
void H7GyroLink_Init(void);

/* UART0 中断入口；只接收 H7 的 0x52/0x53 姿态帧。 */
void H7GyroLink_HandleUARTInterrupt(void);

/* 原子读取 H7 已处理的姿态反馈；尚未收到任何帧时返回 0。 */
uint8_t H7GyroLink_GetFeedback(H7GyroLinkFeedback *feedback);

#endif
