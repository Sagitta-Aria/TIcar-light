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

/* 初始化H7姿态接收；GMR使用UART3/PB3，普通配置沿用UART0/PA11。 */
void H7GyroLink_Init(void);

/* H7 UART中断入口；只接收H7的0x52/0x53姿态帧。 */
void H7GyroLink_HandleUARTInterrupt(void);

/* 消费一个H7 JY61兼容帧字节；返回1表示该字节属于IMU帧。 */
uint8_t H7GyroLink_ConsumeByte(uint8_t data);

/* GMR H7 LCD通过同一UART3/PB2发送；带有限等待，失败返回0。 */
uint8_t H7GyroLink_TrySendBytes(const uint8_t *data, uint16_t length);

/* 原子读取H7反馈；库关闭或尚未收到任何帧时清零输出并返回0。 */
uint8_t H7GyroLink_GetFeedback(H7GyroLinkFeedback *feedback);

#endif
