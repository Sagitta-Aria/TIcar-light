#ifndef H7_CONTROL_UART_H
#define H7_CONTROL_UART_H

#include <stdint.h>

typedef enum {
    H7_BALL_BALANCE_MODE_DEBUG = 0,
    H7_BALL_BALANCE_MODE_TASK3,
    H7_BALL_BALANCE_MODE_TASK4,
    H7_BALL_BALANCE_MODE_TASK5,
    H7_BALL_BALANCE_MODE_TASK6,
    H7_BALL_BALANCE_MODE_IMU_Y_FEEDFORWARD
} H7BallBalanceMode;

/* Configure Tianmeng UART2/PB15 as a transmit-only H7 display/control link. */
void H7ControlUart_Init(void);

/* Send one complete LCD command without interleaving another sender. */
uint8_t H7ControlUart_TrySendBytes(const uint8_t *data, uint16_t length);

/* 请求H7启动BMI+视觉滚球任务；通信任务只在发送失败时重试，不发送周期心跳。 */
void H7ControlUart_RequestBallBalanceStart(void);

/* 请求H7启动指定比赛/测试滚球参数组。 */
void H7ControlUart_RequestBallBalanceMode(H7BallBalanceMode mode);

/* 请求H7停止滚球/步进输出；通信任务只在发送失败时重试。 */
void H7ControlUart_RequestBallBalanceStop(void);

/* 请求H7按绝对位置模式执行一次步进命令；通信任务负责失败重试。 */
void H7ControlUart_RequestStepperMove(int32_t positionSteps,
    uint16_t speedRpm, uint8_t acceleration);

/*
 * 每5ms通信任务调用：发送待处理的一次性请求，并每20ms发送一次@A前进加速度。
 * forwardAccelerationX100单位为0.01 (encoder count/20ms)/s。
 */
void H7ControlUart_ServiceTx(
    int32_t forwardAccelerationX100, uint16_t elapsedMs);

#endif
