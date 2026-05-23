#ifndef TRACKING_EXCEPTION_H
#define TRACKING_EXCEPTION_H

#include <stdint.h>

/* TrackingExceptionState：循迹异常的当前状态，方便后续接入状态机。 */
typedef enum {
    TRACKING_EXCEPTION_STATE_NORMAL = 0,
    TRACKING_EXCEPTION_STATE_WIDE_LINE,
    TRACKING_EXCEPTION_STATE_LOST_HOLD,
    TRACKING_EXCEPTION_STATE_LOST_SEARCH_LEFT,
    TRACKING_EXCEPTION_STATE_LOST_SEARCH_RIGHT,
    TRACKING_EXCEPTION_STATE_ADC_FAULT,
    TRACKING_EXCEPTION_STATE_LOST_STOP
} TrackingExceptionState;

/* TrackingExceptionAction：异常处理后的动作，继续跑还是立即停车。 */
typedef enum {
    TRACKING_EXCEPTION_ACTION_RUN = 0,
    TRACKING_EXCEPTION_ACTION_STOP
} TrackingExceptionAction;

/* TrackingException_Init：初始化循迹异常处理器。 */
void TrackingException_Init(void);

/* TrackingException_Reset：清空丢线、搜线和故障计数。 */
void TrackingException_Reset(void);

/*
 * TrackingException_Update：根据当前灰度结果决定是正常循迹、搜线还是停车。
 * 使用场景：Tracking_Task 每个主循环调用一次。
 */
TrackingExceptionAction TrackingException_Update(uint8_t sampleOk,
    uint8_t digitalMask, int16_t lineError, uint16_t baseCommand,
    uint16_t turnLimit, int16_t *leftCommand, int16_t *rightCommand);

/* TrackingException_GetState：读取当前异常状态。 */
TrackingExceptionState TrackingException_GetState(void);

/* TrackingException_GetStateName：把状态转成字符串，便于串口/OLED 调试。 */
const char *TrackingException_GetStateName(TrackingExceptionState state);

/* TrackingException_GetLostTicks：读取连续丢线的循环次数。 */
uint16_t TrackingException_GetLostTicks(void);

/* TrackingException_GetAdcFaultTicks：读取连续 ADC 失败次数。 */
uint16_t TrackingException_GetAdcFaultTicks(void);

#endif
