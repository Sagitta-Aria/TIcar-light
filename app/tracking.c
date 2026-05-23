#include "tracking.h"

#include "board_config.h"
#include "gray.h"
#include "motion.h"
#include "route.h"
#include "tracking_exception.h"

static uint8_t g_trackingEnabled;

void Tracking_Init(void)
{
    g_trackingEnabled = 0U;
    TrackingException_Init();
}

void Tracking_SetEnabled(uint8_t enabled)
{
    g_trackingEnabled = enabled ? 1U : 0U;
    TrackingException_Reset();
    if (!g_trackingEnabled) {
        Motion_Stop();
    }
}

uint8_t Tracking_IsEnabled(void)
{
    return g_trackingEnabled;
}

void Tracking_Task(void)
{
    int16_t error = 0;
    int16_t leftCommand = 0;
    int16_t rightCommand = 0;
    uint16_t baseCommand;
    uint8_t digitalMask = 0U;
    uint8_t sampleOk;
    uint16_t turnLimit;

    if (!g_trackingEnabled) {
        return;
    }

    baseCommand = Route_GetBaseCommand();
    turnLimit = Route_GetTurnLimit();
    sampleOk = Gray_Update();
    if (sampleOk) {
        digitalMask = Gray_GetDigitalMask();
        if (digitalMask != 0U) {
            (void)Gray_GetWeightedLineError(&error);
        }
    }

    /*
     * 异常处理器统一决定正常循迹、短暂保持、搜线或停车。
     * 后续要接入状态机时，直接读取 TrackingException_GetState() 即可。
     */
    if (TrackingException_Update(sampleOk, digitalMask, error, baseCommand,
        turnLimit, &leftCommand, &rightCommand) ==
        TRACKING_EXCEPTION_ACTION_STOP) {
        Motion_Stop();
        return;
    }

    Motion_SetChassisCommand(leftCommand, rightCommand);
}
