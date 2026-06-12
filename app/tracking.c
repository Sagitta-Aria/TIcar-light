#include "tracking.h"

#include "board_config.h"
#include "gray.h"
#include "motion.h"
#include "route.h"
#include "tracking_exception.h"

static uint8_t g_trackingEnabled;

/*
 * 作用：初始化循迹模块。
 * 使用场景：App_Init 阶段调用一次。
 * 说明：默认不启用，只有状态机进入 Track Test 或正式循迹后才输出底盘命令。
 */
void Tracking_Init(void)
{
    g_trackingEnabled = 0U;
    TrackingException_Init();
}

/*
 * 作用：启停灰度循迹。
 * 使用场景：状态机进入/退出循迹相关状态时。
 * 说明：关闭时立即停车，避免离开页面后底盘继续按上一拍命令运动。
 */
void Tracking_SetEnabled(uint8_t enabled)
{
    uint8_t nextEnabled = enabled ? 1U : 0U;

    g_trackingEnabled = nextEnabled;
    TrackingException_Reset();
    if (!g_trackingEnabled) {
        Motion_Stop();
    }
}

/* 作用：返回循迹是否启用，用于调试或状态显示。 */
uint8_t Tracking_IsEnabled(void)
{
    return g_trackingEnabled;
}

/*
 * 作用：执行一轮灰度循迹控制。
 * 使用场景：StateMachine_Task 在 TRACKING 或 TRACKING_TEST 状态下调用。
 * 说明：本函数只处理底盘命令；路线速度限制由 route 模块提供，异常停车由 tracking_exception 决定。
 */
void Tracking_Task(void)
{
    int16_t error = 0;
    int16_t leftSpeedSps = 0;
    int16_t rightSpeedSps = 0;
    uint16_t baseSpeedSps;
    uint8_t digitalMask = 0U;
    uint8_t sampleOk;
    uint16_t turnLimit;

    if (!g_trackingEnabled) {
        return;
    }

    baseSpeedSps = Route_GetBaseSpeedSps();
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
    if (TrackingException_Update(sampleOk, digitalMask, error, baseSpeedSps,
        turnLimit, &leftSpeedSps, &rightSpeedSps) ==
        TRACKING_EXCEPTION_ACTION_STOP) {
        Motion_Stop();
        return;
    }

    Motion_SetChassisCommand(leftSpeedSps, rightSpeedSps);
}
