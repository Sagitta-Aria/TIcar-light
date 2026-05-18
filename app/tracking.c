#include "tracking.h"

#include "board_config.h"
#include "gray.h"
#include "motion.h"

static uint8_t g_trackingEnabled;

void Tracking_Init(void)
{
    g_trackingEnabled = 0U;
}

void Tracking_SetEnabled(uint8_t enabled)
{
    g_trackingEnabled = enabled ? 1U : 0U;
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

    if (!g_trackingEnabled) {
        return;
    }

    /*
     * 使用加权偏差做循迹修正。
     * 这里适合正式跑线时使用，不适合做单点黑白测试。
     * 如果当前没有任何通道压线，就让电机停住，避免盲跑。
     */
    if (!Gray_Update() || !Gray_GetWeightedLineError(&error)) {
#if CAR_TRACK_LOST_STOP
        Motion_Stop();
#endif
        return;
    }

    {
        int16_t correction =
            (int16_t)((error * CAR_TRACK_TURN_GAIN) / GRAY_LINE_ERROR_SCALE);
        int16_t left = (int16_t)(CAR_TRACK_BASE_DUTY + correction);
        int16_t right = (int16_t)(CAR_TRACK_BASE_DUTY - correction);
        Motion_SetSpeed(left, right);
    }
}
