#include "vision.h"

#include "gimbal.h"
#include "link.h"
#include "staticconfig.h"

#define VISION_LINE_SIZE      (64U)
#define VISION_MAX_VALUES     (4U)
#define VISION_ERROR_SCALE    (10)

typedef struct {
    uint8_t running;
    uint8_t hasFrame;
    uint32_t frameCount;
    uint32_t badFrameCount;
    int16_t rawX;
    int16_t rawY;
} VisionState;

static VisionState g_vision;

static int16_t Vision_ClampInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static uint8_t Vision_IsDigit(char value)
{
    return ((value >= '0') && (value <= '9')) ? 1U : 0U;
}

/*
 * 作用：从视觉一行文本里提取最多四个数，统一转成 0.1 像素单位。
 * 格式：兼容 "12.5,-8.0;30.2,15.7" 或 "x=12,y=-8"。
 */
static uint8_t Vision_ParseValues(const char *line,
    int16_t values[VISION_MAX_VALUES])
{
    uint8_t count = 0U;
    uint8_t negative;
    uint8_t hasInteger;
    uint8_t hasFraction;
    uint8_t firstFractionDigit;
    uint8_t secondFractionDigit;
    uint8_t roundUp;
    int32_t value;
    int32_t limit;

    while ((line != 0) && (*line != '\0') &&
        (count < VISION_MAX_VALUES)) {
        while ((*line != '\0') && (*line != '-') &&
            !Vision_IsDigit(*line)) {
            ++line;
        }
        if (*line == '\0') {
            break;
        }

        negative = 0U;
        if (*line == '-') {
            negative = 1U;
            ++line;
        }
        if (!Vision_IsDigit(*line) && (*line != '.')) {
            continue;
        }

        value = 0;
        hasInteger = 0U;
        hasFraction = 0U;
        roundUp = 0U;
        limit = negative ? 32768 : 32767;
        while (Vision_IsDigit(*line)) {
            hasInteger = 1U;
            value = (value * 10) + (int32_t)(*line - '0');
            if (value > limit) {
                value = limit;
            }
            ++line;
        }

        firstFractionDigit = 0U;
        secondFractionDigit = 0U;
        if (*line == '.') {
            ++line;
            if (Vision_IsDigit(*line)) {
                hasFraction = 1U;
                firstFractionDigit = (uint8_t)(*line - '0');
                ++line;
                if (Vision_IsDigit(*line)) {
                    secondFractionDigit = (uint8_t)(*line - '0');
                    roundUp = (secondFractionDigit >= 5U) ? 1U : 0U;
                    ++line;
                }
                while (Vision_IsDigit(*line)) {
                    ++line;
                }
            }
        }

        if (!hasInteger && (hasFraction == 0U)) {
            continue;
        }

        value = (value * VISION_ERROR_SCALE) + (int32_t)firstFractionDigit;
        if (value > limit) {
            value = limit;
        }
        if (roundUp && (value < limit)) {
            ++value;
        }
        if (negative) {
            value = -value;
        }

        values[count] = Vision_ClampInt16(value);
        ++count;
    }

    return count;
}

/* 作用：按当前六套云台参数选择中心误差或圆点误差，并写入 gimbal。 */
static void Vision_ApplyValues(const int16_t *values, uint8_t count)
{
    const StaticConfigGimbalTask *config = StaticConfig_GetActiveGimbal();
    uint8_t valueIndex = 0U;

    if (count < 2U) {
        ++g_vision.badFrameCount;
        return;
    }

    if (config->useCircleError != 0U) {
        if (count < 4U) {
            ++g_vision.badFrameCount;
            return;
        }
        valueIndex = 2U;
    }

    g_vision.rawX = values[valueIndex];
    g_vision.rawY = values[valueIndex + 1U];
    Gimbal_UpdateFromCameraError(g_vision.rawX, g_vision.rawY);
    g_vision.hasFrame = 1U;
    ++g_vision.frameCount;
}

void Vision_Init(void)
{
    g_vision.running = 0U;
    g_vision.hasFrame = 0U;
    g_vision.frameCount = 0U;
    g_vision.badFrameCount = 0U;
    g_vision.rawX = 0;
    g_vision.rawY = 0;
}

void Vision_Start(void)
{
    Link_ClearRx();
    g_vision.running = 1U;
    g_vision.hasFrame = 0U;
    g_vision.frameCount = 0U;
    g_vision.badFrameCount = 0U;
    g_vision.rawX = 0;
    g_vision.rawY = 0;
}

void Vision_Stop(void)
{
    g_vision.running = 0U;
    g_vision.hasFrame = 0U;
}

void Vision_Task(void)
{
    char line[VISION_LINE_SIZE];
    int16_t values[VISION_MAX_VALUES];
    uint8_t valueCount;

    if (g_vision.running == 0U) {
        return;
    }

    if (Link_PopLine(line, (uint16_t)sizeof(line)) == 0U) {
        return;
    }

    valueCount = Vision_ParseValues(line, values);
    Vision_ApplyValues(values, valueCount);
}

uint8_t Vision_IsRunning(void)
{
    return g_vision.running;
}

uint8_t Vision_HasFrame(void)
{
    return g_vision.hasFrame;
}

uint32_t Vision_GetFrameCount(void)
{
    return g_vision.frameCount;
}

uint32_t Vision_GetBadFrameCount(void)
{
    return g_vision.badFrameCount;
}

int16_t Vision_GetRawX(void)
{
    return g_vision.rawX;
}

int16_t Vision_GetRawY(void)
{
    return g_vision.rawY;
}
