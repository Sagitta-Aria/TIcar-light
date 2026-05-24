#include "gimbal_test.h"

#include "board_config.h"
#include "gimbal.h"
#include "link.h"

#define GIMBAL_TEST_LINE_SIZE        (64U)
#define GIMBAL_TEST_MAX_VALUES       (4U)

typedef struct {
    uint8_t running;
    uint8_t hasVision;
    uint32_t frameCount;
    uint32_t badFrameCount;
} GimbalTestState;

static GimbalTestState g_gimbalTest;

static int16_t GimbalTest_ClampInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }
    if (value < -32768) {
        return -32768;
    }
    return (int16_t)value;
}

static uint8_t GimbalTest_IsDigit(char value)
{
    return ((value >= '0') && (value <= '9')) ? 1U : 0U;
}

/*
 * 作用：从一行视觉数据里提取最多 4 个有符号整数。
 * 格式：支持 "tx,ty,cx,cy" 或 "cx,cy"，分隔符不限于逗号。
 */
static uint8_t GimbalTest_ParseValues(const char *line,
    int16_t values[GIMBAL_TEST_MAX_VALUES])
{
    uint8_t count = 0U;
    uint8_t negative;
    int32_t value;

    while ((line != 0) && (*line != '\0') &&
        (count < GIMBAL_TEST_MAX_VALUES)) {
        while ((*line != '\0') && (*line != '-') &&
            !GimbalTest_IsDigit(*line)) {
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
        if (!GimbalTest_IsDigit(*line)) {
            continue;
        }

        value = 0;
        while (GimbalTest_IsDigit(*line)) {
            value = (value * 10) + (int32_t)(*line - '0');
            if (value > 32768) {
                value = 32768;
            }
            ++line;
        }
        if (negative) {
            value = -value;
        }
        values[count] = GimbalTest_ClampInt16(value);
        ++count;
    }

    return count;
}

static void GimbalTest_ApplyValues(const int16_t *values, uint8_t count)
{
    if (count >= 4U) {
        Gimbal_UpdateFromVision(values[0], values[1], values[2], values[3]);
        g_gimbalTest.hasVision = 1U;
        ++g_gimbalTest.frameCount;
        return;
    }

    if (count >= 2U) {
        Gimbal_UpdateFromVision(CAR_GIMBAL_TEST_TARGET_X,
            CAR_GIMBAL_TEST_TARGET_Y, values[0], values[1]);
        g_gimbalTest.hasVision = 1U;
        ++g_gimbalTest.frameCount;
        return;
    }

    ++g_gimbalTest.badFrameCount;
}

void GimbalTest_Init(void)
{
    g_gimbalTest.running = 0U;
    g_gimbalTest.hasVision = 0U;
    g_gimbalTest.frameCount = 0U;
    g_gimbalTest.badFrameCount = 0U;
}

void GimbalTest_Start(void)
{
    Link_ClearRx();
    g_gimbalTest.running = 1U;
    g_gimbalTest.hasVision = 0U;
    g_gimbalTest.frameCount = 0U;
    g_gimbalTest.badFrameCount = 0U;
    Gimbal_SetTarget(CAR_GIMBAL_TEST_TARGET_X, CAR_GIMBAL_TEST_TARGET_Y);
    Gimbal_SetEnabled(1U);
}

void GimbalTest_Stop(void)
{
    g_gimbalTest.running = 0U;
    g_gimbalTest.hasVision = 0U;
    Gimbal_SetEnabled(0U);
}

void GimbalTest_Task(void)
{
    char line[GIMBAL_TEST_LINE_SIZE];
    int16_t values[GIMBAL_TEST_MAX_VALUES];
    uint8_t valueCount;

    if (!g_gimbalTest.running) {
        return;
    }

    while (Link_PopLine(line, (uint16_t)sizeof(line))) {
        valueCount = GimbalTest_ParseValues(line, values);
        GimbalTest_ApplyValues(values, valueCount);
    }
}

uint8_t GimbalTest_IsRunning(void)
{
    return g_gimbalTest.running;
}

uint8_t GimbalTest_HasVision(void)
{
    return g_gimbalTest.hasVision;
}

uint32_t GimbalTest_GetFrameCount(void)
{
    return g_gimbalTest.frameCount;
}

uint32_t GimbalTest_GetBadFrameCount(void)
{
    return g_gimbalTest.badFrameCount;
}
