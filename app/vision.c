/*
 * K230视觉协议解析器：消费UART3完整文本行，校验五个定点字段并更新云台误差。
 * Link ISR只提交行，解析在Gimbal/Comm任务上下文完成；坏帧只计数，不复用为控制输入。
 * 第五字段是目标表观长度，用于连续拟合yaw增益，不再代表近/中/远离散档位。
 */
#include "library_config.h"

#if CAR_PROFILE_IS_FULL

#include "vision.h"

#include "control_config.h"
#include "gimbal.h"
#include "link.h"
#include "staticconfig.h"

#define VISION_LINE_SIZE      (64U)
#define VISION_MAX_VALUES     (5U)
#define VISION_ERROR_SCALE    (10)

typedef struct {
    uint8_t running;
    uint8_t hasFrame;
    uint32_t frameCount;
    uint32_t badFrameCount;
    int16_t rawX;
    int16_t rawY;
    int16_t stageScaleX10;
    uint16_t yawGainQ1024;
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
 * 作用：从视觉一行文本里提取五个数，统一转成0.1单位。
 * 格式："centerDx,centerDy;circleDx,circleDy;stageScale"。
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

/*
 * 作用：把视觉第5字段目标长度拟合为连续yaw增益。
 * 使用场景：仅在五字段视觉帧通过数量校验后调用。
 * 边界：长度在30.0~140.0外先钳位；结果使用Q1024且不操作电机。
 */
static uint16_t Vision_ComputeYawGainQ1024(int16_t targetLengthX10)
{
    int32_t length = targetLengthX10;
    uint32_t lengthOffset;
    uint32_t lengthRange = (uint32_t)(VISION_TARGET_LENGTH_MAX_X10 -
        VISION_TARGET_LENGTH_MIN_X10);
    uint32_t gainRange = GIMBAL_VISION_YAW_GAIN_MAX_Q1024 -
        GIMBAL_VISION_YAW_GAIN_MIN_Q1024;

    if (length <= VISION_TARGET_LENGTH_MIN_X10) {
        return GIMBAL_VISION_YAW_GAIN_MIN_Q1024;
    }
    if (length >= VISION_TARGET_LENGTH_MAX_X10) {
        return GIMBAL_VISION_YAW_GAIN_MAX_Q1024;
    }

    lengthOffset = (uint32_t)(length - VISION_TARGET_LENGTH_MIN_X10);
    return (uint16_t)(GIMBAL_VISION_YAW_GAIN_MIN_Q1024 +
        ((lengthOffset * gainRange) + lengthRange / 2U) / lengthRange);
}

/*
 * 作用：没有有效视觉长度时恢复1.0倍，防止新任务沿用上一任务的K。
 * 使用场景：Vision初始化、启动和停止；不会启动云台或伪造视觉帧。
 */
static void Vision_ResetYawGain(void)
{
    g_vision.stageScaleX10 = 0;
    g_vision.yawGainQ1024 = GIMBAL_VISION_YAW_GAIN_Q1024_SCALE;
    Gimbal_SetVisionYawGainQ1024(g_vision.yawGainQ1024);
}

/* 选择点/圆误差，拟合本帧yaw K，并把有效帧写入Gimbal控制器。 */
static void Vision_ApplyValues(const int16_t *values, uint8_t count)
{
    const StaticConfigGimbalTask *config = StaticConfig_GetActiveGimbal();
    uint8_t valueIndex = 0U;

    if (count < VISION_MAX_VALUES) {
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
    g_vision.stageScaleX10 = values[4];
    g_vision.yawGainQ1024 = Vision_ComputeYawGainQ1024(
        g_vision.stageScaleX10);
    Gimbal_SetVisionYawGainQ1024(g_vision.yawGainQ1024);
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
    Vision_ResetYawGain();
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
    Vision_ResetYawGain();
}

void Vision_Stop(void)
{
    g_vision.running = 0U;
    g_vision.hasFrame = 0U;
    Vision_ResetYawGain();
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

/* 返回最近一帧目标表观长度，单位0.1；没有有效帧时为0。 */
int16_t Vision_GetStageScaleX10(void)
{
    return g_vision.stageScaleX10;
}

/* 返回最近长度拟合出的视觉yaw K；仅供显示/日志，不触发控制计算。 */
uint16_t Vision_GetYawGainQ1024(void)
{
    return g_vision.yawGainQ1024;
}

#endif
