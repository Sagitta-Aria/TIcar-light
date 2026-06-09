#include "gimbal_test.h"

#include "board_config.h"
#include "gimbal.h"
#include "link.h"
#include "log_uart.h"
#include "motor_enable.h"

#define GIMBAL_TEST_LINE_SIZE        (64U)
#define GIMBAL_TEST_MAX_VALUES       (4U)
#define GIMBAL_TEST_ERROR_SCALE      (10)
#define GIMBAL_TEST_STATUS_LOG_TICKS (25U)
#define GIMBAL_TEST_BAD_LINE_LOG_PERIOD (25U)

typedef struct {
    /* running：菜单进入 Vision Test 后置 1，退出时清 0。 */
    uint8_t running;
    /* hasVision：至少成功解析过一帧视觉坐标后置 1，用于 OLED 显示状态。 */
    uint8_t hasVision;
    /* hasLinkLine：UART3 至少弹出过一行，哪怕格式暂时没解析成功也说明链路活着。 */
    uint8_t hasLinkLine;
    /* frameCount：成功解析并应用的视觉帧数，后续串口调试时可读取。 */
    uint32_t frameCount;
    /* badFrameCount：数字数量不足的坏帧数量，用于判断协议或波特率是否异常。 */
    uint32_t badFrameCount;
    /* rawX/rawY：当前选中的视觉误差，单位为 0.1 像素。 */
    int16_t rawX;
    int16_t rawY;
    /* statusLogTicks：等待视觉数据时的低频状态日志分频。 */
    uint16_t statusLogTicks;
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

/*
 * 作用：判断一个字符是不是十进制数字。
 * 使用场景：视觉行解析器跳过逗号、空格、冒号等分隔符后找数字。
 */
static uint8_t GimbalTest_IsDigit(char value)
{
    return ((value >= '0') && (value <= '9')) ? 1U : 0U;
}

/* 作用：计算 C 字符串长度，避免坏帧含控制字符时只看到空日志。 */
static uint16_t GimbalTest_StringLength(const char *text)
{
    uint16_t length = 0U;

    while ((text != 0) && (text[length] != '\0')) {
        ++length;
    }
    return length;
}

/*
 * 作用：低频打印解析失败的原始行长度和前几个字节。
 * 使用场景：判断队列里弹出的是空行、控制字符，还是协议格式不匹配。
 */
static void GimbalTest_LogBadLine(const char *line)
{
    uint16_t length = GimbalTest_StringLength(line);
    uint16_t i;

    LOG_RAW("[GIMBAL BAD] len=");
    LogUart_SendUnsigned(length);
    LOG_RAW(" hex=");
    for (i = 0U; (i < length) && (i < 8U); ++i) {
        LogUart_SendHex32((uint32_t)(uint8_t)line[i]);
        LOG_RAW(" ");
    }
    LOG_RAW(" text=");
    LOG_LINE(line);
}

/*
 * 作用：从一行视觉数据里提取最多四个误差值，转成 0.1 像素单位。
 * 格式：兼容小数，例如 "12.5,-8.0;30.2,15.7"。
 * 说明：输入按 10 倍放大；10 -> 100，12.34 -> 123，-0.9 -> -9。
 */
static uint8_t GimbalTest_ParseValues(const char *line,
    int16_t values[GIMBAL_TEST_MAX_VALUES])
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
        (count < GIMBAL_TEST_MAX_VALUES)) {
        /* 跳过所有非数字、非负号字符，因此 "x=10,y=-5" 也能解析。 */
        while ((*line != '\0') && (*line != '-') &&
            !GimbalTest_IsDigit(*line)) {
            ++line;
        }
        if (*line == '\0') {
            break;
        }

        /* 记录负号；如果负号后面既不是数字也不是小数点，则继续找下一段数字。 */
        negative = 0U;
        if (*line == '-') {
            negative = 1U;
            ++line;
        }
        if (!GimbalTest_IsDigit(*line) && (*line != '.')) {
            continue;
        }

        value = 0;
        hasInteger = 0U;
        hasFraction = 0U;
        roundUp = 0U;
        limit = negative ? 32768 : 32767;
        while (GimbalTest_IsDigit(*line)) {
            hasInteger = 1U;
            value = (value * 10) + (int32_t)(*line - '0');
            /* 防止异常长数字溢出，后面再统一夹到 int16_t 范围。 */
            if (value > limit) {
                value = limit;
            }
            ++line;
        }
        firstFractionDigit = 0U;
        secondFractionDigit = 0U;
        if (*line == '.') {
            ++line;
            if (GimbalTest_IsDigit(*line)) {
                hasFraction = 1U;
                firstFractionDigit = (uint8_t)(*line - '0');
                ++line;
                if (GimbalTest_IsDigit(*line)) {
                    secondFractionDigit = (uint8_t)(*line - '0');
                    roundUp = (secondFractionDigit >= 5U) ? 1U : 0U;
                    ++line;
                }
                while (GimbalTest_IsDigit(*line)) {
                    ++line;
                }
            }
        }
        if (!hasInteger && (hasFraction == 0U)) {
            continue;
        }
        value = (value * GIMBAL_TEST_ERROR_SCALE) +
            (int32_t)firstFractionDigit;
        if (value > limit) {
            value = limit;
        }
        if (roundUp && (value < limit)) {
            ++value;
        }
        if (negative) {
            value = -value;
        }
        values[count] = GimbalTest_ClampInt16(value);
        ++count;
    }

    return count;
}

/*
 * 作用：把解析出的数字映射成云台视觉输入。
 * 说明：新视觉格式为 "centerDx,centerDy;circleDx,circleDy"。
 *      每个 dx/dy 都是 160-x,120-y，符号已经是 target - current。
 */
static void GimbalTest_ApplyValues(const char *line, const int16_t *values,
    uint8_t count)
{
    uint8_t valueIndex = 0U;

    if (count >= 2U) {
        if (CAR_GIMBAL_TEST_USE_CIRCLE_ERROR != 0U) {
            if (count < 4U) {
                ++g_gimbalTest.badFrameCount;
                if ((g_gimbalTest.badFrameCount <= 5U) ||
                    ((g_gimbalTest.badFrameCount %
                        GIMBAL_TEST_BAD_LINE_LOG_PERIOD) == 0U)) {
                    GimbalTest_LogBadLine(line);
                }
                return;
            }
            valueIndex = 2U;
        }

        g_gimbalTest.rawX = values[valueIndex];
        g_gimbalTest.rawY = values[valueIndex + 1U];
        Gimbal_UpdateFromCameraError(g_gimbalTest.rawX, g_gimbalTest.rawY);
        g_gimbalTest.hasVision = 1U;
        ++g_gimbalTest.frameCount;
        return;
    }

    ++g_gimbalTest.badFrameCount;
    if ((g_gimbalTest.badFrameCount <= 5U) ||
        ((g_gimbalTest.badFrameCount % GIMBAL_TEST_BAD_LINE_LOG_PERIOD) ==
            0U)) {
        GimbalTest_LogBadLine(line);
    }
}

/*
 * 作用：低频打印视觉测试状态。
 * 使用场景：排查“PB3 能收到 HB，但进入 Gimbal 后又没了”的硬件/调度边界。
 */
static void GimbalTest_LogStatus(const char *tag)
{
    LOG_RAW("[GIMBAL LINK] ");
    LOG_RAW(tag);
    LOG_RAW(" rx=");
    LogUart_SendUnsigned(Link_GetRxLineCount());
    LOG_RAW(" overwrite=");
    LogUart_SendUnsigned(Link_GetRxOverwriteCount());
    LOG_RAW(" byte=");
    LogUart_SendUnsigned(Link_GetRxByteCount());
    LOG_RAW(" build=");
    LogUart_SendUnsigned(Link_GetRxBuildLength());
    LOG_RAW(" err=");
    LogUart_SendUnsigned(Link_GetRxErrorCount());
    LOG_RAW(" fe=");
    LogUart_SendUnsigned(Link_GetRxFrameErrorCount());
    LOG_RAW(" ne=");
    LogUart_SendUnsigned(Link_GetRxNoiseErrorCount());
    LOG_RAW(" frame=");
    LogUart_SendUnsigned(g_gimbalTest.frameCount);
    LOG_RAW(" bad=");
    LogUart_SendUnsigned(g_gimbalTest.badFrameCount);
    LOG_RAW(" dx0.1px=");
    LogUart_SendSigned(g_gimbalTest.rawX);
    LOG_RAW(" dy0.1px=");
    LogUart_SendSigned(g_gimbalTest.rawY);
    LOG_RAW(" ex0.1px=");
    LogUart_SendSigned(Gimbal_GetErrorX());
    LOG_RAW(" ey0.1px=");
    LogUart_SendSigned(Gimbal_GetErrorY());
    LOG_RAW(" cx=");
    LogUart_SendSigned(Gimbal_GetCommandX());
    LOG_RAW(" cy=");
    LogUart_SendSigned(Gimbal_GetCommandY());
    LOG_LINE("");
}

void GimbalTest_Init(void)
{
    g_gimbalTest.running = 0U;
    g_gimbalTest.hasVision = 0U;
    g_gimbalTest.hasLinkLine = 0U;
    g_gimbalTest.frameCount = 0U;
    g_gimbalTest.badFrameCount = 0U;
    g_gimbalTest.rawX = 0;
    g_gimbalTest.rawY = 0;
    g_gimbalTest.statusLogTicks = 0U;
}

/*
 * 作用：开始视觉测试。
 * 流程：清空历史 Link 数据，复位计数，把默认目标点写入云台，并启用闭环输出。
 */
void GimbalTest_Start(void)
{
    LOG_LINE("[GIMBAL LINK] start clear rx");
    Link_ClearRx();
    g_gimbalTest.running = 1U;
    g_gimbalTest.hasVision = 0U;
    g_gimbalTest.hasLinkLine = 0U;
    g_gimbalTest.frameCount = 0U;
    g_gimbalTest.badFrameCount = 0U;
    g_gimbalTest.rawX = 0;
    g_gimbalTest.rawY = 0;
    g_gimbalTest.statusLogTicks = 0U;
    LOG_LINE("[GIMBAL LINK] enable gimbal en");
    MotorEnable_SetGimbal(1U);
    LOG_LINE("[GIMBAL LINK] gimbal en ok");
    Gimbal_SetTarget(0, 0);
    Gimbal_SetEnabled(1U);
    GimbalTest_LogStatus("started");
}

/*
 * 作用：停止视觉测试。
 * 说明：关闭云台闭环时会停止两个云台轴，不影响底盘电机。
 */
void GimbalTest_Stop(void)
{
    if (g_gimbalTest.running != 0U) {
        MotorEnable_SetGimbal(CAR_STEPPER_ENABLE_DEFAULT_ON);
    }
    g_gimbalTest.running = 0U;
    g_gimbalTest.hasVision = 0U;
    g_gimbalTest.hasLinkLine = 0U;
    g_gimbalTest.statusLogTicks = 0U;
    Gimbal_SetEnabled(0U);
}

/*
 * 作用：主循环周期任务。
 * 说明：UART3 中断只负责收字节拼行，本函数在主循环里取完整行并解析。
 */
void GimbalTest_Task(void)
{
    char line[GIMBAL_TEST_LINE_SIZE];
    int16_t values[GIMBAL_TEST_MAX_VALUES];
    uint8_t valueCount;

    if (!g_gimbalTest.running) {
        return;
    }

    if (Link_PopLine(line, (uint16_t)sizeof(line))) {
        g_gimbalTest.hasLinkLine = 1U;
        valueCount = GimbalTest_ParseValues(line, values);
        GimbalTest_ApplyValues(line, values, valueCount);
    }

    ++g_gimbalTest.statusLogTicks;
    if (g_gimbalTest.statusLogTicks >= GIMBAL_TEST_STATUS_LOG_TICKS) {
        g_gimbalTest.statusLogTicks = 0U;
        if (g_gimbalTest.hasVision != 0U) {
            GimbalTest_LogStatus("tracking");
        } else if (g_gimbalTest.hasLinkLine != 0U) {
            GimbalTest_LogStatus("rx-bad");
        } else {
            GimbalTest_LogStatus("waiting");
        }
    }
}

uint8_t GimbalTest_IsRunning(void)
{
    return g_gimbalTest.running;
}

uint8_t GimbalTest_HasVision(void)
{
    return ((g_gimbalTest.hasVision != 0U) ||
        (g_gimbalTest.hasLinkLine != 0U)) ? 1U : 0U;
}

uint32_t GimbalTest_GetFrameCount(void)
{
    return g_gimbalTest.frameCount;
}

uint32_t GimbalTest_GetBadFrameCount(void)
{
    return g_gimbalTest.badFrameCount;
}
