#include "gray.h"

#include "log_uart.h"
#include "pin_map.h"

typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;

/* g_grayMap：把从左到右的 7 路传感器映射到对应 ADC 和 MEM 槽位。 */
static const GrayAdcSlot g_grayMap[GRAY_SENSOR_COUNT] = {
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY1},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY2},
    {PIN_GRAY_ADC1, GRAY_ADC1_MEM_GRAY3},
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY4},
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY5},
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY6},
    {PIN_GRAY_ADC0, GRAY_ADC0_MEM_GRAY7},
};

/* g_grayWeight：线路位置权重，负数表示偏左，正数表示偏右。 */
static const int16_t g_grayWeight[GRAY_SENSOR_COUNT] = {
    -3, -2, -1, 0, 1, 2, 3
};

/* g_grayRaw：最新的平均 ADC 原始值。 */
static uint16_t g_grayRaw[GRAY_SENSOR_COUNT];

/* g_grayDigital：每一路防抖后的黑白结果。 */
static uint8_t g_grayDigital[GRAY_SENSOR_COUNT];

/* g_grayThreshold：每一路的阈值。 */
static uint16_t g_grayThreshold[GRAY_SENSOR_COUNT];

/* g_grayMin/g_grayMax：校准时记录的最小值和最大值。 */
static uint16_t g_grayMin[GRAY_SENSOR_COUNT];
static uint16_t g_grayMax[GRAY_SENSOR_COUNT];

/* g_grayCandidate：等待确认的候选黑白状态。 */
static uint8_t g_grayCandidate[GRAY_SENSOR_COUNT];

/* g_grayConfirmCount：候选状态连续出现的次数。 */
static uint8_t g_grayConfirmCount[GRAY_SENSOR_COUNT];

/* g_grayMask：把黑白结果压缩成 bit0~bit6。 */
static uint8_t g_grayMask;

/* g_grayValid：上一轮 ADC 更新是否成功。 */
static uint8_t g_grayValid;

/* g_grayFilterReady：第一次采样后，黑白防抖才开始生效。 */
static uint8_t g_grayFilterReady;

static uint8_t Gray_IsValidChannel(GrayChannel channel)
{
    return ((uint32_t)channel < GRAY_SENSOR_COUNT) ? 1U : 0U;
}

static void Gray_LoadDefaultThresholds(void)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        g_grayThreshold[i] = GRAY_DEFAULT_THRESHOLD;
    }
}

/*
 * 作用：等待 ADC 序列完成。
 * 使用场景：Gray_Update 里检查 ADC0/ADC1 是否采样结束。
 * 不要用于：你只想做一次非阻塞状态查询的时候。
 * 说明：带超时保护，避免外设异常时卡死主循环。
 */
static uint8_t Gray_WaitDone(ADC12_Regs *adc)
{
    uint32_t timeout = GRAY_ADC_TIMEOUT_COUNT;

    while (DL_ADC12_isConversionStarted(adc)) {
        if (timeout == 0U) {
            DL_ADC12_stopConversion(adc);
            return 0U;
        }
        --timeout;
    }
    return 1U;
}

/*
 * 作用：读取一轮原始 ADC 值。
 * 使用场景：Gray_Update 的内部采样步骤。
 * 不要用于：只想读取某一路固定值的简单调试场景。
 * 说明：会启动一次 ADC 序列并等待完成。
 */
static uint8_t Gray_ReadRawOnce(uint16_t values[GRAY_SENSOR_COUNT])
{
    uint32_t i;

    Gray_StartConversion();
    if (!Gray_WaitDone(PIN_GRAY_ADC0)) {
        return 0U;
    }
    if (!Gray_WaitDone(PIN_GRAY_ADC1)) {
        return 0U;
    }

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        values[i] =
            DL_ADC12_getMemResult(g_grayMap[i].adc, g_grayMap[i].mem);
    }

    return 1U;
}

/*
 * 作用：把某一路原始值和阈值转换成 0/1 黑白状态。
 * 使用场景：Gray_Update 里的原始值转状态。
 * 不要用于：需要强度信息的循迹算法。
 */
static uint8_t Gray_RawToDigital(uint16_t raw, uint16_t threshold)
{
#if GRAY_ACTIVE_HIGH
    return (raw >= threshold) ? 1U : 0U;
#else
    return (raw < threshold) ? 1U : 0U;
#endif
}

static uint16_t Gray_GetLineStrength(uint32_t index)
{
    uint16_t raw;
    uint16_t threshold;

    if (g_grayDigital[index] == 0U) {
        return 0U;
    }

    raw = g_grayRaw[index];
    threshold = g_grayThreshold[index];

#if GRAY_ACTIVE_HIGH
    return (raw > threshold) ? (uint16_t)(raw - threshold) : 1U;
#else
    return (raw < threshold) ? (uint16_t)(threshold - raw) : 1U;
#endif
}

/*
 * 作用：重新拼接 7 路黑白状态的位图。
 * 使用场景：任何黑白状态更新后。
 * 不要用于：原始 ADC 更新阶段还没完成时。
 */
static void Gray_RebuildMask(void)
{
    uint32_t i;

    g_grayMask = 0U;
    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (g_grayDigital[i]) {
            g_grayMask |= (uint8_t)(1U << i);
        }
    }
}

/*
 * 作用：把滤波后的原始值刷新成黑白状态，并做连续确认防抖。
 * 使用场景：Gray_Update 之后和阈值/校准更新之后。
 * 说明：第一次进入时直接接受当前状态，后续变化需连续确认。
 */
static void Gray_UpdateDigitalFromRaw(void)
{
    uint32_t i;
    uint8_t sampleDigital;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        sampleDigital = Gray_RawToDigital(g_grayRaw[i], g_grayThreshold[i]);

        if (!g_grayFilterReady) {
            g_grayDigital[i] = sampleDigital;
            g_grayCandidate[i] = sampleDigital;
            g_grayConfirmCount[i] = GRAY_DIGITAL_CONFIRM_COUNT;
            continue;
        }

        if (sampleDigital == g_grayDigital[i]) {
            g_grayCandidate[i] = sampleDigital;
            g_grayConfirmCount[i] = GRAY_DIGITAL_CONFIRM_COUNT;
        } else if (sampleDigital == g_grayCandidate[i]) {
            if (g_grayConfirmCount[i] < GRAY_DIGITAL_CONFIRM_COUNT) {
                ++g_grayConfirmCount[i];
            }
            if (g_grayConfirmCount[i] >= GRAY_DIGITAL_CONFIRM_COUNT) {
                g_grayDigital[i] = sampleDigital;
            }
        } else {
            g_grayCandidate[i] = sampleDigital;
            g_grayConfirmCount[i] = 1U;
        }
    }

    g_grayFilterReady = 1U;
    Gray_RebuildMask();
}

/*
 * 作用：初始化灰度传感器模块。
 * 使用场景：Board_Init 后、进入循迹前调用一次。
 * 说明：会清掉滤波状态、加载默认阈值、重置校准数据并立即采样一次。
 */
void Gray_Init(void)
{
    g_grayFilterReady = 0U;
    Gray_LoadDefaultThresholds();
    Gray_CalibrationReset();
    DL_ADC12_enableConversions(PIN_GRAY_ADC0);
    DL_ADC12_enableConversions(PIN_GRAY_ADC1);
    (void)Gray_Update();
}

void Gray_StartConversion(void)
{
    if (!DL_ADC12_isConversionStarted(PIN_GRAY_ADC0)) {
        DL_ADC12_startConversion(PIN_GRAY_ADC0);
    }
    if (!DL_ADC12_isConversionStarted(PIN_GRAY_ADC1)) {
        DL_ADC12_startConversion(PIN_GRAY_ADC1);
    }
}

/*
 * 作用：执行一次灰度采样流程，完成平均滤波和黑白防抖。
 * 使用场景：循迹主循环、校准采样、调试读取。
 * 不要用于：只读某一路静态值且不希望触发全套更新的场景。
 * 说明：每次会采样 GRAY_FILTER_SAMPLE_COUNT 次再求平均。
 */
uint8_t Gray_Update(void)
{
    uint16_t rawOnce[GRAY_SENSOR_COUNT];
    uint32_t sum[GRAY_SENSOR_COUNT];
    uint32_t i;
    uint32_t sample;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        sum[i] = 0U;
    }

    for (sample = 0U; sample < GRAY_FILTER_SAMPLE_COUNT; ++sample) {
        if (!Gray_ReadRawOnce(rawOnce)) {
            g_grayValid = 0U;
            return 0U;
        }
        for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
            sum[i] += rawOnce[i];
        }
    }

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        g_grayRaw[i] =
            (uint16_t)((sum[i] + (GRAY_FILTER_SAMPLE_COUNT / 2U)) /
                GRAY_FILTER_SAMPLE_COUNT);
    }

    Gray_UpdateDigitalFromRaw();
    g_grayValid = 1U;
    return 1U;
}

uint16_t Gray_GetRaw(GrayChannel channel)
{
    if (!Gray_IsValidChannel(channel)) {
        return 0U;
    }
    return g_grayRaw[channel];
}

uint8_t Gray_GetDigital(GrayChannel channel)
{
    if (!Gray_IsValidChannel(channel)) {
        return 0U;
    }
    return g_grayDigital[channel];
}

uint8_t Gray_GetDigitalMask(void)
{
    return g_grayMask;
}

/*
 * 作用：根据 0/1 黑白状态计算粗略循迹偏差。
 * 使用场景：简单循迹、调试对比、观察传感器是否接线正确。
 * 不要用于：需要更细腻转向控制的最终循迹主逻辑。
 */
uint8_t Gray_GetLineError(int16_t *error)
{
    int16_t sum = 0;
    uint8_t activeCount = 0U;
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (g_grayDigital[i]) {
            sum = (int16_t)(sum + g_grayWeight[i]);
            ++activeCount;
        }
    }

    if (activeCount == 0U) {
        if (error != 0) {
            *error = 0;
        }
        return 0U;
    }

    if (error != 0) {
        *error = (int16_t)(sum / (int16_t)activeCount);
    }
    return 1U;
}

/*
 * 作用：根据 ADC 强度计算加权循迹偏差。
 * 使用场景：正式循迹主逻辑，配合左右电机差速控制。
 * 不要用于：只想看某一路是否亮/灭的简单判断。
 * 说明：中间值接近 0，越偏左越负，越偏右越正。
 */
uint8_t Gray_GetWeightedLineError(int16_t *error)
{
    int32_t weightedSum = 0;
    uint32_t strengthSum = 0U;
    uint32_t i;
    uint16_t strength;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        strength = Gray_GetLineStrength(i);
        if (strength > 0U) {
            weightedSum += (int32_t)g_grayWeight[i] * (int32_t)strength;
            strengthSum += strength;
        }
    }

    if (strengthSum == 0U) {
        if (error != 0) {
            *error = 0;
        }
        return 0U;
    }

    if (error != 0) {
        *error = (int16_t)((weightedSum * GRAY_LINE_ERROR_SCALE) /
            (int32_t)strengthSum);
    }
    return 1U;
}

uint8_t Gray_IsLineLost(void)
{
    return (g_grayMask == 0U) ? 1U : 0U;
}

/*
 * 作用：手动修改某一路阈值。
 * 使用场景：你已经知道某一路应该更高或更低的阈值时。
 * 不要用于：还没确定传感器极性和黑白方向的时候。
 */
void Gray_SetThreshold(GrayChannel channel, uint16_t threshold)
{
    if (!Gray_IsValidChannel(channel)) {
        return;
    }
    if (threshold > GRAY_ADC_MAX_VALUE) {
        threshold = GRAY_ADC_MAX_VALUE;
    }
    g_grayThreshold[channel] = threshold;
    if (g_grayValid) {
        Gray_UpdateDigitalFromRaw();
    }
}

uint16_t Gray_GetThreshold(GrayChannel channel)
{
    if (!Gray_IsValidChannel(channel)) {
        return 0U;
    }
    return g_grayThreshold[channel];
}

/*
 * 作用：把当前一轮滤波后的 ADC 值复制出来。
 * 使用场景：调试、打印、临时查看原始传感器数据。
 */
void Gray_ReadRaw(uint16_t values[GRAY_SENSOR_COUNT])
{
    uint32_t i;

    if (values == 0) {
        return;
    }

    (void)Gray_Update();
    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        values[i] = g_grayRaw[i];
    }
}

/*
 * 作用：把当前一轮防抖后的黑白结果复制出来。
 * 使用场景：调试和状态观察。
 */
void Gray_ReadDigital(uint8_t values[GRAY_SENSOR_COUNT])
{
    uint32_t i;

    if (values == 0) {
        return;
    }

    (void)Gray_Update();
    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        values[i] = g_grayDigital[i];
    }
}

/*
 * 作用：重置校准用的最小/最大值。
 * 使用场景：开始做灰度阈值校准之前。
 */
void Gray_CalibrationReset(void)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        g_grayMin[i] = GRAY_ADC_MAX_VALUE;
        g_grayMax[i] = 0U;
    }
    LOG_LINE("gray calibration: reset");
}

/*
 * 作用：采一轮校准数据，更新每路最小/最大值。
 * 使用场景：拿着车放在黑线和白底之间移动时重复调用。
 */
void Gray_CalibrationSample(void)
{
    uint32_t i;

    if (!Gray_Update()) {
        return;
    }

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (g_grayRaw[i] < g_grayMin[i]) {
            g_grayMin[i] = g_grayRaw[i];
        }
        if (g_grayRaw[i] > g_grayMax[i]) {
            g_grayMax[i] = g_grayRaw[i];
        }
    }
}

/*
 * 作用：把校准得到的最小/最大值转成阈值。
 * 使用场景：校准采样结束后统一应用。
 */
void Gray_CalibrationApply(void)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (g_grayMax[i] > g_grayMin[i]) {
            g_grayThreshold[i] =
                (uint16_t)((g_grayMin[i] + g_grayMax[i]) / 2U);
        }
    }
    if (g_grayValid) {
        Gray_UpdateDigitalFromRaw();
    }
    LOG_LINE("gray calibration: apply");
}
