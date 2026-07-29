/*
 * 七路灰度硬件抽象：按library_config选择GPIO数字输入或ADC模拟输入，并统一输出mask/误差。
 * 普通任务可调用Gray_Update；TIMG0 ISR应走Gray_ReadDigitalMaskFast，避免ADC等待和复杂滤波。
 * 数字模式不使用模拟阈值校准，但保留兼容接口以便快速切换比赛传感器方案。
 */
#include "library_config.h"

#if CAR_LIBRARY_GRAY_INPUT_ENABLED

#include "gray.h"

#include "board_config.h"
#include "pin_map.h"

#define GRAY_CALIBRATION_MIN_SPAN   (64U)

#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
#if GRAY_DIGITAL_INPUT_PULL_UP
#define GRAY_DIGITAL_RESISTOR       DL_GPIO_RESISTOR_PULL_UP
#else
#define GRAY_DIGITAL_RESISTOR       DL_GPIO_RESISTOR_NONE
#endif
#endif

#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint32_t iomux;
} GrayDigitalSlot;
#else
typedef struct {
    ADC12_Regs *adc;
    DL_ADC12_MEM_IDX mem;
} GrayAdcSlot;
#endif

#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
/* g_grayDigitalMap：把从左到右的 7 路数字灰度输出映射到 GPIO。 */
static const GrayDigitalSlot g_grayDigitalMap[GRAY_SENSOR_COUNT] = {
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_1, PIN_GRAY_1_IOMUX},
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_2, PIN_GRAY_2_IOMUX},
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_3, PIN_GRAY_3_IOMUX},
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_4, PIN_GRAY_4_IOMUX},
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_5, PIN_GRAY_5_IOMUX},
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_6, PIN_GRAY_6_IOMUX},
    {PIN_GRAY_DIGITAL_PORT, PIN_GRAY_7, PIN_GRAY_7_IOMUX},
};
#else
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
#endif

/* g_grayWeight：线路位置权重，当前实车左右反接，S5~S7 在车体右侧。 */
static const int16_t g_grayWeight[GRAY_SENSOR_COUNT] = {
    -1, -1, -1, 0, 1, 1, 1
};

/* g_grayRaw：最新原始采样值；数字模式下只会是 0 或 4095。 */
static uint16_t g_grayRaw[GRAY_SENSOR_COUNT];

/* g_grayDigital：每一路防抖后的黑白结果。 */
static uint8_t g_grayDigital[GRAY_SENSOR_COUNT];

/* g_grayThreshold：模拟模式阈值；数字模式下保留但不参与黑白判断。 */
static uint16_t g_grayThreshold[GRAY_SENSOR_COUNT];

/* g_grayMin/g_grayMax：校准时记录的最小值和最大值。 */
static uint16_t g_grayMin[GRAY_SENSOR_COUNT];
static uint16_t g_grayMax[GRAY_SENSOR_COUNT];

/* g_grayCalibrationComplete：校准完成状态位，供 OLED 菜单只读显示。 */
static uint8_t g_grayCalibrationComplete;

/* g_grayCandidate：等待确认的候选黑白状态。 */
static uint8_t g_grayCandidate[GRAY_SENSOR_COUNT];

/* g_grayConfirmCount：候选状态连续出现的次数。 */
static uint8_t g_grayConfirmCount[GRAY_SENSOR_COUNT];

/* g_grayMask：把黑白结果压缩成 bit6~bit0，对应 S1~S7。 */
static uint8_t g_grayMask;

/* g_grayValid：上一轮灰度更新是否成功。 */
static uint8_t g_grayValid;

/* g_grayFilterReady：第一次采样后，黑白防抖才开始生效。 */
static uint8_t g_grayFilterReady;

static uint8_t Gray_IsValidChannel(GrayChannel channel)
{
    return ((uint32_t)channel < GRAY_SENSOR_COUNT) ? 1U : 0U;
}

/* 作用：把 S1~S7 的数组序号映射成 bit6~bit0。 */
static uint8_t Gray_BitForIndex(uint32_t index)
{
    return (uint8_t)(1U << ((GRAY_SENSOR_COUNT - 1U) - index));
}

/* 作用：判断某一路灰度是否参与循迹计算。 */
static uint8_t Gray_IsTrackSensorEnabled(uint32_t index)
{
    return ((CAR_GRAY_TRACK_SENSOR_MASK & Gray_BitForIndex(index)) != 0U) ?
        1U : 0U;
}

/*
 * 作用：刷新灰度校准完成状态位。
 * 使用场景：校准采样更新 min/max 后调用。
 * 说明：这里只判断每一路都看到过高低变化，不负责保存阈值。
 */
static void Gray_UpdateCalibrationComplete(void)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (Gray_IsTrackSensorEnabled(i) == 0U) {
            continue;
        }
        if ((uint32_t)g_grayMax[i] <=
            ((uint32_t)g_grayMin[i] + GRAY_CALIBRATION_MIN_SPAN)) {
            g_grayCalibrationComplete = 0U;
            return;
        }
    }

    g_grayCalibrationComplete = 1U;
}

static void Gray_LoadDefaultThresholds(void)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        g_grayThreshold[i] = GRAY_DEFAULT_THRESHOLD;
    }
}

#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
/*
 * 作用：把灰度引脚切成数字 GPIO 输入。
 * 使用场景：模块已经输出 0/1 黑白结果时。
 */
static void Gray_ConfigDigitalInputs(void)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        DL_GPIO_initDigitalInputFeatures(g_grayDigitalMap[i].iomux,
            DL_GPIO_INVERSION_DISABLE, GRAY_DIGITAL_RESISTOR,
            DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    }
}

/*
 * 作用：读取某一路数字灰度是否压线。
 * 使用场景：Gray_Update 里把模块输出的数字量转换成 g_grayDigital。
 */
static uint8_t Gray_ReadDigitalActive(uint32_t index)
{
    uint8_t levelHigh =
        ((DL_GPIO_readPins(g_grayDigitalMap[index].port,
             g_grayDigitalMap[index].pin) & g_grayDigitalMap[index].pin) !=
            0U) ? 1U : 0U;

#if GRAY_DIGITAL_ACTIVE_HIGH
    return levelHigh;
#else
    return (levelHigh == 0U) ? 1U : 0U;
#endif
}

#else
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
#endif

/*
 * 作用：把某一路原始值和阈值转换成 0/1 黑白状态。
 * 使用场景：Gray_Update 里的原始值转状态。
 * 不要用于：需要强度信息的循迹算法。
 */
static uint8_t Gray_RawToDigital(uint16_t raw, uint16_t threshold)
{
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    (void)threshold;
    return (raw != 0U) ? 1U : 0U;
#else
#if GRAY_ACTIVE_HIGH
    return (raw >= threshold) ? 1U : 0U;
#else
    return (raw < threshold) ? 1U : 0U;
#endif
#endif
}

static uint16_t Gray_GetLineStrength(uint32_t index)
{
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    return (g_grayDigital[index] != 0U) ? 1U : 0U;
#else
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
#endif
}

/*
 * 作用：重新拼接 7 路黑白状态的位图。
 * 使用场景：任何黑白状态更新后。
 * 不要用于：原始采样更新阶段还没完成时。
 */
static void Gray_RebuildMask(void)
{
    uint32_t i;

    g_grayMask = 0U;
    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if ((Gray_IsTrackSensorEnabled(i) != 0U) && g_grayDigital[i]) {
            g_grayMask |= Gray_BitForIndex(i);
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
        if (Gray_IsTrackSensorEnabled(i) == 0U) {
            g_grayDigital[i] = 0U;
            g_grayCandidate[i] = 0U;
            g_grayConfirmCount[i] = GRAY_DIGITAL_CONFIRM_COUNT;
            continue;
        }

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
            if (g_grayConfirmCount[i] >= GRAY_DIGITAL_CONFIRM_COUNT) {
                g_grayDigital[i] = sampleDigital;
            }
        }
    }

    g_grayFilterReady = 1U;
    Gray_RebuildMask();
}

/*
 * 作用：初始化灰度传感器模块。
 * 使用场景：Board_Init 后、进入循迹前调用一次。
 * 说明：会清掉滤波状态、加载默认阈值、重置校准数据并立即采样一次。
 * 数字模式下阈值不参与判断，模块输出电平就是最终黑白结果。
 */
void Gray_Init(void)
{
    g_grayFilterReady = 0U;
    Gray_LoadDefaultThresholds();
    Gray_CalibrationReset();
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    Gray_ConfigDigitalInputs();
#else
    DL_ADC12_enableConversions(PIN_GRAY_ADC0);
    DL_ADC12_enableConversions(PIN_GRAY_ADC1);
#endif
    (void)Gray_Update();
}

void Gray_StartConversion(void)
{
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    /* 数字灰度模式下没有 ADC 转换需要启动。 */
#else
    if (!DL_ADC12_isConversionStarted(PIN_GRAY_ADC0)) {
        DL_ADC12_startConversion(PIN_GRAY_ADC0);
    }
    if (!DL_ADC12_isConversionStarted(PIN_GRAY_ADC1)) {
        DL_ADC12_startConversion(PIN_GRAY_ADC1);
    }
#endif
}

/*
 * 作用：执行一次灰度采样流程，完成平均滤波和黑白防抖。
 * 使用场景：循迹主循环、校准采样、调试读取。
 * 不要用于：只读某一路静态值且不希望触发全套更新的场景。
 * 说明：每次会采样 GRAY_FILTER_SAMPLE_COUNT 次再求平均。
 */
uint8_t Gray_Update(void)
{
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (Gray_IsTrackSensorEnabled(i) == 0U) {
            g_grayRaw[i] = 0U;
            continue;
        }
        g_grayRaw[i] = (Gray_ReadDigitalActive(i) != 0U) ?
            GRAY_ADC_MAX_VALUE : 0U;
    }

    Gray_UpdateDigitalFromRaw();
    g_grayValid = 1U;
    return 1U;
#else
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
        if (Gray_IsTrackSensorEnabled(i) == 0U) {
            g_grayRaw[i] = 0U;
        }
    }

    Gray_UpdateDigitalFromRaw();
    g_grayValid = 1U;
    return 1U;
#endif
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

uint8_t Gray_ReadDigitalMaskFast(void)
{
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    uint8_t mask = 0U;
    uint32_t pins = DL_GPIO_readPins(PIN_GRAY_DIGITAL_PORT,
        PIN_GRAY_1 | PIN_GRAY_2 | PIN_GRAY_3 | PIN_GRAY_4 |
        PIN_GRAY_5 | PIN_GRAY_6 | PIN_GRAY_7);

#if GRAY_DIGITAL_ACTIVE_HIGH
    if ((pins & PIN_GRAY_1) != 0U) { mask |= 0x40U; }
    if ((pins & PIN_GRAY_2) != 0U) { mask |= 0x20U; }
    if ((pins & PIN_GRAY_3) != 0U) { mask |= 0x10U; }
    if ((pins & PIN_GRAY_4) != 0U) { mask |= 0x08U; }
    if ((pins & PIN_GRAY_5) != 0U) { mask |= 0x04U; }
    if ((pins & PIN_GRAY_6) != 0U) { mask |= 0x02U; }
    if ((pins & PIN_GRAY_7) != 0U) { mask |= 0x01U; }
#else
    if ((pins & PIN_GRAY_1) == 0U) { mask |= 0x40U; }
    if ((pins & PIN_GRAY_2) == 0U) { mask |= 0x20U; }
    if ((pins & PIN_GRAY_3) == 0U) { mask |= 0x10U; }
    if ((pins & PIN_GRAY_4) == 0U) { mask |= 0x08U; }
    if ((pins & PIN_GRAY_5) == 0U) { mask |= 0x04U; }
    if ((pins & PIN_GRAY_6) == 0U) { mask |= 0x02U; }
    if ((pins & PIN_GRAY_7) == 0U) { mask |= 0x01U; }
#endif

    mask = (uint8_t)(mask & CAR_GRAY_TRACK_SENSOR_MASK);
    g_grayMask = mask;
    return mask;
#else
    (void)Gray_Update();
    return g_grayMask;
#endif
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
        if ((Gray_IsTrackSensorEnabled(i) != 0U) && g_grayDigital[i]) {
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
 * 作用：根据灰度强度计算加权循迹偏差。
 * 使用场景：正式循迹主逻辑，配合左右电机差速控制。
 * 不要用于：只想看某一路是否亮/灭的简单判断。
 * 说明：中间值接近 0，越偏左越负，越偏右越正；数字模式下每路有效权重相同。
 */
uint8_t Gray_GetWeightedLineError(int16_t *error)
{
    int32_t weightedSum = 0;
    uint32_t strengthSum = 0U;
    uint32_t i;
    uint16_t strength;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        if (Gray_IsTrackSensorEnabled(i) == 0U) {
            continue;
        }
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
 * 说明：数字灰度模式不使用该阈值，保留接口只是为了兼容模拟模式。
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
 * 作用：把当前一轮滤波后的原始值复制出来。
 * 使用场景：调试、打印、临时查看原始传感器数据。
 * 说明：数字模式下返回值为 0 或 4095，便于沿用原有串口打印格式。
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
    g_grayCalibrationComplete = 0U;
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
    Gray_UpdateCalibrationComplete();
}

uint8_t Gray_IsCalibrationComplete(void)
{
    return g_grayCalibrationComplete;
}

/*
 * 作用：应用灰度校准结果。
 * 使用场景：校准采样结束后统一应用。
 * 说明：数字模式下模块已经完成比较，这里只刷新状态，不再生成阈值。
 */
void Gray_CalibrationApply(void)
{
#if CAR_LIBRARY_GRAY_INPUT_IS_DIGITAL
    if (g_grayValid) {
        Gray_UpdateDigitalFromRaw();
    }
#else
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
#endif
}

#endif /* CAR_LIBRARY_GRAY_INPUT_ENABLED */
