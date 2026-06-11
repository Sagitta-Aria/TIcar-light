#include "key.h"

#include "board_config.h"
#include "log_uart.h"
#include "pin_map.h"

/* 单次 GPIOB 中断最多处理的按键事件数，避免抖动异常时长时间停在 ISR。 */
#define KEY_IRQ_SERVICE_LIMIT    (8U)
/* 短按只要被主循环采到一次就认可，避免日志/视觉任务拖慢采样导致漏按。 */
#define KEY_SHORT_PRESS_TICKS    (1U)
/* 长按阈值：当前 10ms 一轮，80 轮约 800ms。 */
#define KEY_LONG_PRESS_TICKS     (80U)
#define KEY_EVENT_MASK_1         (0x01U)
#define KEY_EVENT_MASK_2         (0x02U)
#define KEY_EVENT_MASK_1_LONG    (0x04U)
#define KEY_EVENT_MASK_2_LONG    (0x08U)

static volatile uint8_t g_keyEventMask;
static volatile uint16_t g_keyPressedTicks[KEY_ID_COUNT];
static volatile uint8_t g_keyLongReported[KEY_ID_COUNT];
static volatile uint8_t g_keyWasPressed[KEY_ID_COUNT];
static uint8_t g_keyLastLevelHigh[KEY_ID_COUNT];
static uint16_t g_keyDebugTicks;

/*
 * 作用：保护主循环和 GPIO 中断共享的按键事件状态。
 * 使用场景：Key_Task / Key_PopEvent 与 Key_HandleGPIOInterrupt 并发访问时。
 */
static uint32_t Key_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Key_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint32_t Key_PinFromId(KeyId key)
{
    return (key == KEY_ID_1) ? PIN_KEY_1 : PIN_KEY_2;
}

static uint8_t Key_EventMaskFromId(KeyId key)
{
    return (key == KEY_ID_1) ? KEY_EVENT_MASK_1 : KEY_EVENT_MASK_2;
}

static uint8_t Key_LongEventMaskFromId(KeyId key)
{
    return (key == KEY_ID_1) ? KEY_EVENT_MASK_1_LONG :
        KEY_EVENT_MASK_2_LONG;
}

/* 作用：读取指定按键 GPIO 原始电平，1 表示高电平。 */
static uint8_t Key_ReadLevelHigh(KeyId key)
{
    uint32_t pin = Key_PinFromId(key);
    return ((DL_GPIO_readPins(PIN_KEY_PORT, pin) & pin) != 0U) ? 1U : 0U;
}

/* 作用：按配置把原始电平转换成“是否按下”。 */
static uint8_t Key_LevelToPressed(uint8_t levelHigh)
{
#if (CAR_KEY_ACTIVE_LOW != 0U)
    return (levelHigh == 0U) ? 1U : 0U;
#else
    return levelHigh;
#endif
}

/*
 * 作用：在按键释放中断里锁存短按。
 * 说明：KEY 输入当前配置为低有效、上升沿中断；这样很快的短按也不完全依赖主循环采样。
 */
static void Key_HandleReleaseEdge(KeyId key)
{
    if (g_keyLongReported[key] == 0U) {
        g_keyEventMask |= Key_EventMaskFromId(key);
    }
    g_keyPressedTicks[key] = 0U;
    g_keyLongReported[key] = 0U;
    g_keyWasPressed[key] = 0U;
}

/* 作用：打印 PB9/PB8 原始电平和按下判断，用来定位按键误触发。 */
static void Key_LogRawState(const char *tag)
{
#if CAR_KEY_DEBUG_LOG
    uint8_t k1High = Key_ReadLevelHigh(KEY_ID_1);
    uint8_t k2High = Key_ReadLevelHigh(KEY_ID_2);

    LOG_RAW("[KEY] ");
    LOG_RAW(tag);
    LOG_RAW(" PB9=");
    LOG_RAW(k1High ? "H" : "L");
    LOG_RAW(" PB8=");
    LOG_RAW(k2High ? "H" : "L");
    LOG_RAW(" pressed=");
    LOG_RAW(Key_LevelToPressed(k1High) ? "K1" : "-");
    LOG_RAW(",");
    LOG_LINE(Key_LevelToPressed(k2High) ? "K2" : "-");
#else
    (void)tag;
#endif
}

/* 作用：按周期和电平变化打印原始按键状态。 */
static void Key_DebugTask(void)
{
#if CAR_KEY_DEBUG_LOG
    uint8_t k1High = Key_ReadLevelHigh(KEY_ID_1);
    uint8_t k2High = Key_ReadLevelHigh(KEY_ID_2);

    if ((k1High != g_keyLastLevelHigh[KEY_ID_1]) ||
        (k2High != g_keyLastLevelHigh[KEY_ID_2])) {
        g_keyLastLevelHigh[KEY_ID_1] = k1High;
        g_keyLastLevelHigh[KEY_ID_2] = k2High;
        g_keyDebugTicks = 0U;
        Key_LogRawState("change");
        return;
    }

#if CAR_KEY_DEBUG_POLL_LOG
    if (g_keyDebugTicks < CAR_KEY_DEBUG_PERIOD_TICKS) {
        ++g_keyDebugTicks;
        return;
    }

    g_keyDebugTicks = 0U;
    Key_LogRawState("poll");
#endif
#endif
}

/*
 * 作用：按项目配置重新设置按键输入上下拉。
 * 说明：SysConfig 里原本是下拉高有效；这里用 board_config 覆盖，便于适配实物接法。
 */
static void Key_ConfigInputPins(void)
{
#if (CAR_KEY_ACTIVE_LOW != 0U)
    uint32_t resistor = DL_GPIO_RESISTOR_PULL_UP;
#else
    uint32_t resistor = DL_GPIO_RESISTOR_PULL_DOWN;
#endif

    DL_GPIO_initDigitalInputFeatures(KEY_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, resistor,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY_2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, resistor,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

void Key_Init(void)
{
    Key_ConfigInputPins();
    g_keyEventMask = 0U;
    g_keyPressedTicks[KEY_ID_1] = 0U;
    g_keyPressedTicks[KEY_ID_2] = 0U;
    g_keyLongReported[KEY_ID_1] = 0U;
    g_keyLongReported[KEY_ID_2] = 0U;
    g_keyWasPressed[KEY_ID_1] = 0U;
    g_keyWasPressed[KEY_ID_2] = 0U;
    g_keyLastLevelHigh[KEY_ID_1] = Key_ReadLevelHigh(KEY_ID_1);
    g_keyLastLevelHigh[KEY_ID_2] = Key_ReadLevelHigh(KEY_ID_2);
    g_keyDebugTicks = 0U;
    Key_LogRawState("init");
    DL_GPIO_clearInterruptStatus(PIN_KEY_PORT, PIN_KEY_1 | PIN_KEY_2);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Key_Task(void)
{
    uint8_t i;

    Key_DebugTask();

    /*
     * 按键事件统一在主循环里轮询生成：
     * - 短按：松手时上报，避免长按退出前先触发一次加减。
     * - 长按：达到阈值立刻上报一次，随后等松手复位。
     * GPIO 上升沿中断也会锁存短按，避免极快点按被主循环采样漏掉。
     */
    for (i = 0U; i < (uint8_t)KEY_ID_COUNT; ++i) {
        if (Key_IsPressed((KeyId)i) != 0U) {
            if (g_keyPressedTicks[i] < 0xFFFFU) {
                ++g_keyPressedTicks[i];
            }
            if ((g_keyPressedTicks[i] >= KEY_LONG_PRESS_TICKS) &&
                (g_keyLongReported[i] == 0U)) {
                uint32_t primask = Key_EnterCritical();
                g_keyEventMask |= Key_LongEventMaskFromId((KeyId)i);
                Key_ExitCritical(primask);
                g_keyLongReported[i] = 1U;
                Key_LogRawState("long");
            }
            g_keyWasPressed[i] = 1U;
        } else {
            if ((g_keyWasPressed[i] != 0U) &&
                (g_keyLongReported[i] == 0U) &&
                (g_keyPressedTicks[i] >= KEY_SHORT_PRESS_TICKS)) {
                uint32_t primask = Key_EnterCritical();
                g_keyEventMask |= Key_EventMaskFromId((KeyId)i);
                Key_ExitCritical(primask);
            }
            g_keyPressedTicks[i] = 0U;
            g_keyLongReported[i] = 0U;
            g_keyWasPressed[i] = 0U;
        }
    }
}

uint8_t Key_IsPressed(KeyId key)
{
    return Key_LevelToPressed(Key_ReadLevelHigh(key));
}

KeyEvent Key_PopEvent(void)
{
    KeyEvent event = KEY_EVENT_NONE;
    uint8_t mask;
    uint32_t primask = Key_EnterCritical();

    mask = g_keyEventMask;
    if ((mask & KEY_EVENT_MASK_1_LONG) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_1_LONG));
        event = KEY_EVENT_1_LONG;
    } else if ((mask & KEY_EVENT_MASK_2_LONG) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_2_LONG));
        event = KEY_EVENT_2_LONG;
    } else if ((mask & KEY_EVENT_MASK_1) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_1));
        event = KEY_EVENT_1;
    } else if ((mask & KEY_EVENT_MASK_2) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_2));
        event = KEY_EVENT_2;
    }

    Key_ExitCritical(primask);
    return event;
}

void Key_HandleGPIOInterrupt(void)
{
    DL_GPIO_IIDX pending;
    uint8_t serviceCount = 0U;

    do {
        pending = DL_GPIO_getPendingInterrupt(PIN_KEY_PORT);
        switch (pending) {
        case KEY_1_IIDX:
            Key_HandleReleaseEdge(KEY_ID_1);
            break;
        case KEY_2_IIDX:
            Key_HandleReleaseEdge(KEY_ID_2);
            break;
        default:
            break;
        }
        ++serviceCount;
    } while ((pending != DL_GPIO_IIDX_NO_INTR) &&
        (serviceCount < KEY_IRQ_SERVICE_LIMIT));
}
