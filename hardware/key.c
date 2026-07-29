/*
 * K1/K2按键驱动：GPIO边沿只唤醒Input任务，任务内完成消抖、短按和长按判定。
 * 事件存入固定长度静态队列；不在ISR中切换菜单、刷新LCD或控制电机。
 */
#include "key.h"

#include "board_config.h"
#include "pin_map.h"

/* 按键按住期间InputTask每1ms运行一次，短按至少确认一拍。 */
#define KEY_SHORT_PRESS_TICKS    (1U)
/* InputTask每1ms调用一次，800拍约800ms。 */
#define KEY_LONG_PRESS_TICKS     (800U)
#define KEY_EVENT_MASK_1         (0x01U)
#define KEY_EVENT_MASK_2         (0x02U)
#define KEY_EVENT_MASK_1_LONG    (0x04U)
#define KEY_EVENT_MASK_2_LONG    (0x08U)

static volatile uint8_t g_keyEventMask;
static volatile uint16_t g_keyPressedTicks[KEY_ID_COUNT];
static volatile uint8_t g_keyLongReported[KEY_ID_COUNT];
static volatile uint8_t g_keyWasPressed[KEY_ID_COUNT];

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

    DL_GPIO_initDigitalInputFeatures(PIN_KEY_1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, resistor,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(PIN_KEY_2_IOMUX,
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
    DL_GPIO_setLowerPinsPolarity(PIN_KEY_PORT,
        PIN_KEY_1_EDGE_RISE_FALL | PIN_KEY_2_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(PIN_KEY_PORT, PIN_KEY_1 | PIN_KEY_2);
    DL_GPIO_enableInterrupt(PIN_KEY_PORT, PIN_KEY_1 | PIN_KEY_2);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Key_Task(void)
{
    uint8_t i;

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

uint8_t Key_HasPendingEvent(void)
{
    uint8_t hasEvent;
    uint32_t primask = Key_EnterCritical();

    hasEvent = (g_keyEventMask != 0U) ? 1U : 0U;
    Key_ExitCritical(primask);
    return hasEvent;
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

uint8_t Key_HandleGPIOInterrupt(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(PIN_KEY_PORT,
        PIN_KEY_1 | PIN_KEY_2);

    if (((pending & PIN_KEY_1) != 0U) &&
        (Key_IsPressed(KEY_ID_1) == 0U)) {
        Key_HandleReleaseEdge(KEY_ID_1);
    }
    if (((pending & PIN_KEY_2) != 0U) &&
        (Key_IsPressed(KEY_ID_2) == 0U)) {
        Key_HandleReleaseEdge(KEY_ID_2);
    }
    if (pending != 0U) {
        DL_GPIO_clearInterruptStatus(PIN_KEY_PORT, pending);
    }
    return (pending != 0U) ? 1U : 0U;
}
