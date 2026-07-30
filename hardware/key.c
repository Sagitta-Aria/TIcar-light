/*
 * 按键驱动：GPIO边沿只唤醒Input任务，按键松开时上报一次短按。
 * 事件存入固定长度静态队列；不在ISR中切换菜单、刷新LCD或控制电机。
 */
#include "key.h"

#include "board_config.h"
#include "pin_map.h"

#define KEY_EVENT_MASK_1         (0x01U)
#define KEY_EVENT_MASK_2         (0x02U)
#define KEY_EVENT_MASK_3         (0x04U)
#define KEY_EVENT_MASK_4         (0x08U)
#define KEY_EVENT_MASK_5         (0x10U)

static volatile uint8_t g_keyEventMask;
static volatile uint8_t g_keyWasPressed[KEY_ID_COUNT];

/* 保护Input任务和GPIO中断共享的按键事件状态。 */
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
    switch (key) {
    case KEY_ID_1:
        return PIN_KEY_1;
    case KEY_ID_2:
        return PIN_KEY_2;
#if CAR_LIBRARY_BOARD_IS_TIANMENG
    case KEY_ID_3:
        return PIN_KEY_3;
    case KEY_ID_4:
        return PIN_KEY_4;
    case KEY_ID_5:
        return PIN_KEY_5;
#endif
    default:
        return 0U;
    }
}

static uint8_t Key_EventMaskFromId(KeyId key)
{
    return (uint8_t)(1U << (uint8_t)key);
}

/* 读取指定按键GPIO原始电平，1表示高电平。 */
static uint8_t Key_ReadLevelHigh(KeyId key)
{
    uint32_t pin = Key_PinFromId(key);
    return ((DL_GPIO_readPins(PIN_KEY_PORT, pin) & pin) != 0U) ? 1U : 0U;
}

/* 按配置把原始电平转换成“是否按下”。 */
static uint8_t Key_LevelToPressed(uint8_t levelHigh)
{
#if (CAR_KEY_ACTIVE_LOW != 0U)
    return (levelHigh == 0U) ? 1U : 0U;
#else
    return levelHigh;
#endif
}

/* 在按键释放中断里锁存短按，使极快点按不依赖任务采样。 */
static void Key_HandleReleaseEdge(KeyId key)
{
    g_keyEventMask |= Key_EventMaskFromId(key);
    g_keyWasPressed[key] = 0U;
}

/* 按board_config设置有效电平；天猛星额外启用K3至K5。 */
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
#if CAR_LIBRARY_BOARD_IS_TIANMENG
    DL_GPIO_initDigitalInputFeatures(PIN_KEY_3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, resistor,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(PIN_KEY_4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, resistor,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(PIN_KEY_5_IOMUX,
        DL_GPIO_INVERSION_DISABLE, resistor,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
#endif
}

void Key_Init(void)
{
    uint8_t i;
    uint32_t keyPins = PIN_KEY_1 | PIN_KEY_2;

    Key_ConfigInputPins();
    g_keyEventMask = 0U;
    for (i = 0U; i < (uint8_t)KEY_ID_COUNT; ++i) {
        g_keyWasPressed[i] = 0U;
    }
#if CAR_LIBRARY_BOARD_IS_TIANMENG
    keyPins |= PIN_KEY_3 | PIN_KEY_4 | PIN_KEY_5;
#endif
    DL_GPIO_setLowerPinsPolarity(PIN_KEY_PORT,
        PIN_KEY_1_EDGE_RISE_FALL | PIN_KEY_2_EDGE_RISE_FALL
#if CAR_LIBRARY_BOARD_IS_TIANMENG
        | PIN_KEY_3_EDGE_RISE_FALL | PIN_KEY_4_EDGE_RISE_FALL
#endif
    );
#if CAR_LIBRARY_BOARD_IS_TIANMENG
    DL_GPIO_setUpperPinsPolarity(PIN_KEY_PORT,
        PIN_KEY_5_EDGE_RISE_FALL);
#endif
    DL_GPIO_clearInterruptStatus(PIN_KEY_PORT, keyPins);
    DL_GPIO_enableInterrupt(PIN_KEY_PORT, keyPins);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Key_Task(void)
{
    uint8_t i;

    /* 释放边沿会直接锁存事件；轮询路径用于补偿极端情况下的边沿丢失。 */
    for (i = 0U; i < (uint8_t)KEY_ID_COUNT; ++i) {
        if (Key_IsPressed((KeyId)i) != 0U) {
            g_keyWasPressed[i] = 1U;
        } else if (g_keyWasPressed[i] != 0U) {
            uint32_t primask = Key_EnterCritical();
            g_keyEventMask |= Key_EventMaskFromId((KeyId)i);
            Key_ExitCritical(primask);
            g_keyWasPressed[i] = 0U;
        }
    }
}

uint8_t Key_IsPressed(KeyId key)
{
    if ((uint8_t)key >= (uint8_t)KEY_ID_COUNT) {
        return 0U;
    }
#if !CAR_LIBRARY_BOARD_IS_TIANMENG
    if ((key == KEY_ID_3) || (key == KEY_ID_4) || (key == KEY_ID_5)) {
        return 0U;
    }
#endif
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
    if ((mask & KEY_EVENT_MASK_1) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_1));
        event = KEY_EVENT_1;
    } else if ((mask & KEY_EVENT_MASK_2) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_2));
        event = KEY_EVENT_2;
    } else if ((mask & KEY_EVENT_MASK_3) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_3));
        event = KEY_EVENT_3;
    } else if ((mask & KEY_EVENT_MASK_4) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_4));
        event = KEY_EVENT_4;
    } else if ((mask & KEY_EVENT_MASK_5) != 0U) {
        g_keyEventMask = (uint8_t)(mask & (uint8_t)(~KEY_EVENT_MASK_5));
        event = KEY_EVENT_5;
    }

    Key_ExitCritical(primask);
    return event;
}

uint8_t Key_HandleGPIOInterrupt(void)
{
    uint8_t i;
    uint32_t keyPins = PIN_KEY_1 | PIN_KEY_2;
    uint32_t pending;

#if CAR_LIBRARY_BOARD_IS_TIANMENG
    keyPins |= PIN_KEY_3 | PIN_KEY_4 | PIN_KEY_5;
#endif
    pending = DL_GPIO_getEnabledInterruptStatus(PIN_KEY_PORT, keyPins);
    for (i = 0U; i < (uint8_t)KEY_ID_COUNT; ++i) {
        uint32_t pin = Key_PinFromId((KeyId)i);

        if (((pending & pin) != 0U) &&
            (Key_IsPressed((KeyId)i) == 0U)) {
            Key_HandleReleaseEdge((KeyId)i);
        }
    }
    if (pending != 0U) {
        DL_GPIO_clearInterruptStatus(PIN_KEY_PORT, pending);
    }
    return (pending != 0U) ? 1U : 0U;
}
