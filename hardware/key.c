#include "key.h"

#include "pin_map.h"

/* 单次 GPIOB 中断最多处理的按键事件数，避免抖动异常时长时间停在 ISR。 */
#define KEY_IRQ_SERVICE_LIMIT    (8U)
/* 按键事件锁定 4 个 App 周期，当前 10ms 一轮，约 40ms 消抖。 */
#define KEY_DEBOUNCE_TICKS       (4U)
#define KEY_EVENT_MASK_1         (0x01U)
#define KEY_EVENT_MASK_2         (0x02U)

static volatile uint8_t g_keyEventMask;
static volatile uint8_t g_keyDebounceTicks[KEY_ID_COUNT];

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

/*
 * 作用：在中断里尝试记录一次有效按键。
 * 使用场景：GPIOB 上升沿触发后调用。
 * 说明：按键保持锁定期间会忽略新的同键中断，避免机械抖动造成多次菜单跳转。
 */
static void Key_TryPushEvent(KeyId key)
{
    if (g_keyDebounceTicks[key] != 0U) {
        return;
    }
    if (Key_IsPressed(key) == 0U) {
        return;
    }

    g_keyEventMask |= Key_EventMaskFromId(key);
    g_keyDebounceTicks[key] = KEY_DEBOUNCE_TICKS;
}

void Key_Init(void)
{
    g_keyEventMask = 0U;
    g_keyDebounceTicks[KEY_ID_1] = 0U;
    g_keyDebounceTicks[KEY_ID_2] = 0U;
    DL_GPIO_clearInterruptStatus(PIN_KEY_PORT, PIN_KEY_1 | PIN_KEY_2);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

void Key_Task(void)
{
    uint8_t i;
    uint32_t primask = Key_EnterCritical();

    for (i = 0U; i < (uint8_t)KEY_ID_COUNT; ++i) {
        if (g_keyDebounceTicks[i] > 0U) {
            --g_keyDebounceTicks[i];
        }
    }
    Key_ExitCritical(primask);
}

uint8_t Key_IsPressed(KeyId key)
{
    uint32_t pin = Key_PinFromId(key);
    return (DL_GPIO_readPins(PIN_KEY_PORT, pin) & pin) ? 1U : 0U;
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
            Key_TryPushEvent(KEY_ID_1);
            break;
        case KEY_2_IIDX:
            Key_TryPushEvent(KEY_ID_2);
            break;
        default:
            break;
        }
        ++serviceCount;
    } while ((pending != DL_GPIO_IIDX_NO_INTR) &&
        (serviceCount < KEY_IRQ_SERVICE_LIMIT));
}
