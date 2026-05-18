#include "key.h"

#include "pin_map.h"

static volatile KeyEvent g_keyEvent = KEY_EVENT_NONE;

static uint32_t Key_PinFromId(KeyId key)
{
    return (key == KEY_ID_1) ? PIN_KEY_1 : PIN_KEY_2;
}

void Key_Init(void)
{
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
}

uint8_t Key_IsPressed(KeyId key)
{
    uint32_t pin = Key_PinFromId(key);
    return (DL_GPIO_readPins(PIN_KEY_PORT, pin) & pin) ? 1U : 0U;
}

KeyEvent Key_PopEvent(void)
{
    KeyEvent event = g_keyEvent;
    g_keyEvent = KEY_EVENT_NONE;
    return event;
}

void Key_HandleGPIOInterrupt(void)
{
    DL_GPIO_IIDX pending;

    do {
        pending = DL_GPIO_getPendingInterrupt(PIN_KEY_PORT);
        switch (pending) {
        case KEY_1_IIDX:
            g_keyEvent = KEY_EVENT_1;
            break;
        case KEY_2_IIDX:
            g_keyEvent = KEY_EVENT_2;
            break;
        default:
            break;
        }
    } while (pending != DL_GPIO_IIDX_NO_INTR);
}
