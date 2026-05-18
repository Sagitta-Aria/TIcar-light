#include "encoder.h"

#include "pin_map.h"

static volatile int32_t g_leftCount;
static volatile int32_t g_rightCount;

void Encoder_Init(void)
{
    Encoder_Reset();
    NVIC_EnableIRQ(GPIOA_INT_IRQn);
}

int32_t Encoder_GetLeft(void)
{
    return g_leftCount;
}

int32_t Encoder_GetRight(void)
{
    return g_rightCount;
}

void Encoder_Reset(void)
{
    g_leftCount = 0;
    g_rightCount = 0;
}

void Encoder_HandleGPIOInterrupt(void)
{
    DL_GPIO_IIDX pending;
    uint32_t state;

    do {
        pending = DL_GPIO_getPendingInterrupt(PIN_ENCODER_PORT);
        state = DL_GPIO_readPins(PIN_ENCODER_PORT,
            PIN_ENCODER_LEFT_A | PIN_ENCODER_LEFT_B |
            PIN_ENCODER_RIGHT_A | PIN_ENCODER_RIGHT_B);

        switch (pending) {
        case ENCODER_LEFT_A_IIDX:
            g_leftCount += (state & PIN_ENCODER_LEFT_B) ? 1 : -1;
            break;
        case ENCODER_LEFT_B_IIDX:
            g_leftCount += (state & PIN_ENCODER_LEFT_A) ? -1 : 1;
            break;
        case ENCODER_RIGHT_A_IIDX:
            g_rightCount += (state & PIN_ENCODER_RIGHT_B) ? 1 : -1;
            break;
        case ENCODER_RIGHT_B_IIDX:
            g_rightCount += (state & PIN_ENCODER_RIGHT_A) ? -1 : 1;
            break;
        default:
            break;
        }
    } while (pending != DL_GPIO_IIDX_NO_INTR);
}
