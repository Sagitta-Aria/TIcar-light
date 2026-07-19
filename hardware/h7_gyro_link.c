#include "h7_gyro_link.h"

#include "FreeRTOS.h"
#include "task.h"

#include "ti_msp_dl_config.h"

#define H7_GYRO_LINK_FRAME_SIZE          (11U)
#define H7_GYRO_LINK_FRAME_HEAD          (0x55U)
#define H7_GYRO_LINK_FRAME_RATE          (0x52U)
#define H7_GYRO_LINK_FRAME_ANGLE         (0x53U)
#define H7_GYRO_LINK_SERVICE_LIMIT       (16U)
#define H7_GYRO_LINK_RX_DRAIN_LIMIT      (64U)
#define H7_GYRO_LINK_FULL_TURN_X100      (36000L)
#define H7_GYRO_LINK_HALF_TURN_X100      (18000L)

#if (H7GyroLink_BAUD_RATE != LogUart_BAUD_RATE)
#error "H7 feedback and UART0 log TX must use the same baud rate"
#endif

typedef struct {
    volatile int16_t rollX100;
    volatile int16_t pitchX100;
    volatile int16_t yawRawX100;
    volatile int32_t yawUnwrappedX100;
    volatile int32_t rollRateX100PerSec;
    volatile int32_t pitchRateX100PerSec;
    volatile int32_t yawRateX100PerSec;
    volatile uint32_t angleFrameCount;
    volatile uint32_t gyroFrameCount;
    volatile uint32_t angleFrameTick;
    volatile uint32_t gyroFrameTick;
    volatile uint32_t badFrameCount;
    uint8_t frame[H7_GYRO_LINK_FRAME_SIZE];
    uint8_t frameIndex;
    uint8_t hasYaw;
} H7GyroLinkState;

static H7GyroLinkState g_h7GyroLink;

static uint32_t H7GyroLink_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void H7GyroLink_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static int16_t H7GyroLink_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t H7GyroLink_AngleRawToX100(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 18000L) / 32768L);
}

static int32_t H7GyroLink_RateRawToX100PerSec(int16_t raw)
{
    return ((int32_t)raw * 6250L) / 1024L;
}

static uint8_t H7GyroLink_ChecksumOk(
    const uint8_t frame[H7_GYRO_LINK_FRAME_SIZE])
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (uint8_t)(H7_GYRO_LINK_FRAME_SIZE - 1U); ++i) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return (uint8_t)(sum == frame[H7_GYRO_LINK_FRAME_SIZE - 1U]);
}

static void H7GyroLink_UpdateYaw(int16_t yawRawX100)
{
    int32_t delta;

    if (g_h7GyroLink.hasYaw == 0U) {
        g_h7GyroLink.hasYaw = 1U;
        g_h7GyroLink.yawRawX100 = yawRawX100;
        g_h7GyroLink.yawUnwrappedX100 = yawRawX100;
        return;
    }

    delta = (int32_t)yawRawX100 - g_h7GyroLink.yawRawX100;
    if (delta > H7_GYRO_LINK_HALF_TURN_X100) {
        delta -= H7_GYRO_LINK_FULL_TURN_X100;
    } else if (delta < -H7_GYRO_LINK_HALF_TURN_X100) {
        delta += H7_GYRO_LINK_FULL_TURN_X100;
    }
    g_h7GyroLink.yawRawX100 = yawRawX100;
    g_h7GyroLink.yawUnwrappedX100 += delta;
}

static void H7GyroLink_ApplyFrame(
    const uint8_t frame[H7_GYRO_LINK_FRAME_SIZE])
{
    if (H7GyroLink_ChecksumOk(frame) == 0U) {
        ++g_h7GyroLink.badFrameCount;
        return;
    }

    if (frame[1] == H7_GYRO_LINK_FRAME_RATE) {
        g_h7GyroLink.rollRateX100PerSec =
            H7GyroLink_RateRawToX100PerSec(
                H7GyroLink_ReadInt16(&frame[2]));
        g_h7GyroLink.pitchRateX100PerSec =
            H7GyroLink_RateRawToX100PerSec(
                H7GyroLink_ReadInt16(&frame[4]));
        g_h7GyroLink.yawRateX100PerSec =
            H7GyroLink_RateRawToX100PerSec(
                H7GyroLink_ReadInt16(&frame[6]));
        g_h7GyroLink.gyroFrameTick =
            (uint32_t)xTaskGetTickCountFromISR();
        ++g_h7GyroLink.gyroFrameCount;
    } else if (frame[1] == H7_GYRO_LINK_FRAME_ANGLE) {
        g_h7GyroLink.rollX100 = H7GyroLink_AngleRawToX100(
            H7GyroLink_ReadInt16(&frame[2]));
        g_h7GyroLink.pitchX100 = H7GyroLink_AngleRawToX100(
            H7GyroLink_ReadInt16(&frame[4]));
        H7GyroLink_UpdateYaw(H7GyroLink_AngleRawToX100(
            H7GyroLink_ReadInt16(&frame[6])));
        g_h7GyroLink.angleFrameTick =
            (uint32_t)xTaskGetTickCountFromISR();
        ++g_h7GyroLink.angleFrameCount;
    }
}

static void H7GyroLink_ParseByte(uint8_t data)
{
    if (g_h7GyroLink.frameIndex == 0U) {
        if (data == H7_GYRO_LINK_FRAME_HEAD) {
            g_h7GyroLink.frame[0] = data;
            g_h7GyroLink.frameIndex = 1U;
        }
        return;
    }

    if ((g_h7GyroLink.frameIndex == 1U) &&
        (data != H7_GYRO_LINK_FRAME_RATE) &&
        (data != H7_GYRO_LINK_FRAME_ANGLE)) {
        g_h7GyroLink.frameIndex = 0U;
        if (data == H7_GYRO_LINK_FRAME_HEAD) {
            g_h7GyroLink.frame[0] = data;
            g_h7GyroLink.frameIndex = 1U;
        }
        return;
    }

    g_h7GyroLink.frame[g_h7GyroLink.frameIndex] = data;
    ++g_h7GyroLink.frameIndex;
    if (g_h7GyroLink.frameIndex >= H7_GYRO_LINK_FRAME_SIZE) {
        H7GyroLink_ApplyFrame(g_h7GyroLink.frame);
        g_h7GyroLink.frameIndex = 0U;
    }
}

void H7GyroLink_Init(void)
{
    uint8_t i;

    g_h7GyroLink.rollX100 = 0;
    g_h7GyroLink.pitchX100 = 0;
    g_h7GyroLink.yawRawX100 = 0;
    g_h7GyroLink.yawUnwrappedX100 = 0;
    g_h7GyroLink.rollRateX100PerSec = 0;
    g_h7GyroLink.pitchRateX100PerSec = 0;
    g_h7GyroLink.yawRateX100PerSec = 0;
    g_h7GyroLink.angleFrameCount = 0U;
    g_h7GyroLink.gyroFrameCount = 0U;
    g_h7GyroLink.angleFrameTick = 0U;
    g_h7GyroLink.gyroFrameTick = 0U;
    g_h7GyroLink.badFrameCount = 0U;
    g_h7GyroLink.frameIndex = 0U;
    g_h7GyroLink.hasYaw = 0U;
    for (i = 0U; i < H7_GYRO_LINK_FRAME_SIZE; ++i) {
        g_h7GyroLink.frame[i] = 0U;
    }

    DL_UART_Main_setRXFIFOThreshold(H7GyroLink_INST,
        DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(H7GyroLink_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(H7GyroLink_INST_INT_IRQN);
    NVIC_EnableIRQ(H7GyroLink_INST_INT_IRQN);
}

void H7GyroLink_HandleUARTInterrupt(void)
{
    DL_UART_IIDX pending;
    uint8_t data;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;

    do {
        pending = DL_UART_Main_getPendingInterrupt(H7GyroLink_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < H7_GYRO_LINK_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(H7GyroLink_INST, &data)) {
                H7GyroLink_ParseByte(data);
                ++rxCount;
            }
        } else if ((pending == DL_UART_MAIN_IIDX_FRAMING_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_NOISE_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_OVERRUN_ERROR)) {
            ++g_h7GyroLink.badFrameCount;
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < H7_GYRO_LINK_SERVICE_LIMIT));
}

uint8_t H7GyroLink_GetFeedback(H7GyroLinkFeedback *feedback)
{
    uint32_t primask;

    if (feedback == 0) {
        return 0U;
    }

    primask = H7GyroLink_EnterCritical();
    feedback->rollX100 = g_h7GyroLink.rollX100;
    feedback->pitchX100 = g_h7GyroLink.pitchX100;
    feedback->yawRawX100 = g_h7GyroLink.yawRawX100;
    feedback->yawUnwrappedX100 = g_h7GyroLink.yawUnwrappedX100;
    feedback->rollRateX100PerSec = g_h7GyroLink.rollRateX100PerSec;
    feedback->pitchRateX100PerSec = g_h7GyroLink.pitchRateX100PerSec;
    feedback->yawRateX100PerSec = g_h7GyroLink.yawRateX100PerSec;
    feedback->angleFrameCount = g_h7GyroLink.angleFrameCount;
    feedback->gyroFrameCount = g_h7GyroLink.gyroFrameCount;
    feedback->angleFrameTick = g_h7GyroLink.angleFrameTick;
    feedback->gyroFrameTick = g_h7GyroLink.gyroFrameTick;
    feedback->badFrameCount = g_h7GyroLink.badFrameCount;
    H7GyroLink_ExitCritical(primask);

    return (uint8_t)(((feedback->angleFrameCount != 0U) ||
        (feedback->gyroFrameCount != 0U)) ? 1U : 0U);
}
