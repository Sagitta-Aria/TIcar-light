/*
 * External M0 attitude protocol parser. It validates the 16-byte frame and
 * CRC16, and converts the continuous integral angle and yaw rate to fixed
 * point. UART ownership is kept in the sibling m0_attitude_uart module so
 * the protocol remains independent from pin assignments.
 */
#include "m0_attitude_link.h"

#include "ti_msp_dl_config.h"

#define M0_ATTITUDE_LINK_FRAME_SIZE                 (16U)
#define M0_ATTITUDE_LINK_HEAD_FIRST                 (0xAAU)
#define M0_ATTITUDE_LINK_HEAD_SECOND                (0x55U)
#define M0_ATTITUDE_LINK_CRC_DATA_OFFSET             (2U)
#define M0_ATTITUDE_LINK_CRC_DATA_SIZE               (10U)
#define M0_ATTITUDE_LINK_CRC_OFFSET                  (12U)
#define M0_ATTITUDE_LINK_TAIL_FIRST                 (0x55U)
#define M0_ATTITUDE_LINK_TAIL_SECOND                (0xAAU)
#define M0_ATTITUDE_LINK_MAX_ANGLE_X100             (INT32_MAX)
#define M0_ATTITUDE_LINK_MAX_RATE_X100_PER_SEC      (400000L)
#define M0_ATTITUDE_LINK_MAX_FORWARD_SEQUENCE_DELTA (32767U)
#define M0_ATTITUDE_LINK_COMMAND_HEAD_FIRST          (0xA5U)
#define M0_ATTITUDE_LINK_COMMAND_HEAD_SECOND         (0x5AU)
#define M0_ATTITUDE_LINK_COMMAND_SET_REPORT_RATE     (0x03U)
#define M0_ATTITUDE_LINK_REPORT_RATE_100_HZ          (0x04U)
#define M0_ATTITUDE_LINK_COMMAND_TAIL                (0x5AU)

typedef struct {
    volatile int32_t yawRawX100;
    volatile int32_t yawUnwrappedX100;
    volatile int32_t yawRateX100PerSec;
    volatile uint16_t sequence;
    volatile uint32_t frameCount;
    volatile uint32_t badFrameCount;
    volatile uint32_t droppedFrameCount;
    volatile uint32_t discardedByteCount;
    uint8_t frame[M0_ATTITUDE_LINK_FRAME_SIZE];
    uint8_t frameIndex;
    uint8_t hasYaw;
} M0AttitudeLinkState;

static M0AttitudeLinkState g_m0AttitudeLink;

static uint32_t M0AttitudeLink_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void M0AttitudeLink_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void M0AttitudeLink_AddSaturated(volatile uint32_t *value,
    uint32_t increment)
{
    if ((UINT32_MAX - *value) < increment) {
        *value = UINT32_MAX;
    } else {
        *value += increment;
    }
}

static uint16_t M0AttitudeLink_ReadUint16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

/* Decode a signed IEEE754 float to X100 without Cortex-M0+ FP helpers. */
static uint8_t M0AttitudeLink_DecodeFloatX100(const uint8_t *data,
    uint32_t maximumMagnitude, int32_t *valueX100)
{
    uint32_t bits;
    uint32_t exponentRaw;
    uint32_t mantissa;
    uint64_t scaled;
    uint32_t magnitude;
    int32_t exponent;
    uint32_t shift;

    bits = (uint32_t)data[0] |
        ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) |
        ((uint32_t)data[3] << 24U);
    exponentRaw = (bits >> 23U) & 0xFFU;
    if (exponentRaw == 0xFFU) {
        return 0U;
    }

    if (exponentRaw == 0U) {
        magnitude = 0U;
    } else {
        exponent = (int32_t)exponentRaw - 127;
        mantissa = (bits & 0x007FFFFFUL) | 0x00800000UL;
        scaled = (uint64_t)mantissa * 100U;
        if (exponent >= 23) {
            shift = (uint32_t)(exponent - 23);
            if ((shift >= 32U) ||
                (scaled > ((uint64_t)maximumMagnitude >> shift))) {
                return 0U;
            }
            magnitude = (uint32_t)(scaled << shift);
        } else {
            shift = (uint32_t)(23 - exponent);
            if (shift >= 64U) {
                magnitude = 0U;
            } else {
                if (shift > 0U) {
                    scaled += ((uint64_t)1U << (shift - 1U));
                }
                magnitude = (uint32_t)(scaled >> shift);
            }
        }
    }

    if (magnitude > maximumMagnitude) {
        return 0U;
    }
    *valueX100 = ((bits & 0x80000000UL) != 0U) ?
        -(int32_t)magnitude : (int32_t)magnitude;
    return 1U;
}

static uint16_t M0AttitudeLink_CalculateCrc16(const uint8_t *data,
    uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t index;
    uint8_t bit;

    for (index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 1U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static uint8_t M0AttitudeLink_ApplyFrame(
    const uint8_t frame[M0_ATTITUDE_LINK_FRAME_SIZE])
{
    uint16_t sequence;
    uint16_t sequenceDelta;
    uint16_t receivedCrc;
    uint16_t calculatedCrc;
    int32_t yawX100;
    int32_t yawRateX100PerSec;

    if ((frame[0] != M0_ATTITUDE_LINK_HEAD_FIRST) ||
        (frame[1] != M0_ATTITUDE_LINK_HEAD_SECOND) ||
        (frame[14] != M0_ATTITUDE_LINK_TAIL_FIRST) ||
        (frame[15] != M0_ATTITUDE_LINK_TAIL_SECOND)) {
        return 0U;
    }

    receivedCrc = M0AttitudeLink_ReadUint16(
        &frame[M0_ATTITUDE_LINK_CRC_OFFSET]);
    calculatedCrc = M0AttitudeLink_CalculateCrc16(
        &frame[M0_ATTITUDE_LINK_CRC_DATA_OFFSET],
        M0_ATTITUDE_LINK_CRC_DATA_SIZE);
    if (receivedCrc != calculatedCrc) {
        return 0U;
    }

    if (M0AttitudeLink_DecodeFloatX100(&frame[4],
        (uint32_t)M0_ATTITUDE_LINK_MAX_ANGLE_X100, &yawX100) == 0U) {
        return 0U;
    }
    if (M0AttitudeLink_DecodeFloatX100(&frame[8],
        (uint32_t)M0_ATTITUDE_LINK_MAX_RATE_X100_PER_SEC,
        &yawRateX100PerSec) == 0U) {
        return 0U;
    }

    sequence = M0AttitudeLink_ReadUint16(&frame[2]);
    if (g_m0AttitudeLink.hasYaw != 0U) {
        sequenceDelta = (uint16_t)(sequence - g_m0AttitudeLink.sequence);
        if ((sequenceDelta > 1U) &&
            (sequenceDelta <= M0_ATTITUDE_LINK_MAX_FORWARD_SEQUENCE_DELTA)) {
            M0AttitudeLink_AddSaturated(
                &g_m0AttitudeLink.droppedFrameCount,
                (uint32_t)sequenceDelta - 1U);
        }
    } else {
        g_m0AttitudeLink.hasYaw = 1U;
    }

    g_m0AttitudeLink.yawRawX100 = yawX100;
    g_m0AttitudeLink.yawUnwrappedX100 = yawX100;
    g_m0AttitudeLink.yawRateX100PerSec = yawRateX100PerSec;
    g_m0AttitudeLink.sequence = sequence;
    M0AttitudeLink_AddSaturated(&g_m0AttitudeLink.frameCount, 1U);
    return 1U;
}

static void M0AttitudeLink_ResyncAfterBadFrame(void)
{
    uint8_t start;
    uint8_t count;
    uint8_t index;

    if ((g_m0AttitudeLink.frame[14] == M0_ATTITUDE_LINK_TAIL_FIRST) &&
        (g_m0AttitudeLink.frame[15] == M0_ATTITUDE_LINK_TAIL_SECOND)) {
        g_m0AttitudeLink.frameIndex = 0U;
        return;
    }

    for (start = M0_ATTITUDE_LINK_FRAME_SIZE - 2U; start > 0U;
        --start) {
        if ((g_m0AttitudeLink.frame[start] ==
                M0_ATTITUDE_LINK_HEAD_FIRST) &&
            (g_m0AttitudeLink.frame[start + 1U] ==
                M0_ATTITUDE_LINK_HEAD_SECOND)) {
            count = M0_ATTITUDE_LINK_FRAME_SIZE - start;
            for (index = 0U; index < count; ++index) {
                g_m0AttitudeLink.frame[index] =
                    g_m0AttitudeLink.frame[start + index];
            }
            g_m0AttitudeLink.frameIndex = count;
            return;
        }
    }

    if (g_m0AttitudeLink.frame[M0_ATTITUDE_LINK_FRAME_SIZE - 1U] ==
        M0_ATTITUDE_LINK_HEAD_FIRST) {
        g_m0AttitudeLink.frame[0] = M0_ATTITUDE_LINK_HEAD_FIRST;
        g_m0AttitudeLink.frameIndex = 1U;
    } else {
        g_m0AttitudeLink.frameIndex = 0U;
    }
}

void M0AttitudeLink_Init(void)
{
    uint8_t index;

    g_m0AttitudeLink.yawRawX100 = 0;
    g_m0AttitudeLink.yawUnwrappedX100 = 0;
    g_m0AttitudeLink.yawRateX100PerSec = 0;
    g_m0AttitudeLink.sequence = 0U;
    g_m0AttitudeLink.frameCount = 0U;
    g_m0AttitudeLink.badFrameCount = 0U;
    g_m0AttitudeLink.droppedFrameCount = 0U;
    g_m0AttitudeLink.discardedByteCount = 0U;
    for (index = 0U; index < M0_ATTITUDE_LINK_FRAME_SIZE; ++index) {
        g_m0AttitudeLink.frame[index] = 0U;
    }
    g_m0AttitudeLink.frameIndex = 0U;
    g_m0AttitudeLink.hasYaw = 0U;
}

uint8_t M0AttitudeLink_ConsumeByte(uint8_t data)
{
    if (g_m0AttitudeLink.frameIndex == 0U) {
        if (data == M0_ATTITUDE_LINK_HEAD_FIRST) {
            g_m0AttitudeLink.frame[0] = data;
            g_m0AttitudeLink.frameIndex = 1U;
        } else {
            M0AttitudeLink_AddSaturated(
                &g_m0AttitudeLink.discardedByteCount, 1U);
        }
        return 0U;
    }

    if (g_m0AttitudeLink.frameIndex == 1U) {
        if (data == M0_ATTITUDE_LINK_HEAD_SECOND) {
            g_m0AttitudeLink.frame[1] = data;
            g_m0AttitudeLink.frameIndex = 2U;
        } else {
            M0AttitudeLink_AddSaturated(
                &g_m0AttitudeLink.discardedByteCount, 1U);
            if (data == M0_ATTITUDE_LINK_HEAD_FIRST) {
                g_m0AttitudeLink.frame[0] = data;
            } else {
                g_m0AttitudeLink.frameIndex = 0U;
                M0AttitudeLink_AddSaturated(
                    &g_m0AttitudeLink.discardedByteCount, 1U);
            }
        }
        return 0U;
    }

    g_m0AttitudeLink.frame[g_m0AttitudeLink.frameIndex] = data;
    ++g_m0AttitudeLink.frameIndex;
    if (g_m0AttitudeLink.frameIndex < M0_ATTITUDE_LINK_FRAME_SIZE) {
        return 0U;
    }

    if (M0AttitudeLink_ApplyFrame(g_m0AttitudeLink.frame) != 0U) {
        g_m0AttitudeLink.frameIndex = 0U;
        return 1U;
    }

    M0AttitudeLink_AddSaturated(&g_m0AttitudeLink.badFrameCount, 1U);
    M0AttitudeLink_ResyncAfterBadFrame();
    return 0U;
}

uint8_t M0AttitudeLink_BuildSetReportRate100Hz(uint8_t sequence,
    uint8_t frame[M0_ATTITUDE_LINK_COMMAND_FRAME_SIZE])
{
    uint16_t crc;

    if (frame == 0) {
        return 0U;
    }
    frame[0] = M0_ATTITUDE_LINK_COMMAND_HEAD_FIRST;
    frame[1] = M0_ATTITUDE_LINK_COMMAND_HEAD_SECOND;
    frame[2] = M0_ATTITUDE_LINK_COMMAND_SET_REPORT_RATE;
    frame[3] = M0_ATTITUDE_LINK_REPORT_RATE_100_HZ;
    frame[4] = sequence;
    crc = M0AttitudeLink_CalculateCrc16(&frame[2], 3U);
    frame[5] = (uint8_t)(crc & 0xFFU);
    frame[6] = (uint8_t)(crc >> 8U);
    frame[7] = M0_ATTITUDE_LINK_COMMAND_TAIL;
    return 1U;
}

uint8_t M0AttitudeLink_GetSnapshot(M0AttitudeLinkSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == 0) {
        return 0U;
    }

    primask = M0AttitudeLink_EnterCritical();
    snapshot->yawRawX100 = g_m0AttitudeLink.yawRawX100;
    snapshot->yawUnwrappedX100 = g_m0AttitudeLink.yawUnwrappedX100;
    snapshot->yawRateX100PerSec = g_m0AttitudeLink.yawRateX100PerSec;
    snapshot->sequence = g_m0AttitudeLink.sequence;
    snapshot->frameCount = g_m0AttitudeLink.frameCount;
    snapshot->badFrameCount = g_m0AttitudeLink.badFrameCount;
    snapshot->droppedFrameCount = g_m0AttitudeLink.droppedFrameCount;
    snapshot->discardedByteCount = g_m0AttitudeLink.discardedByteCount;
    M0AttitudeLink_ExitCritical(primask);

    return (snapshot->frameCount != 0U) ? 1U : 0U;
}
