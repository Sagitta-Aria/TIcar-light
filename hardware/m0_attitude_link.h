#ifndef M0_ATTITUDE_LINK_H
#define M0_ATTITUDE_LINK_H

#include <stdint.h>

#define M0_ATTITUDE_LINK_COMMAND_FRAME_SIZE (8U)

typedef struct {
    /* Latest continuous integral angle from CY-Z, in 0.01 degree. */
    int32_t yawRawX100;
    /* Compatibility field; CY-Z already supplies a continuous angle. */
    int32_t yawUnwrappedX100;
    /* Latest yaw rate in 0.01 degree/second. */
    int32_t yawRateX100PerSec;
    /* Sender's little-endian 16-bit frame counter. */
    uint16_t sequence;
    /* Accepted, rejected, missing, and out-of-frame byte diagnostics. */
    uint32_t frameCount;
    uint32_t badFrameCount;
    uint32_t droppedFrameCount;
    uint32_t discardedByteCount;
} M0AttitudeLinkSnapshot;

/* Reset the external M0 attitude frame parser and its statistics. */
void M0AttitudeLink_Init(void);

/*
 * Feed one received byte from an ISR or task. The observed wire format is
 * AA 55 + uint16 sequence + float yaw + float yaw rate + CRC16 + 55 AA.
 * Multi-byte fields are little-endian. CRC16-Modbus covers sequence, yaw,
 * and yaw rate (frame bytes 2 through 11).
 * The captured serial configuration is 115200 baud, 8 data bits, no parity,
 * and 1 stop bit. This parser does not configure or claim a UART instance.
 * Returns 1 only when this byte completes an accepted frame.
 */
uint8_t M0AttitudeLink_ConsumeByte(uint8_t data);

/* Build the documented command that enables CY-Z 100 Hz UART telemetry. */
uint8_t M0AttitudeLink_BuildSetReportRate100Hz(uint8_t sequence,
    uint8_t frame[M0_ATTITUDE_LINK_COMMAND_FRAME_SIZE]);

/* Atomically read the latest processed yaw; returns 0 before the first frame. */
uint8_t M0AttitudeLink_GetSnapshot(M0AttitudeLinkSnapshot *snapshot);

#endif
