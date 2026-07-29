#ifndef IMU660RX_H
#define IMU660RX_H

#include <stdint.h>

/* Product variants supported by the unified SPI driver. */
typedef enum {
    IMU660RX_MODEL_NONE = 0U,
    IMU660RX_MODEL_AUTO = 1U,
    IMU660RX_MODEL_RA = 2U,
    IMU660RX_MODEL_RB = 3U,
    IMU660RX_MODEL_RC = 4U
} IMU660RXModel;

/* Public results are explicit so a missing sensor never looks like zero motion. */
typedef enum {
    IMU660RX_STATUS_OK = 0U,
    IMU660RX_STATUS_DISABLED = 1U,
    IMU660RX_STATUS_INVALID_ARGUMENT = 2U,
    IMU660RX_STATUS_BUSY = 3U,
    IMU660RX_STATUS_SPI_TIMEOUT = 4U,
    IMU660RX_STATUS_NOT_FOUND = 5U,
    IMU660RX_STATUS_CONFIG_ERROR = 6U,
    IMU660RX_STATUS_NOT_READY = 7U
} IMU660RXStatus;

/* One coherent six-axis sample. Scaled values use integer engineering units. */
typedef struct {
    int16_t accelRawX;
    int16_t accelRawY;
    int16_t accelRawZ;
    int16_t gyroRawX;
    int16_t gyroRawY;
    int16_t gyroRawZ;
    int32_t accelMgX;
    int32_t accelMgY;
    int32_t accelMgZ;
    int32_t gyroMdpsX;
    int32_t gyroMdpsY;
    int32_t gyroMdpsZ;
    uint32_t sequence;
    IMU660RXModel model;
} IMU660RXSample;

/*
 * Initializes Tianmeng SPI1 and the model selected by CAR_LIBRARY_IMU660RX_METHOD.
 * Call once after the board clock is stable and before tasks start using the bus.
 */
IMU660RXStatus IMU660RX_Init(void);

/*
 * Polls all six axes in one burst. This task-context API is not allowed from an ISR;
 * concurrent callers receive IMU660RX_STATUS_BUSY instead of corrupting the SPI frame.
 */
IMU660RXStatus IMU660RX_Read(IMU660RXSample *sample);

/* Read-only state helpers for startup diagnostics and application feature checks. */
uint8_t IMU660RX_IsReady(void);
IMU660RXModel IMU660RX_GetModel(void);
IMU660RXStatus IMU660RX_GetLastStatus(void);
const char *IMU660RX_ModelName(IMU660RXModel model);

#endif
