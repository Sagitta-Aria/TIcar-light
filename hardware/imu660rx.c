/*
 * SeekFree IMU660RA/RB/RC polling driver for the Tianmeng MSPM0G3507 board.
 * SPI1 is shared with the on-board flash, so PB6 flash CS is forced high before
 * every IMU transaction. PB14 is the external IMU CS and conflicts with H8 LCD CS.
 */
#include "imu660rx.h"

#include <stddef.h>

#include "library_config.h"

#if CAR_LIBRARY_IMU660RX_ENABLED

#include "delay.h"
#include "pin_map.h"
#include "ti_msp_dl_config.h"

#define IMU660RX_SPI_INST                    (SPI1)
#define IMU660RX_SPI_TIMEOUT_COUNT           (100000UL)
#define IMU660RX_SPI_DUMMY_BYTE              (0xFFU)
#define IMU660RX_SPI_READ_BIT                (0x80U)
#define IMU660RX_PROBE_RETRIES               (5U)
#define IMU660RX_ACCEL_RANGE_MG              (8000L)
#define IMU660RX_GYRO_RANGE_MDPS             (2000000L)

#define IMU660RA_CHIP_ID_REG                 (0x00U)
#define IMU660RA_CHIP_ID_VALUE               (0x24U)
#define IMU660RA_ACC_DATA_REG                (0x0CU)
#define IMU660RA_INT_STATUS_REG              (0x21U)
#define IMU660RA_ACC_CONF_REG                (0x40U)
#define IMU660RA_ACC_RANGE_REG               (0x41U)
#define IMU660RA_GYRO_CONF_REG               (0x42U)
#define IMU660RA_GYRO_RANGE_REG              (0x43U)
#define IMU660RA_INIT_CTRL_REG               (0x59U)
#define IMU660RA_INIT_DATA_REG               (0x5EU)
#define IMU660RA_PWR_CONF_REG                (0x7CU)
#define IMU660RA_PWR_CTRL_REG                (0x7DU)

#define IMU660RB_FUNC_CFG_ACCESS_REG         (0x01U)
#define IMU660RB_INT1_CTRL_REG               (0x0DU)
#define IMU660RB_WHO_AM_I_REG                (0x0FU)
#define IMU660RB_WHO_AM_I_VALUE              (0x6BU)
#define IMU660RB_CTRL1_XL_REG                (0x10U)
#define IMU660RB_CTRL2_G_REG                 (0x11U)
#define IMU660RB_CTRL3_C_REG                 (0x12U)
#define IMU660RB_CTRL4_C_REG                 (0x13U)
#define IMU660RB_CTRL5_C_REG                 (0x14U)
#define IMU660RB_CTRL6_C_REG                 (0x15U)
#define IMU660RB_CTRL7_G_REG                 (0x16U)
#define IMU660RB_CTRL9_XL_REG                (0x18U)
#define IMU660RB_GYRO_DATA_REG               (0x22U)

#define IMU660RC_FUNC_CFG_ACCESS_REG         (0x01U)
#define IMU660RC_WHO_AM_I_REG                (0x0FU)
#define IMU660RC_WHO_AM_I_VALUE              (0x70U)
#define IMU660RC_CTRL1_REG                   (0x10U)
#define IMU660RC_CTRL2_REG                   (0x11U)
#define IMU660RC_CTRL3_REG                   (0x12U)
#define IMU660RC_CTRL6_REG                   (0x15U)
#define IMU660RC_CTRL7_REG                   (0x16U)
#define IMU660RC_CTRL8_REG                   (0x17U)
#define IMU660RC_CTRL9_REG                   (0x18U)
#define IMU660RC_GYRO_DATA_REG               (0x22U)

#if CAR_LIBRARY_IMU660RX_USES_RA
/*
 * BMI270 requires this vendor configuration stream during initialization.
 * Source: seekfree/IMU660RX_Product commit d73c66c9748db4daea36c2234f9b8b792088c58c.
 */
#include "imu660ra_config_file.inc"
#endif

typedef struct {
    volatile uint8_t busy;
    uint8_t ready;
    IMU660RXModel model;
    IMU660RXStatus lastStatus;
    uint32_t sequence;
} IMU660RXState;

static IMU660RXState g_imu660rx = {
    .lastStatus = IMU660RX_STATUS_NOT_READY
};

static uint32_t IMU660RX_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void IMU660RX_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint8_t IMU660RX_TryLock(void)
{
    uint32_t primask = IMU660RX_EnterCritical();
    uint8_t locked = 0U;

    if (g_imu660rx.busy == 0U) {
        g_imu660rx.busy = 1U;
        locked = 1U;
    }
    IMU660RX_ExitCritical(primask);
    return locked;
}

static void IMU660RX_Unlock(void)
{
    uint32_t primask = IMU660RX_EnterCritical();

    g_imu660rx.busy = 0U;
    IMU660RX_ExitCritical(primask);
}

static void IMU660RX_SetChipSelect(uint8_t active)
{
    if (active != 0U) {
        DL_GPIO_clearPins(PIN_IMU_CS_PORT, PIN_IMU_CS);
    } else {
        DL_GPIO_setPins(PIN_IMU_CS_PORT, PIN_IMU_CS);
    }
}

static IMU660RXStatus IMU660RX_TransferByte(uint8_t tx, uint8_t *rx)
{
    uint32_t timeout = IMU660RX_SPI_TIMEOUT_COUNT;

    while (DL_SPI_isTXFIFOFull(IMU660RX_SPI_INST)) {
        if (--timeout == 0U) {
            return IMU660RX_STATUS_SPI_TIMEOUT;
        }
    }
    DL_SPI_transmitData8(IMU660RX_SPI_INST, tx);

    timeout = IMU660RX_SPI_TIMEOUT_COUNT;
    while (DL_SPI_isRXFIFOEmpty(IMU660RX_SPI_INST)) {
        if (--timeout == 0U) {
            return IMU660RX_STATUS_SPI_TIMEOUT;
        }
    }
    *rx = DL_SPI_receiveData8(IMU660RX_SPI_INST);
    return IMU660RX_STATUS_OK;
}

static void IMU660RX_DrainReceiveFifo(void)
{
    uint8_t discarded[4];

    while (DL_SPI_drainRXFIFO8(IMU660RX_SPI_INST, discarded,
        (uint32_t)sizeof(discarded)) != 0U) {
    }
}

static void IMU660RX_InitSpi(void)
{
    static const DL_SPI_ClockConfig clockConfig = {
        .clockSel = DL_SPI_CLOCK_BUSCLK,
        .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
    };
    static const DL_SPI_Config spiConfig = {
        .mode = DL_SPI_MODE_CONTROLLER,
        .frameFormat = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
        .parity = DL_SPI_PARITY_NONE,
        .dataSize = DL_SPI_DATA_SIZE_8,
        .bitOrder = DL_SPI_BIT_ORDER_MSB_FIRST,
        .chipSelectPin = DL_SPI_CHIP_SELECT_NONE
    };

    DL_SPI_reset(IMU660RX_SPI_INST);
    DL_SPI_enablePower(IMU660RX_SPI_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralInputFunction(
        PIN_IMU_SPI_POCI_IOMUX, PIN_IMU_SPI_POCI_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        PIN_IMU_SPI_PICO_IOMUX, PIN_IMU_SPI_PICO_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        PIN_IMU_SPI_SCK_IOMUX, PIN_IMU_SPI_SCK_FUNC);

    /* Never select the board flash while the shared clock/data pins are active. */
    DL_GPIO_initDigitalOutput(PIN_FLASH_CS_IOMUX);
    DL_GPIO_setPins(PIN_FLASH_CS_PORT, PIN_FLASH_CS);
    DL_GPIO_enableOutput(PIN_FLASH_CS_PORT, PIN_FLASH_CS);

    DL_GPIO_initDigitalOutput(PIN_IMU_CS_IOMUX);
    DL_GPIO_setPins(PIN_IMU_CS_PORT, PIN_IMU_CS);
    DL_GPIO_enableOutput(PIN_IMU_CS_PORT, PIN_IMU_CS);

    DL_SPI_setClockConfig(IMU660RX_SPI_INST,
        (DL_SPI_ClockConfig *)&clockConfig);
    DL_SPI_init(IMU660RX_SPI_INST, (DL_SPI_Config *)&spiConfig);
    /* 32 MHz BUSCLK / ((1 + 1) * 2) = 8 MHz, below the 10 MHz module limit. */
    DL_SPI_setBitRateSerialClockDivider(IMU660RX_SPI_INST, 1U);
    DL_SPI_setFIFOThreshold(IMU660RX_SPI_INST,
        DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
    DL_SPI_enable(IMU660RX_SPI_INST);
    IMU660RX_DrainReceiveFifo();
}

static IMU660RXStatus IMU660RX_WriteRegisters(uint8_t reg,
    const uint8_t *data, uint32_t length)
{
    IMU660RXStatus status;
    uint8_t discarded;
    uint32_t index;

    DL_GPIO_setPins(PIN_FLASH_CS_PORT, PIN_FLASH_CS);
    IMU660RX_DrainReceiveFifo();
    IMU660RX_SetChipSelect(1U);
    status = IMU660RX_TransferByte((uint8_t)(reg & ~IMU660RX_SPI_READ_BIT),
        &discarded);
    for (index = 0U; (index < length) &&
        (status == IMU660RX_STATUS_OK); ++index) {
        status = IMU660RX_TransferByte(data[index], &discarded);
    }
    IMU660RX_SetChipSelect(0U);
    return status;
}

static IMU660RXStatus IMU660RX_WriteRegister(uint8_t reg, uint8_t data)
{
    return IMU660RX_WriteRegisters(reg, &data, 1U);
}

static IMU660RXStatus IMU660RX_ReadRegisters(IMU660RXModel model,
    uint8_t reg, uint8_t *data, uint32_t length)
{
    IMU660RXStatus status;
    uint8_t discarded;
    uint32_t index;

    DL_GPIO_setPins(PIN_FLASH_CS_PORT, PIN_FLASH_CS);
    IMU660RX_DrainReceiveFifo();
    IMU660RX_SetChipSelect(1U);
    status = IMU660RX_TransferByte((uint8_t)(reg | IMU660RX_SPI_READ_BIT),
        &discarded);
    if ((status == IMU660RX_STATUS_OK) &&
        (model == IMU660RX_MODEL_RA)) {
        status = IMU660RX_TransferByte(IMU660RX_SPI_DUMMY_BYTE, &discarded);
    }
    for (index = 0U; (index < length) &&
        (status == IMU660RX_STATUS_OK); ++index) {
        status = IMU660RX_TransferByte(IMU660RX_SPI_DUMMY_BYTE, &data[index]);
    }
    IMU660RX_SetChipSelect(0U);
    return status;
}

static IMU660RXStatus IMU660RX_ReadRegister(IMU660RXModel model,
    uint8_t reg, uint8_t *data)
{
    return IMU660RX_ReadRegisters(model, reg, data, 1U);
}

static IMU660RXStatus IMU660RX_ProbeModel(IMU660RXModel model)
{
    uint8_t expected;
    uint8_t reg;
    uint8_t value = 0U;
    uint8_t attempt;
    IMU660RXStatus status = IMU660RX_STATUS_NOT_FOUND;

    if (model == IMU660RX_MODEL_RA) {
        reg = IMU660RA_CHIP_ID_REG;
        expected = IMU660RA_CHIP_ID_VALUE;
    } else if (model == IMU660RX_MODEL_RB) {
        reg = IMU660RB_WHO_AM_I_REG;
        expected = IMU660RB_WHO_AM_I_VALUE;
    } else if (model == IMU660RX_MODEL_RC) {
        reg = IMU660RC_WHO_AM_I_REG;
        expected = IMU660RC_WHO_AM_I_VALUE;
    } else {
        return IMU660RX_STATUS_INVALID_ARGUMENT;
    }

    for (attempt = 0U; attempt < IMU660RX_PROBE_RETRIES; ++attempt) {
        status = IMU660RX_ReadRegister(model, reg, &value);
        if (status != IMU660RX_STATUS_OK) {
            return status;
        }
        if (value == expected) {
            return IMU660RX_STATUS_OK;
        }
        delay_ms(1U);
    }
    return IMU660RX_STATUS_NOT_FOUND;
}

static IMU660RXStatus IMU660RX_DetectModel(IMU660RXModel *model)
{
    IMU660RXStatus status;

    status = IMU660RX_ProbeModel(IMU660RX_MODEL_RB);
    if (status == IMU660RX_STATUS_OK) {
        *model = IMU660RX_MODEL_RB;
        return status;
    }
    if (status == IMU660RX_STATUS_SPI_TIMEOUT) {
        return status;
    }

    status = IMU660RX_ProbeModel(IMU660RX_MODEL_RC);
    if (status == IMU660RX_STATUS_OK) {
        *model = IMU660RX_MODEL_RC;
        return status;
    }
    if (status == IMU660RX_STATUS_SPI_TIMEOUT) {
        return status;
    }

    status = IMU660RX_ProbeModel(IMU660RX_MODEL_RA);
    if (status == IMU660RX_STATUS_OK) {
        *model = IMU660RX_MODEL_RA;
        return status;
    }
    return status;
}

static IMU660RXStatus IMU660RX_InitRA(void)
{
#if CAR_LIBRARY_IMU660RX_USES_RA
    IMU660RXStatus status;
    uint8_t initStatus = 0U;

    status = IMU660RX_ProbeModel(IMU660RX_MODEL_RA);
    if (status != IMU660RX_STATUS_OK) {
        return status;
    }
    status = IMU660RX_WriteRegister(IMU660RA_PWR_CONF_REG, 0x00U);
    delay_ms(1U);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RA_INIT_CTRL_REG, 0x00U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegisters(IMU660RA_INIT_DATA_REG,
            imu660ra_config_file, (uint32_t)sizeof(imu660ra_config_file));
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RA_INIT_CTRL_REG, 0x01U);
    }
    delay_ms(20U);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_ReadRegister(IMU660RX_MODEL_RA,
            IMU660RA_INT_STATUS_REG, &initStatus);
    }
    if ((status != IMU660RX_STATUS_OK) || (initStatus != 0x01U)) {
        return (status == IMU660RX_STATUS_OK) ?
            IMU660RX_STATUS_CONFIG_ERROR : status;
    }
    status = IMU660RX_WriteRegister(IMU660RA_PWR_CTRL_REG, 0x0EU);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RA_ACC_CONF_REG, 0xA7U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RA_GYRO_CONF_REG, 0xA9U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RA_ACC_RANGE_REG, 0x02U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RA_GYRO_RANGE_REG, 0x00U);
    }
    return status;
#else
    return IMU660RX_STATUS_CONFIG_ERROR;
#endif
}

static IMU660RXStatus IMU660RX_InitRB(void)
{
    IMU660RXStatus status;

    status = IMU660RX_WriteRegister(IMU660RB_FUNC_CFG_ACCESS_REG, 0x00U);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL3_C_REG, 0x01U);
    }
    delay_ms(2U);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_FUNC_CFG_ACCESS_REG, 0x00U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_ProbeModel(IMU660RX_MODEL_RB);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_INT1_CTRL_REG, 0x00U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL1_XL_REG, 0x3CU);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL2_G_REG, 0x5CU);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL3_C_REG, 0x44U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL4_C_REG, 0x02U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL5_C_REG, 0x00U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL6_C_REG, 0x00U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL7_G_REG, 0x00U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RB_CTRL9_XL_REG, 0x01U);
    }
    return status;
}

static IMU660RXStatus IMU660RX_InitRC(void)
{
    IMU660RXStatus status;

    status = IMU660RX_ProbeModel(IMU660RX_MODEL_RC);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_FUNC_CFG_ACCESS_REG, 0x04U);
    }
    delay_ms(30U);
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL3_REG, 0x44U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL8_REG, 0x02U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL6_REG, 0x04U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL1_REG, 0x15U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL2_REG, 0x18U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL7_REG, 0x01U);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_WriteRegister(IMU660RC_CTRL9_REG, 0x08U);
    }
    return status;
}

static IMU660RXStatus IMU660RX_InitModel(IMU660RXModel model)
{
    switch (model) {
    case IMU660RX_MODEL_RA:
        return IMU660RX_InitRA();
    case IMU660RX_MODEL_RB:
        return IMU660RX_InitRB();
    case IMU660RX_MODEL_RC:
        return IMU660RX_InitRC();
    default:
        return IMU660RX_STATUS_INVALID_ARGUMENT;
    }
}

static int16_t IMU660RX_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int32_t IMU660RX_ScaleSigned(int16_t raw, int32_t fullScale)
{
    int64_t magnitude = (int64_t)raw * (int64_t)fullScale;

    if (magnitude >= 0) {
        return (int32_t)((magnitude + 16384LL) / 32768LL);
    }
    return (int32_t)(-(((-magnitude) + 16384LL) / 32768LL));
}

IMU660RXStatus IMU660RX_Init(void)
{
    IMU660RXModel requested = (IMU660RXModel)CAR_LIBRARY_IMU660RX_METHOD;
    IMU660RXStatus status;

    if (IMU660RX_TryLock() == 0U) {
        return IMU660RX_STATUS_BUSY;
    }

    g_imu660rx.ready = 0U;
    g_imu660rx.model = IMU660RX_MODEL_NONE;
    g_imu660rx.sequence = 0U;
    IMU660RX_InitSpi();
    delay_ms(20U);

    if (requested == IMU660RX_MODEL_AUTO) {
        status = IMU660RX_DetectModel(&requested);
    } else {
        status = IMU660RX_ProbeModel(requested);
    }
    if (status == IMU660RX_STATUS_OK) {
        status = IMU660RX_InitModel(requested);
    }
    if (status == IMU660RX_STATUS_OK) {
        g_imu660rx.model = requested;
        g_imu660rx.ready = 1U;
    }
    g_imu660rx.lastStatus = status;
    IMU660RX_Unlock();
    return status;
}

IMU660RXStatus IMU660RX_Read(IMU660RXSample *sample)
{
    uint8_t data[12];
    const uint8_t *accel;
    const uint8_t *gyro;
    uint8_t startRegister;
    IMU660RXStatus status;

    if (sample == NULL) {
        return IMU660RX_STATUS_INVALID_ARGUMENT;
    }
    if (g_imu660rx.ready == 0U) {
        return IMU660RX_STATUS_NOT_READY;
    }
    if (IMU660RX_TryLock() == 0U) {
        return IMU660RX_STATUS_BUSY;
    }
    if (g_imu660rx.ready == 0U) {
        IMU660RX_Unlock();
        return IMU660RX_STATUS_NOT_READY;
    }

    if (g_imu660rx.model == IMU660RX_MODEL_RA) {
        startRegister = IMU660RA_ACC_DATA_REG;
    } else if (g_imu660rx.model == IMU660RX_MODEL_RB) {
        startRegister = IMU660RB_GYRO_DATA_REG;
    } else {
        startRegister = IMU660RC_GYRO_DATA_REG;
    }
    status = IMU660RX_ReadRegisters(g_imu660rx.model, startRegister,
        data, (uint32_t)sizeof(data));
    if (status != IMU660RX_STATUS_OK) {
        g_imu660rx.lastStatus = status;
        IMU660RX_Unlock();
        return status;
    }

    if (g_imu660rx.model == IMU660RX_MODEL_RA) {
        accel = &data[0];
        gyro = &data[6];
    } else {
        gyro = &data[0];
        accel = &data[6];
    }
    sample->accelRawX = IMU660RX_ReadInt16(&accel[0]);
    sample->accelRawY = IMU660RX_ReadInt16(&accel[2]);
    sample->accelRawZ = IMU660RX_ReadInt16(&accel[4]);
    sample->gyroRawX = IMU660RX_ReadInt16(&gyro[0]);
    sample->gyroRawY = IMU660RX_ReadInt16(&gyro[2]);
    sample->gyroRawZ = IMU660RX_ReadInt16(&gyro[4]);
    sample->accelMgX = IMU660RX_ScaleSigned(sample->accelRawX,
        IMU660RX_ACCEL_RANGE_MG);
    sample->accelMgY = IMU660RX_ScaleSigned(sample->accelRawY,
        IMU660RX_ACCEL_RANGE_MG);
    sample->accelMgZ = IMU660RX_ScaleSigned(sample->accelRawZ,
        IMU660RX_ACCEL_RANGE_MG);
    sample->gyroMdpsX = IMU660RX_ScaleSigned(sample->gyroRawX,
        IMU660RX_GYRO_RANGE_MDPS);
    sample->gyroMdpsY = IMU660RX_ScaleSigned(sample->gyroRawY,
        IMU660RX_GYRO_RANGE_MDPS);
    sample->gyroMdpsZ = IMU660RX_ScaleSigned(sample->gyroRawZ,
        IMU660RX_GYRO_RANGE_MDPS);
    sample->sequence = ++g_imu660rx.sequence;
    sample->model = g_imu660rx.model;
    g_imu660rx.lastStatus = IMU660RX_STATUS_OK;
    IMU660RX_Unlock();
    return IMU660RX_STATUS_OK;
}

uint8_t IMU660RX_IsReady(void)
{
    return g_imu660rx.ready;
}

IMU660RXModel IMU660RX_GetModel(void)
{
    return g_imu660rx.model;
}

IMU660RXStatus IMU660RX_GetLastStatus(void)
{
    return g_imu660rx.lastStatus;
}

#else

IMU660RXStatus IMU660RX_Init(void)
{
    return IMU660RX_STATUS_DISABLED;
}

IMU660RXStatus IMU660RX_Read(IMU660RXSample *sample)
{
    return (sample == NULL) ? IMU660RX_STATUS_INVALID_ARGUMENT :
        IMU660RX_STATUS_DISABLED;
}

uint8_t IMU660RX_IsReady(void)
{
    return 0U;
}

IMU660RXModel IMU660RX_GetModel(void)
{
    return IMU660RX_MODEL_NONE;
}

IMU660RXStatus IMU660RX_GetLastStatus(void)
{
    return IMU660RX_STATUS_DISABLED;
}

#endif

const char *IMU660RX_ModelName(IMU660RXModel model)
{
    switch (model) {
    case IMU660RX_MODEL_RA:
        return "IMU660RA";
    case IMU660RX_MODEL_RB:
        return "IMU660RB";
    case IMU660RX_MODEL_RC:
        return "IMU660RC";
    case IMU660RX_MODEL_AUTO:
        return "AUTO";
    default:
        return "NONE";
    }
}
