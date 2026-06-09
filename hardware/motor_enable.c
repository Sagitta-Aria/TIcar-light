#include "motor_enable.h"

#include "board_config.h"
#include "pin_map.h"

typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
    uint8_t enabled;
} MotorEnablePin;

typedef enum {
    MOTOR_ENABLE_CHASSIS_LEFT = 0,
    MOTOR_ENABLE_CHASSIS_RIGHT,
    MOTOR_ENABLE_GIMBAL_1,
    MOTOR_ENABLE_GIMBAL_2,
    MOTOR_ENABLE_COUNT
} MotorEnableIndex;

static MotorEnablePin g_motorEnablePins[MOTOR_ENABLE_COUNT] = {
    {
        PIN_STEPPER_CHASSIS_LEFT_EN_PORT,
        PIN_STEPPER_CHASSIS_LEFT_EN,
        0U
    },
    {
        PIN_STEPPER_CHASSIS_RIGHT_EN_PORT,
        PIN_STEPPER_CHASSIS_RIGHT_EN,
        0U
    },
    {
        PIN_STEPPER_GIMBAL_1_EN_PORT,
        PIN_STEPPER_GIMBAL_1_EN,
        0U
    },
    {
        PIN_STEPPER_GIMBAL_2_EN_PORT,
        PIN_STEPPER_GIMBAL_2_EN,
        0U
    }
};

static void MotorEnable_Write(MotorEnablePin *pin, uint8_t enabled)
{
#if (CAR_STEPPER_ENABLE_ACTIVE_LOW != 0U)
    if (enabled != 0U) {
        DL_GPIO_clearPins(pin->port, pin->pin);
    } else {
        DL_GPIO_setPins(pin->port, pin->pin);
    }
#else
    if (enabled != 0U) {
        DL_GPIO_setPins(pin->port, pin->pin);
    } else {
        DL_GPIO_clearPins(pin->port, pin->pin);
    }
#endif
    pin->enabled = (enabled != 0U) ? 1U : 0U;
}

void MotorEnable_Init(void)
{
    MotorEnable_SetAll(CAR_STEPPER_ENABLE_DEFAULT_ON);
}

void MotorEnable_SetAll(uint8_t enabled)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)MOTOR_ENABLE_COUNT; ++i) {
        MotorEnable_Write(&g_motorEnablePins[i], enabled);
    }
}

void MotorEnable_SetGimbal(uint8_t enabled)
{
    MotorEnable_Write(&g_motorEnablePins[MOTOR_ENABLE_GIMBAL_1], enabled);
    MotorEnable_Write(&g_motorEnablePins[MOTOR_ENABLE_GIMBAL_2], enabled);
}

uint8_t MotorEnable_IsAllEnabled(void)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)MOTOR_ENABLE_COUNT; ++i) {
        if (g_motorEnablePins[i].enabled == 0U) {
            return 0U;
        }
    }
    return 1U;
}

const char *MotorEnable_GetActiveLevelName(void)
{
#if (CAR_STEPPER_ENABLE_ACTIVE_LOW != 0U)
    return "EN LOW";
#else
    return "EN HIGH";
#endif
}
