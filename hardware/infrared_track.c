/* 八路红外循迹GPIO驱动，只负责引脚初始化和高电平黑线状态采集。 */
#include "library_config.h"

#if CAR_LIBRARY_GRAY_INPUT_IS_INFRARED_8

#include "infrared_track.h"

#include "board_config.h"
#include "pin_map.h"

#if (GRAY_SENSOR_COUNT != INFRARED_TRACK_SENSOR_COUNT)
#error "Eight-channel infrared tracking requires GRAY_SENSOR_COUNT == 8"
#endif

typedef struct {
    uint32_t pin;
    uint32_t iomux;
} InfraredTrackSlot;

static const InfraredTrackSlot g_infraredTrackMap[
    INFRARED_TRACK_SENSOR_COUNT] = {
    {PIN_INFRARED_TRACK_1, PIN_INFRARED_TRACK_1_IOMUX},
    {PIN_INFRARED_TRACK_2, PIN_INFRARED_TRACK_2_IOMUX},
    {PIN_INFRARED_TRACK_3, PIN_INFRARED_TRACK_3_IOMUX},
    {PIN_INFRARED_TRACK_4, PIN_INFRARED_TRACK_4_IOMUX},
    {PIN_INFRARED_TRACK_5, PIN_INFRARED_TRACK_5_IOMUX},
    {PIN_INFRARED_TRACK_6, PIN_INFRARED_TRACK_6_IOMUX},
    {PIN_INFRARED_TRACK_7, PIN_INFRARED_TRACK_7_IOMUX},
    {PIN_INFRARED_TRACK_8, PIN_INFRARED_TRACK_8_IOMUX},
};

static uint8_t InfraredTrack_BitForIndex(uint32_t index)
{
    return (uint8_t)(1U << ((INFRARED_TRACK_SENSOR_COUNT - 1U) - index));
}

void InfraredTrack_Init(void)
{
    uint32_t index;
#if GRAY_DIGITAL_INPUT_PULL_UP
    uint32_t resistor = DL_GPIO_RESISTOR_PULL_UP;
#else
    uint32_t resistor = DL_GPIO_RESISTOR_NONE;
#endif

    for (index = 0U; index < INFRARED_TRACK_SENSOR_COUNT; ++index) {
        DL_GPIO_initDigitalInputFeatures(g_infraredTrackMap[index].iomux,
            DL_GPIO_INVERSION_DISABLE, resistor,
            DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    }
}

uint8_t InfraredTrack_ReadMask(void)
{
    uint8_t mask = 0U;
    uint32_t index;
    uint32_t pins = DL_GPIO_readPins(PIN_INFRARED_TRACK_PORT,
        PIN_INFRARED_TRACK_1 | PIN_INFRARED_TRACK_2 |
        PIN_INFRARED_TRACK_3 | PIN_INFRARED_TRACK_4 |
        PIN_INFRARED_TRACK_5 | PIN_INFRARED_TRACK_6 |
        PIN_INFRARED_TRACK_7 | PIN_INFRARED_TRACK_8);

    for (index = 0U; index < INFRARED_TRACK_SENSOR_COUNT; ++index) {
        uint8_t levelHigh =
            ((pins & g_infraredTrackMap[index].pin) != 0U) ? 1U : 0U;

#if GRAY_DIGITAL_ACTIVE_HIGH
        if (levelHigh != 0U) {
#else
        if (levelHigh == 0U) {
#endif
            mask |= InfraredTrack_BitForIndex(index);
        }
    }
    return mask;
}

#endif
