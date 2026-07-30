#ifndef INFRARED_TRACK_H
#define INFRARED_TRACK_H

#include <stdint.h>

#include "library_config.h"

#if CAR_LIBRARY_GRAY_INPUT_IS_INFRARED_8

#define INFRARED_TRACK_SENSOR_COUNT    (8U)

/* 初始化八路数字输入；S1至S8按车头朝前时从左到右排列。 */
void InfraredTrack_Init(void);

/* 读取黑线状态；高电平为黑线，bit7至bit0依次对应S1至S8。 */
uint8_t InfraredTrack_ReadMask(void);

#endif

#endif
