#ifndef GRAY_H
#define GRAY_H

#include <stdint.h>

#include "board_config.h"

/* GrayChannel：灰度传感器编号，按车头朝前时从左到右排列。 */
typedef enum {
    GRAY_1 = 0,
    GRAY_2,
    GRAY_3,
    GRAY_4,
    GRAY_5,
    GRAY_6,
    GRAY_7
} GrayChannel;

/* Gray_Init：初始化灰度 ADC、阈值和滤波状态。 */
void Gray_Init(void);

/* Gray_StartConversion：启动一轮 ADC 序列采样。 */
void Gray_StartConversion(void);

/* Gray_Update：读取滤波后的原始值，并刷新黑白状态。 */
uint8_t Gray_Update(void);

/* Gray_GetRaw：获取某一路最新的滤波后 ADC 值。 */
uint16_t Gray_GetRaw(GrayChannel channel);

/* Gray_GetDigital：获取某一路最新的防抖黑白状态。 */
uint8_t Gray_GetDigital(GrayChannel channel);

/* Gray_GetDigitalMask：把 7 路黑白状态打包成 bit0~bit6。 */
uint8_t Gray_GetDigitalMask(void);

/* Gray_GetLineError：基于黑白状态返回粗略偏差，范围约为 -3~+3。 */
uint8_t Gray_GetLineError(int16_t *error);

/* Gray_GetWeightedLineError：基于 ADC 强度返回加权偏差。 */
uint8_t Gray_GetWeightedLineError(int16_t *error);

/* Gray_IsLineLost：当没有任何通道压线时返回 1。 */
uint8_t Gray_IsLineLost(void);

/* Gray_SetThreshold：设置某一路阈值，自动限制在 ADC 范围内。 */
void Gray_SetThreshold(GrayChannel channel, uint16_t threshold);

/* Gray_GetThreshold：读取某一路阈值。 */
uint16_t Gray_GetThreshold(GrayChannel channel);

/* Gray_ReadRaw：更新一次并拷贝全部滤波后原始值。 */
void Gray_ReadRaw(uint16_t values[GRAY_SENSOR_COUNT]);

/* Gray_ReadDigital：更新一次并拷贝全部防抖黑白状态。 */
void Gray_ReadDigital(uint8_t values[GRAY_SENSOR_COUNT]);

/* Gray_CalibrationReset：开始校准前清空最小/最大值记录。 */
void Gray_CalibrationReset(void);

/* Gray_CalibrationSample：采集一组校准数据，更新最小/最大值。 */
void Gray_CalibrationSample(void);

/* Gray_CalibrationApply：用采样到的最小/最大值中点生成阈值。 */
void Gray_CalibrationApply(void);

#endif
