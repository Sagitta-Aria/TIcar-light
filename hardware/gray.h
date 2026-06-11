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

/* Gray_Init：初始化灰度输入、阈值和滤波状态。 */
void Gray_Init(void);

/* Gray_StartConversion：模拟模式启动 ADC；数字模式下为空操作。 */
void Gray_StartConversion(void);

/* Gray_Update：读取滤波后的原始值，并刷新黑白状态。 */
uint8_t Gray_Update(void);

/* Gray_GetRaw：获取某一路最新原始值；数字模式下为 0 或 4095。 */
uint16_t Gray_GetRaw(GrayChannel channel);

/* Gray_GetDigital：获取某一路最新的防抖黑白状态。 */
uint8_t Gray_GetDigital(GrayChannel channel);

/* Gray_GetDigitalMask：返回参与循迹的黑白状态，当前 bit6~bit0 对应 S1~S7。 */
uint8_t Gray_GetDigitalMask(void);

/* Gray_GetLineError：基于黑白状态返回粗略偏差，范围约为 -3~+3。 */
uint8_t Gray_GetLineError(int16_t *error);

/* Gray_GetWeightedLineError：基于灰度强度返回加权偏差。 */
uint8_t Gray_GetWeightedLineError(int16_t *error);

/* Gray_IsLineLost：当没有任何通道压线时返回 1。 */
uint8_t Gray_IsLineLost(void);

/* Gray_SetThreshold：设置模拟模式阈值，数字模式下不参与判断。 */
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

/* Gray_IsCalibrationComplete：校准采样是否已经覆盖每一路的有效变化。 */
uint8_t Gray_IsCalibrationComplete(void);

/* Gray_CalibrationApply：应用校准结果；数字模式下只刷新状态。 */
void Gray_CalibrationApply(void);

#endif
