#ifndef TRACKING_H
#define TRACKING_H

#include <stdint.h>

/* Tracking_Init：初始化循迹模块和异常处理器，默认不输出底盘命令。 */
void Tracking_Init(void);

/* Tracking_Task：采样灰度并输出底盘 STEP 命令；仅在 enabled=1 时工作。 */
void Tracking_Task(void);

/* Tracking_SetEnabled：启停循迹；关闭时会立即停车并清空异常状态。 */
void Tracking_SetEnabled(uint8_t enabled);

/* Tracking_IsEnabled：读取循迹模块是否正在参与主循环控制。 */
uint8_t Tracking_IsEnabled(void);

#endif
