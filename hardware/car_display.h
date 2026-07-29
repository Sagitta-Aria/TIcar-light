#ifndef CAR_DISPLAY_H
#define CAR_DISPLAY_H

#include <stdint.h>

/*
 * CarDisplay：比赛界面的统一显示出口。
 *
 * 使用场景：Board启动页、Menu任务和App任务状态页统一调用本接口。
 * 不要使用：ISR、CarControl或Gimbal的10ms控制路径；本地OLED刷新会同步访问I2C。
 * 后端选择：由library_config.h独立控制H7 UART LCD和本地I2C OLED，可同时启用。
 */

/* 初始化各显示后端的软件缓存；Board_Init在GPIO上电后调用一次。 */
void CarDisplay_Init(void);

/*
 * 初始化本地OLED的I2C外设和SSD1306。
 * 返回1表示OLED已可用；返回0表示方法未启用或I2C初始化失败。
 * 必须在系统时钟稳定后调用，不能在运行期控制任务中重复调用。
 */
uint8_t CarDisplay_InitLocalOled(void);

/* UART0/PA10初始化完成后标记H7 LCD可发送；关闭时只影响H7后端。 */
void CarDisplay_SetH7Ready(uint8_t ready);

/* 清空所有已启用显示后端；本地OLED会立即执行一次I2C清屏。 */
void CarDisplay_Clear(void);

/*
 * 更新一行ASCII文本缓存。
 * H7支持0至9行；本地OLED行数和字符数由board_config.h字体参数自动计算。
 * 超出某个后端范围的行只会被该后端忽略，不影响另一个后端。
 */
void CarDisplay_ShowLine(uint8_t row, const char *text);

/* 把缓存变化输出到所有已启用后端；只能在低优先级UI/启动流程调用。 */
void CarDisplay_Refresh(void);

/* 任意一个启用的显示后端已经完成硬件初始化时返回1。 */
uint8_t CarDisplay_IsAvailable(void);

#endif
