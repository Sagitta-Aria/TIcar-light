#ifndef H7_LCD_DISPLAY_H
#define H7_LCD_DISPLAY_H

#include <stdint.h>

#define H7_LCD_DISPLAY_ROW_COUNT  (10U)
#define H7_LCD_DISPLAY_MAX_CHARS  (33U)

/* 初始化10行文本缓存；UART0就绪前可先写缓存，但不会立即发送。 */
void H7LcdDisplay_Init(void);

/* H7 UART初始化完成后启用输出，并安排一次完整页面重发。 */
void H7LcdDisplay_SetReady(uint8_t ready);

/* 清空页面缓存；调用Refresh后向H7发送@CLEAR。 */
void H7LcdDisplay_Clear(void);

/* 更新固定行0~9；只缓存ASCII前33字符，不在调用点阻塞发送。 */
void H7LcdDisplay_ShowLine(uint8_t row, const char *text);

/* 缓存Task2计时页面；刷新时发送@T=MM:SS专用命令。 */
void H7LcdDisplay_ShowTimer(uint32_t elapsedSeconds);

/* 发送本次变更；每条@命令独占H7 TX，发送失败的行会保留待重试。 */
void H7LcdDisplay_Refresh(void);

/* UART0显示输出已经启用时返回1。 */
uint8_t H7LcdDisplay_IsReady(void);

#endif
