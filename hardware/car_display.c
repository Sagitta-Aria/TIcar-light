/*
 * H7 UART LCD与本地I2C OLED的统一显示分发器。
 * 上层菜单不感知具体屏幕；编译期开关会裁掉未选择后端的调用和硬件初始化。
 * 本地OLED使用同步I2C整屏刷新，只允许启动流程和低优先级UI任务调用。
 */
#include "car_display.h"

#include "board_config.h"

#if CAR_LIBRARY_H7_LCD_ENABLED
#include "h7_lcd_display.h"
#endif

#if CAR_LIBRARY_LOCAL_OLED_ENABLED
#include "oled.h"
#include "ti_msp_dl_config.h"
#endif

static uint8_t g_carDisplayLocalOledReady;
static uint32_t g_carDisplayLocalTimerSeconds;

/* 重置后端软件状态，不访问尚未初始化的UART或I2C硬件。 */
void CarDisplay_Init(void)
{
#if CAR_LIBRARY_H7_LCD_ENABLED
    H7LcdDisplay_Init();
#endif
    g_carDisplayLocalOledReady = 0U;
    g_carDisplayLocalTimerSeconds = 0xFFFFFFFFUL;
}

/* 系统时钟稳定后初始化本地OLED；失败时保留不可用状态供Board层报错。 */
uint8_t CarDisplay_InitLocalOled(void)
{
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    SYSCFG_DL_OLED_init();
    OLED_Init();
    if (OLED_HasError() != 0U) {
        g_carDisplayLocalOledReady = 0U;
        return 0U;
    }
    g_carDisplayLocalOledReady = 1U;
    return 1U;
#else
    return 0U;
#endif
}

/* 只转发H7 UART就绪状态；本地OLED就绪状态由I2C初始化结果维护。 */
void CarDisplay_SetH7Ready(uint8_t ready)
{
#if CAR_LIBRARY_H7_LCD_ENABLED
    H7LcdDisplay_SetReady(ready);
#else
    (void)ready;
#endif
}

/* 清空所有可用后端；OLED_Clear内部会立即把空显存发送到屏幕。 */
void CarDisplay_Clear(void)
{
#if CAR_LIBRARY_H7_LCD_ENABLED
    H7LcdDisplay_Clear();
#endif
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    if (g_carDisplayLocalOledReady != 0U) {
        OLED_Clear();
    }
#endif
    g_carDisplayLocalTimerSeconds = 0xFFFFFFFFUL;
}

#if CAR_LIBRARY_LOCAL_OLED_ENABLED
/*
 * 把任意上层文本转换成OLED可显示的定长ASCII行。
 * 填充空格用于覆盖上一页残留字符；非ASCII字符替换成问号。
 */
static void CarDisplay_ShowLocalOledLine(uint8_t row, const char *text)
{
    char line[CAR_LOCAL_OLED_MAX_CHARS_PER_ROW + 1U];
    uint8_t index;

    if ((g_carDisplayLocalOledReady == 0U) ||
        (row >= (uint8_t)CAR_LOCAL_OLED_VISIBLE_ROWS)) {
        return;
    }

    for (index = 0U; index <
        (uint8_t)CAR_LOCAL_OLED_MAX_CHARS_PER_ROW; ++index) {
        line[index] = ' ';
    }
    for (index = 0U; (text != 0) && (text[index] != '\0') &&
        (index < (uint8_t)CAR_LOCAL_OLED_MAX_CHARS_PER_ROW); ++index) {
        char value = text[index];

        line[index] = ((value >= 0x20) && (value <= 0x7E)) ? value : '?';
    }
    line[CAR_LOCAL_OLED_MAX_CHARS_PER_ROW] = '\0';
    OLED_ShowLine(row, line, (uint8_t)CAR_LOCAL_OLED_FONT_SIZE_PIXELS);
}
#endif

/* 向每个已选择后端更新同一行；具体截断规则由后端独立处理。 */
void CarDisplay_ShowLine(uint8_t row, const char *text)
{
#if CAR_LIBRARY_H7_LCD_ENABLED
    H7LcdDisplay_ShowLine(row, text);
#endif
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    g_carDisplayLocalTimerSeconds = 0xFFFFFFFFUL;
    CarDisplay_ShowLocalOledLine(row, text);
#endif
#if !CAR_LIBRARY_DISPLAY_ENABLED
    (void)row;
    (void)text;
#endif
}

void CarDisplay_ShowTimer(uint32_t elapsedMs)
{
    uint32_t totalSeconds = elapsedMs / 1000U;
    uint32_t minutes = totalSeconds / 60U;
    char text[6];

    if (minutes > 99U) {
        minutes = 99U;
        totalSeconds = (99U * 60U) + 59U;
    }
    text[0] = (char)('0' + (minutes / 10U));
    text[1] = (char)('0' + (minutes % 10U));
    text[2] = ':';
    text[3] = (char)('0' + ((totalSeconds / 10U) % 6U));
    text[4] = (char)('0' + (totalSeconds % 10U));
    text[5] = '\0';
#if CAR_LIBRARY_H7_LCD_ENABLED
    H7LcdDisplay_ShowTimer(totalSeconds);
#endif
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    if ((g_carDisplayLocalOledReady != 0U) &&
        (g_carDisplayLocalTimerSeconds != totalSeconds)) {
        OLED_ClearBuffer();
        OLED_ShowString(34U, 20U, (uint8_t *)text, 24U);
        g_carDisplayLocalTimerSeconds = totalSeconds;
    }
#endif
    (void)text;
}

/* H7只发送脏行；本地OLED发送完整显存，调用方不得放在控制周期内。 */
void CarDisplay_Refresh(void)
{
#if CAR_LIBRARY_H7_LCD_ENABLED
    H7LcdDisplay_Refresh();
#endif
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    if (g_carDisplayLocalOledReady != 0U) {
        OLED_Refresh();
    }
#endif
}

/* 汇总两个后端的硬件就绪状态，供菜单决定是否需要稍后重试。 */
uint8_t CarDisplay_IsAvailable(void)
{
#if CAR_LIBRARY_H7_LCD_ENABLED
    if (H7LcdDisplay_IsReady() != 0U) {
        return 1U;
    }
#endif
#if CAR_LIBRARY_LOCAL_OLED_ENABLED
    if (g_carDisplayLocalOledReady != 0U) {
        return 1U;
    }
#endif
    return 0U;
}
