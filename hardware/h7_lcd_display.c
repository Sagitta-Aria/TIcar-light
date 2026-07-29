/*
 * H7 LCD后端：把0至9行转换成@L命令，只发送发生变化的行。
 * GMR通过UART3/PB2发送，普通配置沿用UART0/PA10。
 * UI任务负责刷新，ISR和10ms控制任务只能更新业务状态，不能直接阻塞发送显示数据。
 * UART忙时未发送内容会保留到下次重试，不应把显示失败当成电机控制失败。
 */
#include "h7_lcd_display.h"

#include "library_config.h"

#if CAR_LIBRARY_H7_LCD_ENABLED

#if CAR_PROFILE_IS_GMR
#include "h7_gyro_link.h"
#else
#include "log_uart.h"
#endif
#include "ti_msp_dl_config.h"

#define H7_LCD_DISPLAY_ALL_ROWS_DIRTY (0x03FFU)
#define H7_LCD_DISPLAY_COMMAND_SIZE \
    (4U + H7_LCD_DISPLAY_MAX_CHARS + 1U)

static char g_h7LcdRows[H7_LCD_DISPLAY_ROW_COUNT]
    [H7_LCD_DISPLAY_MAX_CHARS + 1U];
static volatile uint16_t g_h7LcdDirtyRows;
static volatile uint8_t g_h7LcdClearPending;
static volatile uint8_t g_h7LcdReady;

static uint32_t H7LcdDisplay_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void H7LcdDisplay_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint8_t H7LcdDisplay_TextEquals(const char *left, const char *right)
{
    uint8_t index;

    for (index = 0U; index <= H7_LCD_DISPLAY_MAX_CHARS; ++index) {
        if (left[index] != right[index]) {
            return 0U;
        }
        if (left[index] == '\0') {
            return 1U;
        }
    }
    return 1U;
}

static void H7LcdDisplay_CopyText(char *destination, const char *source)
{
    uint8_t index = 0U;

    while ((source != 0) && (source[index] != '\0') &&
        (index < H7_LCD_DISPLAY_MAX_CHARS)) {
        char value = source[index];

        destination[index] = ((value >= 0x20) && (value <= 0x7E)) ?
            value : '?';
        ++index;
    }
    destination[index] = '\0';
}

static uint8_t H7LcdDisplay_TrySendBytes(const uint8_t *data,
    uint16_t length)
{
#if CAR_PROFILE_IS_GMR
    return H7GyroLink_TrySendBytes(data, length);
#else
    return LogUart_TrySendBytes(data, length);
#endif
}

static uint8_t H7LcdDisplay_SendRow(uint8_t row, const char *text)
{
    uint8_t command[H7_LCD_DISPLAY_COMMAND_SIZE];
    uint16_t length = 0U;
    uint8_t index = 0U;

    command[length++] = (uint8_t)'@';
    command[length++] = (uint8_t)'L';
    command[length++] = (uint8_t)('0' + row);
    command[length++] = (uint8_t)'=';
    while ((text[index] != '\0') &&
        (index < H7_LCD_DISPLAY_MAX_CHARS)) {
        command[length++] = (uint8_t)text[index];
        ++index;
    }
    command[length++] = (uint8_t)'\n';
    return H7LcdDisplay_TrySendBytes(command, length);
}

void H7LcdDisplay_Init(void)
{
    uint8_t row;

    for (row = 0U; row < H7_LCD_DISPLAY_ROW_COUNT; ++row) {
        g_h7LcdRows[row][0] = '\0';
    }
    g_h7LcdDirtyRows = 0U;
    g_h7LcdClearPending = 1U;
    g_h7LcdReady = 0U;
}

void H7LcdDisplay_SetReady(uint8_t ready)
{
    uint32_t primask = H7LcdDisplay_EnterCritical();

    g_h7LcdReady = (ready != 0U) ? 1U : 0U;
    if (g_h7LcdReady != 0U) {
        g_h7LcdClearPending = 1U;
        g_h7LcdDirtyRows = H7_LCD_DISPLAY_ALL_ROWS_DIRTY;
    }
    H7LcdDisplay_ExitCritical(primask);
}

void H7LcdDisplay_Clear(void)
{
    uint32_t primask = H7LcdDisplay_EnterCritical();
    uint8_t row;

    for (row = 0U; row < H7_LCD_DISPLAY_ROW_COUNT; ++row) {
        g_h7LcdRows[row][0] = '\0';
    }
    g_h7LcdDirtyRows = 0U;
    g_h7LcdClearPending = 1U;
    H7LcdDisplay_ExitCritical(primask);
}

void H7LcdDisplay_ShowLine(uint8_t row, const char *text)
{
    char next[H7_LCD_DISPLAY_MAX_CHARS + 1U];
    uint32_t primask;

    if (row >= H7_LCD_DISPLAY_ROW_COUNT) {
        return;
    }

    H7LcdDisplay_CopyText(next, text);
    primask = H7LcdDisplay_EnterCritical();
    if (H7LcdDisplay_TextEquals(g_h7LcdRows[row], next) == 0U) {
        H7LcdDisplay_CopyText(g_h7LcdRows[row], next);
        g_h7LcdDirtyRows |= (uint16_t)(1U << row);
    }
    H7LcdDisplay_ExitCritical(primask);
}

void H7LcdDisplay_Refresh(void)
{
    static const uint8_t clearCommand[] = "@CLEAR\n";
    char line[H7_LCD_DISPLAY_MAX_CHARS + 1U];
    uint32_t primask;
    uint8_t clearPending;
    uint8_t row;

    if (g_h7LcdReady == 0U) {
        return;
    }

    primask = H7LcdDisplay_EnterCritical();
    clearPending = g_h7LcdClearPending;
    g_h7LcdClearPending = 0U;
    H7LcdDisplay_ExitCritical(primask);
    if ((clearPending != 0U) &&
        (H7LcdDisplay_TrySendBytes(clearCommand,
            (uint16_t)(sizeof(clearCommand) - 1U)) == 0U)) {
        primask = H7LcdDisplay_EnterCritical();
        g_h7LcdClearPending = 1U;
        H7LcdDisplay_ExitCritical(primask);
        return;
    }

    for (row = 0U; row < H7_LCD_DISPLAY_ROW_COUNT; ++row) {
        primask = H7LcdDisplay_EnterCritical();
        if ((g_h7LcdDirtyRows & (uint16_t)(1U << row)) == 0U) {
            H7LcdDisplay_ExitCritical(primask);
            continue;
        }
        H7LcdDisplay_CopyText(line, g_h7LcdRows[row]);
        g_h7LcdDirtyRows &= (uint16_t)~(1U << row);
        H7LcdDisplay_ExitCritical(primask);

        if (H7LcdDisplay_SendRow(row, line) == 0U) {
            primask = H7LcdDisplay_EnterCritical();
            g_h7LcdDirtyRows |= (uint16_t)(1U << row);
            H7LcdDisplay_ExitCritical(primask);
            return;
        }
    }
}

uint8_t H7LcdDisplay_IsReady(void)
{
    return g_h7LcdReady;
}

#endif
