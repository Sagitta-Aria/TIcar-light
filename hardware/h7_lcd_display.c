/*
 * H7 LCD后端：把0至9行转换成@L命令，只发送发生变化的行。
 * GMR通过UART2/PB15发送，Full配置沿用UART0/PA10。
 * UI任务负责刷新，ISR和10ms控制任务只能更新业务状态，不能直接阻塞发送显示数据。
 * UART忙时未发送内容会保留到下次重试，不应把显示失败当成电机控制失败。
 */
#include "h7_lcd_display.h"

#include "library_config.h"

#if CAR_LIBRARY_H7_LCD_ENABLED

#if CAR_PROFILE_IS_GMR
#include "h7_control_uart.h"
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
static char g_h7LcdTimerText[6];
static volatile uint8_t g_h7LcdTimerMode;
static volatile uint8_t g_h7LcdTimerDirty;

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
    return H7ControlUart_TrySendBytes(data, length);
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
    g_h7LcdTimerText[0] = '\0';
    g_h7LcdTimerMode = 0U;
    g_h7LcdTimerDirty = 0U;
}

void H7LcdDisplay_SetReady(uint8_t ready)
{
    uint32_t primask = H7LcdDisplay_EnterCritical();

    g_h7LcdReady = (ready != 0U) ? 1U : 0U;
    if (g_h7LcdReady != 0U) {
        if (g_h7LcdTimerMode != 0U) {
            g_h7LcdTimerDirty = 1U;
        } else {
            g_h7LcdClearPending = 1U;
            g_h7LcdDirtyRows = H7_LCD_DISPLAY_ALL_ROWS_DIRTY;
        }
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
    g_h7LcdTimerMode = 0U;
    g_h7LcdTimerDirty = 0U;
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
    if (g_h7LcdTimerMode != 0U) {
        g_h7LcdTimerMode = 0U;
        g_h7LcdTimerDirty = 0U;
        g_h7LcdClearPending = 1U;
        g_h7LcdDirtyRows = H7_LCD_DISPLAY_ALL_ROWS_DIRTY;
    }
    if (H7LcdDisplay_TextEquals(g_h7LcdRows[row], next) == 0U) {
        H7LcdDisplay_CopyText(g_h7LcdRows[row], next);
        g_h7LcdDirtyRows |= (uint16_t)(1U << row);
    }
    H7LcdDisplay_ExitCritical(primask);
}

void H7LcdDisplay_ShowTimer(uint32_t elapsedSeconds)
{
    char next[6];
    uint32_t minutes = elapsedSeconds / 60U;
    uint32_t primask;

    if (minutes > 99U) {
        minutes = 99U;
        elapsedSeconds = (99U * 60U) + 59U;
    }
    next[0] = (char)('0' + (minutes / 10U));
    next[1] = (char)('0' + (minutes % 10U));
    next[2] = ':';
    next[3] = (char)('0' + ((elapsedSeconds / 10U) % 6U));
    next[4] = (char)('0' + (elapsedSeconds % 10U));
    next[5] = '\0';

    primask = H7LcdDisplay_EnterCritical();
    if ((g_h7LcdTimerMode == 0U) ||
        (H7LcdDisplay_TextEquals(g_h7LcdTimerText, next) == 0U)) {
        H7LcdDisplay_CopyText(g_h7LcdTimerText, next);
        g_h7LcdTimerMode = 1U;
        g_h7LcdTimerDirty = 1U;
        g_h7LcdClearPending = 0U;
    }
    H7LcdDisplay_ExitCritical(primask);
}

void H7LcdDisplay_Refresh(void)
{
    static const uint8_t clearCommand[] = "@CLEAR\n";
    uint8_t timerCommand[9];
    char line[H7_LCD_DISPLAY_MAX_CHARS + 1U];
    uint32_t primask;
    uint8_t clearPending;
    uint8_t timerDirty;
    uint8_t timerMode;
    uint8_t row;

    if (g_h7LcdReady == 0U) {
        return;
    }

    primask = H7LcdDisplay_EnterCritical();
    timerMode = g_h7LcdTimerMode;
    timerDirty = g_h7LcdTimerDirty;
    if ((timerMode != 0U) && (timerDirty != 0U)) {
        timerCommand[0] = (uint8_t)'@';
        timerCommand[1] = (uint8_t)'T';
        timerCommand[2] = (uint8_t)'=';
        timerCommand[3] = (uint8_t)g_h7LcdTimerText[0];
        timerCommand[4] = (uint8_t)g_h7LcdTimerText[1];
        timerCommand[5] = (uint8_t)g_h7LcdTimerText[2];
        timerCommand[6] = (uint8_t)g_h7LcdTimerText[3];
        timerCommand[7] = (uint8_t)g_h7LcdTimerText[4];
        timerCommand[8] = (uint8_t)'\n';
        g_h7LcdTimerDirty = 0U;
    }
    H7LcdDisplay_ExitCritical(primask);
    if (timerMode != 0U) {
        if ((timerDirty != 0U) &&
            (H7LcdDisplay_TrySendBytes(timerCommand,
                (uint16_t)sizeof(timerCommand)) == 0U)) {
            primask = H7LcdDisplay_EnterCritical();
            g_h7LcdTimerDirty = 1U;
            H7LcdDisplay_ExitCritical(primask);
        }
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
