#include "menu.h"

#include "board_config.h"
#include "gray.h"
#include "log_uart.h"
#include "oled.h"

#define MENU_OLED_FONT_SIZE        (12U)
#define MENU_OLED_MAX_CHARS        (21U)
#define MENU_MONO_START_X          (6U)
#define MENU_MONO_START_Y          (16U)
#define MENU_MONO_LINE_STEP        (12U)
#define MENU_MONO_MAX_CHARS        (18U)
#define MENU_LINE_BUFFER_SIZE      (32U)

typedef enum {
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_MISSION
} MenuPage;

typedef enum {
    MENU_MAIN_GRAY_CALIB = 0,
    MENU_MAIN_TRACK_TEST,
    MENU_MAIN_MISSION,
    MENU_MAIN_COUNT
} MenuMainItem;

typedef enum {
    MENU_MISSION_1 = 0,
    MENU_MISSION_2,
    MENU_MISSION_3,
    MENU_MISSION_4,
    MENU_MISSION_BACK,
    MENU_MISSION_COUNT
} MenuMissionItem;

typedef enum {
    MENU_GRAY_SAVE_EXIT = 0,
    MENU_GRAY_NO_SAVE_EXIT,
    MENU_GRAY_ACTION_COUNT
} MenuGrayAction;

static const char *const g_mainItems[MENU_MAIN_COUNT] = {
    "Gray Calib",
    "Track Test",
    "Mission"
};

static const char *const g_missionItems[MENU_MISSION_COUNT] = {
    "Mission 1",
    "Mission 2",
    "Mission 3",
    "Mission 4",
    "Back"
};

static MenuPage g_menuPage;
static uint8_t g_mainIndex;
static uint8_t g_missionIndex;
static uint8_t g_grayActionIndex;
static uint8_t g_forceRefresh;
static uint16_t g_refreshTicks;
static CarState g_lastState;

/*
 * 作用：向字符串缓冲区追加普通文本。
 * 使用场景：构造 OLED 菜单行。
 */
static char *Menu_AppendText(char *write, char *end, const char *text)
{
    while ((text != NULL) && (*text != '\0') && (write < end)) {
        *write = *text;
        ++write;
        ++text;
    }
    *write = '\0';
    return write;
}

/*
 * 作用：向字符串缓冲区追加单个字符。
 * 使用场景：构造选择箭头和状态文本。
 */
static char *Menu_AppendChar(char *write, char *end, char value)
{
    if (write < end) {
        *write = value;
        ++write;
    }
    *write = '\0';
    return write;
}

/*
 * 作用：向字符串缓冲区追加无符号十进制整数。
 * 使用场景：任务运行页显示当前任务编号。
 */
static char *Menu_AppendUnsigned(char *write, char *end, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count] = (char)('0' + (value % 10U));
        value /= 10U;
        ++count;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));

    while ((count > 0U) && (write < end)) {
        --count;
        *write = digits[count];
        ++write;
    }
    *write = '\0';
    return write;
}

/*
 * 作用：把一行文本补齐后写入 OLED。
 * 使用场景：避免短字符串覆盖不了上一屏残留字符。
 */
static void Menu_ShowPaddedLine(uint8_t line, const char *text)
{
    char padded[MENU_OLED_MAX_CHARS + 1U];
    uint8_t i;

    for (i = 0U; i < MENU_OLED_MAX_CHARS; ++i) {
        padded[i] = ' ';
    }
    padded[MENU_OLED_MAX_CHARS] = '\0';

    i = 0U;
    while ((text != NULL) && (text[i] != '\0') &&
        (i < MENU_OLED_MAX_CHARS)) {
        padded[i] = text[i];
        ++i;
    }

    OLED_ShowLine(line, padded, MENU_OLED_FONT_SIZE);
}

/*
 * 作用：在 OLED 下半区显示一行菜单文字。
 * 使用场景：避开双色屏顶部黄区。
 */
static void Menu_ShowMonoLine(uint8_t index, const char *text)
{
    char padded[MENU_MONO_MAX_CHARS + 1U];
    uint8_t i;
    uint8_t y;

    for (i = 0U; i < MENU_MONO_MAX_CHARS; ++i) {
        padded[i] = ' ';
    }
    padded[MENU_MONO_MAX_CHARS] = '\0';

    i = 0U;
    while ((text != NULL) && (text[i] != '\0') &&
        (i < MENU_MONO_MAX_CHARS)) {
        padded[i] = text[i];
        ++i;
    }

    y = (uint8_t)(MENU_MONO_START_Y + (index * MENU_MONO_LINE_STEP));
    OLED_ShowString(MENU_MONO_START_X, y, (u8 *)padded,
        MENU_OLED_FONT_SIZE);
}

static void Menu_ClearOLEDLines(void)
{
    Menu_ShowPaddedLine(0U, "");
    Menu_ShowPaddedLine(1U, "");
    Menu_ShowPaddedLine(2U, "");
    Menu_ShowPaddedLine(3U, "");
    Menu_ShowPaddedLine(4U, "");
}

/*
 * 作用：刷新 4 行下半屏内容。
 * 使用场景：菜单页和运行状态页。
 */
static void Menu_RenderLines(const char *line0, const char *line1,
    const char *line2, const char *line3)
{
    Menu_ClearOLEDLines();
    Menu_ShowMonoLine(0U, line0);
    Menu_ShowMonoLine(1U, line1);
    Menu_ShowMonoLine(2U, line2);
    Menu_ShowMonoLine(3U, line3);
    OLED_Refresh();
}

/*
 * 作用：构造带选择箭头的菜单行。
 * 使用场景：主菜单、任务菜单和灰度校准退出选择。
 */
static void Menu_BuildItemLine(char line[MENU_LINE_BUFFER_SIZE],
    uint8_t selected, const char *text)
{
    char *write = line;
    char *end = &line[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendChar(write, end, selected ? '>' : ' ');
    (void)Menu_AppendText(write, end, text);
}

/*
 * 作用：渲染三行滚动菜单，当前选中项固定在中间行。
 * 使用场景：按 K2 切换选项时。
 */
static void Menu_RenderCenteredList(const char *const *items, uint8_t count,
    uint8_t selected)
{
    char line0[MENU_LINE_BUFFER_SIZE];
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    uint8_t previous;
    uint8_t next;

    if (count == 0U) {
        return;
    }

    previous = (selected == 0U) ? (uint8_t)(count - 1U) :
        (uint8_t)(selected - 1U);
    next = (uint8_t)((selected + 1U) % count);

    Menu_BuildItemLine(line0, 0U, items[previous]);
    Menu_BuildItemLine(line1, 1U, items[selected]);
    Menu_BuildItemLine(line2, 0U, items[next]);
    Menu_RenderLines(line0, line1, line2, "");
}

static void Menu_RenderMainMenu(void)
{
    Menu_RenderCenteredList(g_mainItems, MENU_MAIN_COUNT, g_mainIndex);
}

static void Menu_RenderMissionMenu(void)
{
    Menu_RenderCenteredList(g_missionItems, MENU_MISSION_COUNT,
        g_missionIndex);
}

static void Menu_RenderGrayCalibration(void)
{
    char statusLine[MENU_LINE_BUFFER_SIZE];
    char saveLine[MENU_LINE_BUFFER_SIZE];
    char noSaveLine[MENU_LINE_BUFFER_SIZE];
    char *write = statusLine;
    char *end = &statusLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "Cal Done: ");
    (void)Menu_AppendText(write, end,
        (Gray_IsCalibrationComplete() != 0U) ? "YES" : "NO");

    Menu_BuildItemLine(saveLine,
        (g_grayActionIndex == MENU_GRAY_SAVE_EXIT) ? 1U : 0U,
        "Save Exit");
    Menu_BuildItemLine(noSaveLine,
        (g_grayActionIndex == MENU_GRAY_NO_SAVE_EXIT) ? 1U : 0U,
        "No Save Exit");

    Menu_RenderLines(statusLine, saveLine, noSaveLine, "");
}

static void Menu_RenderTrackingTest(void)
{
    Menu_RenderLines("Track Test", "Running", "K2 Back", "");
}

static void Menu_RenderMission(void)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char *write = line1;
    char *end = &line1[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "Mission ");
    (void)Menu_AppendUnsigned(write, end, StateMachine_GetMissionId());
    Menu_RenderLines("Mission", line1, "K2 Back", "");
}

static void Menu_RenderSimpleState(const char *title, const char *line1)
{
    Menu_RenderLines(title, line1, "K1 Menu", "K2 Back");
}

static void Menu_RenderByState(CarState state)
{
    switch (state) {
    case CAR_STATE_MENU:
        if (g_menuPage == MENU_PAGE_MISSION) {
            Menu_RenderMissionMenu();
        } else {
            Menu_RenderMainMenu();
        }
        break;
    case CAR_STATE_GRAY_CALIBRATION:
        Menu_RenderGrayCalibration();
        break;
    case CAR_STATE_TRACKING_TEST:
        Menu_RenderTrackingTest();
        break;
    case CAR_STATE_MISSION:
        Menu_RenderMission();
        break;
    case CAR_STATE_TRACKING:
        Menu_RenderSimpleState("Tracking", "Running");
        break;
    case CAR_STATE_MOTOR_TEST:
        Menu_RenderSimpleState("Motor Test", "Running");
        break;
    case CAR_STATE_FINISHED:
        Menu_RenderSimpleState("Finished", "Mission done");
        break;
    case CAR_STATE_STOP:
        Menu_RenderSimpleState("Stopped", "Car stopped");
        break;
    case CAR_STATE_ERROR:
        Menu_RenderSimpleState("Error", "Check hardware");
        break;
    case CAR_STATE_IDLE:
        Menu_RenderSimpleState("Idle", "Waiting");
        break;
    case CAR_STATE_INIT:
    default:
        Menu_RenderSimpleState("Init", "Starting");
        break;
    }
}

void Menu_Init(void)
{
    g_menuPage = MENU_PAGE_MAIN;
    g_mainIndex = MENU_MAIN_GRAY_CALIB;
    g_missionIndex = MENU_MISSION_1;
    g_grayActionIndex = MENU_GRAY_SAVE_EXIT;
    g_forceRefresh = 1U;
    g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    g_lastState = CAR_STATE_INIT;
}

void Menu_Next(void)
{
    if (g_menuPage == MENU_PAGE_MISSION) {
        g_missionIndex = (uint8_t)((g_missionIndex + 1U) %
            MENU_MISSION_COUNT);
        LOG_RAW("menu: select ");
        LOG_LINE(g_missionItems[g_missionIndex]);
    } else {
        g_mainIndex = (uint8_t)((g_mainIndex + 1U) % MENU_MAIN_COUNT);
        LOG_RAW("menu: select ");
        LOG_LINE(g_mainItems[g_mainIndex]);
    }
    g_forceRefresh = 1U;
}

CarEvent Menu_Confirm(void)
{
    if (g_menuPage == MENU_PAGE_MAIN) {
        LOG_RAW("menu: confirm ");
        LOG_LINE(g_mainItems[g_mainIndex]);
        switch (g_mainIndex) {
        case MENU_MAIN_GRAY_CALIB:
            g_grayActionIndex = MENU_GRAY_SAVE_EXIT;
            return CAR_EVENT_GRAY_CALIBRATION_START;
        case MENU_MAIN_TRACK_TEST:
            return CAR_EVENT_TRACKING_TEST_START;
        case MENU_MAIN_MISSION:
            g_menuPage = MENU_PAGE_MISSION;
            g_missionIndex = MENU_MISSION_1;
            g_forceRefresh = 1U;
            return CAR_EVENT_NONE;
        default:
            return CAR_EVENT_NONE;
        }
    }

    LOG_RAW("menu: confirm ");
    LOG_LINE(g_missionItems[g_missionIndex]);
    switch (g_missionIndex) {
    case MENU_MISSION_1:
        return CAR_EVENT_MISSION_1_START;
    case MENU_MISSION_2:
        return CAR_EVENT_MISSION_2_START;
    case MENU_MISSION_3:
        return CAR_EVENT_MISSION_3_START;
    case MENU_MISSION_4:
        return CAR_EVENT_MISSION_4_START;
    case MENU_MISSION_BACK:
        g_menuPage = MENU_PAGE_MAIN;
        g_mainIndex = MENU_MAIN_MISSION;
        g_forceRefresh = 1U;
        return CAR_EVENT_NONE;
    default:
        return CAR_EVENT_NONE;
    }
}

void Menu_GrayCalibrationNext(void)
{
    g_grayActionIndex = (uint8_t)((g_grayActionIndex + 1U) %
        MENU_GRAY_ACTION_COUNT);
    g_forceRefresh = 1U;
}

CarEvent Menu_GrayCalibrationConfirm(void)
{
    if (g_grayActionIndex == MENU_GRAY_SAVE_EXIT) {
        if (Gray_IsCalibrationComplete() == 0U) {
            LOG_LINE("gray menu: calibration not complete");
            return CAR_EVENT_NONE;
        }
        LOG_LINE("gray menu: save exit");
        return CAR_EVENT_GRAY_CALIBRATION_APPLY;
    }

    LOG_LINE("gray menu: no save exit");
    return CAR_EVENT_BACK;
}

void Menu_Task(CarState state)
{
    if (state != g_lastState) {
        if (state == CAR_STATE_MENU) {
            g_menuPage = MENU_PAGE_MAIN;
            g_mainIndex = MENU_MAIN_GRAY_CALIB;
        }
        g_lastState = state;
        g_forceRefresh = 1U;
        g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    } else if (g_refreshTicks < CAR_MENU_REFRESH_TICKS) {
        ++g_refreshTicks;
    }

    if (!g_forceRefresh && (g_refreshTicks < CAR_MENU_REFRESH_TICKS)) {
        return;
    }

    g_forceRefresh = 0U;
    g_refreshTicks = 0U;
    Menu_RenderByState(state);
}
