#include "menu.h"

#include "board.h"
#include "board_config.h"
#include "gimbal_motor_test.h"
#include "gimbal_test.h"
#include "gray.h"
#include "log_uart.h"
#include "motor_no_yaw.h"
#include "motor_track.h"
#include "motor_enable_test.h"
#include "oled.h"
#include "staticconfig.h"
#include "track_step_test.h"

#define MENU_OLED_FONT_SIZE        (12U)
#define MENU_OLED_MAX_CHARS        (21U)
#define MENU_MONO_START_X          (6U)
#define MENU_MONO_START_Y          (16U)
#define MENU_MONO_LINE_STEP        (12U)
#define MENU_MONO_MAX_CHARS        (18U)
#define MENU_LINE_BUFFER_SIZE      (32U)

typedef enum {
    MENU_MAIN_GIMBAL = 0,
    MENU_MAIN_MOTOR,
    MENU_MAIN_MISSION,
    MENU_MAIN_COUNT
} MenuMainItem;

typedef enum {
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_GIMBAL,
    MENU_PAGE_MOTOR
} MenuPage;

typedef enum {
    MENU_MOTOR_STEP_TEST = 0,
    MENU_MOTOR_TRACK_RUN,
    MENU_MOTOR_NO_YAW,
    MENU_MOTOR_GRAY_TEST,
    MENU_MOTOR_COUNT
} MenuMotorItem;

typedef enum {
    MENU_GRAY_SAVE_EXIT = 0,
    MENU_GRAY_NO_SAVE_EXIT,
    MENU_GRAY_ACTION_COUNT
} MenuGrayAction;

static const char *const g_mainItems[MENU_MAIN_COUNT] = {
    "Gimbal",
    "Motor",
    "Mission"
};

static const char *const g_gimbalItems[STATICCONFIG_TASK_COUNT] = {
    "Near Center",
    "Near Circle",
    "Mid Center",
    "Mid Circle",
    "Far Center",
    "Far Circle"
};

static const char *const g_motorItems[MENU_MOTOR_COUNT] = {
    "Step Test",
    "Track Run",
    "NO YAW",
    "Gray Test"
};

static MenuPage g_menuPage;
static uint8_t g_mainIndex;
static uint8_t g_gimbalIndex;
static uint8_t g_motorIndex;
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
    while ((text != 0) && (*text != '\0') && (write < end)) {
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
 * 使用场景：显示 SPS、STEP 计数和任务编号。
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
 * 作用：向字符串缓冲区追加有符号十进制整数。
 * 使用场景：显示云台 offset 这类可正可负的调参值。
 */
static char *Menu_AppendSigned(char *write, char *end, int32_t value)
{
    if (value < 0) {
        write = Menu_AppendChar(write, end, '-');
        value = -value;
    }
    return Menu_AppendUnsigned(write, end, (uint32_t)value);
}

/* 作用：按 S1~S7 顺序追加灰度压线位，1 表示当前通道被判定为灭/压线。 */
static char *Menu_AppendGrayBits(char *write, char *end)
{
    uint32_t i;

    for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
        write = Menu_AppendChar(write, end,
            (Gray_GetDigital((GrayChannel)i) != 0U) ? '1' : '0');
    }
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
    while ((text != 0) && (text[i] != '\0') &&
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
    while ((text != 0) && (text[i] != '\0') &&
        (i < MENU_MONO_MAX_CHARS)) {
        padded[i] = text[i];
        ++i;
    }

    y = (uint8_t)(MENU_MONO_START_Y + (index * MENU_MONO_LINE_STEP));
    OLED_ShowString(MENU_MONO_START_X, y, (u8 *)padded,
        MENU_OLED_FONT_SIZE);
}

/*
 * 作用：清空当前菜单占用的 OLED 行。
 * 使用场景：每次重新渲染页面前，避免旧页面残留字符。
 */
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
    if (Board_IsOledAvailable() == 0U) {
        return;
    }

    Menu_ClearOLEDLines();
    Menu_ShowMonoLine(0U, line0);
    Menu_ShowMonoLine(1U, line1);
    Menu_ShowMonoLine(2U, line2);
    Menu_ShowMonoLine(3U, line3);
    OLED_Refresh();
}

/*
 * 作用：构造带选择箭头的菜单行。
 * 使用场景：主菜单选择项。
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
 * 使用场景：主菜单按 K1 切换选项。
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

/* 作用：渲染主菜单，只保留云台、电机、任务三项。 */
static void Menu_RenderMainMenu(void)
{
    Menu_RenderCenteredList(g_mainItems, MENU_MAIN_COUNT, g_mainIndex);
}

/* 作用：渲染 Gimbal 子页，K1 切换六套参数，K2 启动当前参数。 */
static void Menu_RenderGimbalMenu(void)
{
    const StaticConfigGimbalTask *config =
        StaticConfig_GetTask((StaticConfigTaskId)g_gimbalIndex);
    char titleLine[MENU_LINE_BUFFER_SIZE];
    char gainLine[MENU_LINE_BUFFER_SIZE];
    char deadbandLine[MENU_LINE_BUFFER_SIZE];
    char offsetLine[MENU_LINE_BUFFER_SIZE];
    char *write = titleLine;
    char *end = &titleLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendChar(write, end, '>');
    (void)Menu_AppendText(write, end, g_gimbalItems[g_gimbalIndex]);

    write = gainLine;
    end = &gainLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "KP");
    write = Menu_AppendUnsigned(write, end, config->kpX);
    write = Menu_AppendChar(write, end, '/');
    write = Menu_AppendUnsigned(write, end, config->kpY);
    write = Menu_AppendText(write, end, " KD");
    write = Menu_AppendUnsigned(write, end, config->kdX);
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end, config->kdY);

    write = deadbandLine;
    end = &deadbandLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "DB");
    write = Menu_AppendUnsigned(write, end, config->deadbandX);
    write = Menu_AppendChar(write, end, '/');
    write = Menu_AppendUnsigned(write, end, config->deadbandY);
    write = Menu_AppendText(write, end, " SP");
    write = Menu_AppendUnsigned(write, end, config->minSpeedX);
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end, config->minSpeedY);

    write = offsetLine;
    end = &offsetLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "OF");
    write = Menu_AppendSigned(write, end, config->offsetX);
    write = Menu_AppendChar(write, end, '/');
    write = Menu_AppendSigned(write, end, config->offsetY);
    (void)Menu_AppendText(write, end, " K2 Run");

    Menu_RenderLines(titleLine, gainLine, deadbandLine, offsetLine);
}

/* 作用：渲染 Motor 子页，K1 切换固定 STEP 测试/真循迹，K2 启动。 */
static void Menu_RenderMotorMenu(void)
{
    Menu_RenderCenteredList(g_motorItems, MENU_MOTOR_COUNT, g_motorIndex);
}

/*
 * 作用：渲染灰度校准页面。
 * 说明：主菜单已隐藏灰度校准，但保留页面兼容状态机旧入口。
 */
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

/* 作用：渲染底盘无限循迹测试运行页。 */
static void Menu_RenderMotorTest(void)
{
    char speedLine[MENU_LINE_BUFFER_SIZE];
    char stepLine[MENU_LINE_BUFFER_SIZE];
    char *write = speedLine;
    char *end = &speedLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "Speed ");
    (void)Menu_AppendUnsigned(write, end, TrackStepTest_GetSpeedSps());

    write = stepLine;
    end = &stepLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Step ");
    write = Menu_AppendUnsigned(write, end, TrackStepTest_GetTravelSteps());
    write = Menu_AppendChar(write, end, '/');
    (void)Menu_AppendUnsigned(write, end,
        CAR_TRACK_STEP_TEST_TARGET_STEPS);

    Menu_RenderLines("Step Test", speedLine, stepLine,
        TrackStepTest_GetModeName());
}

/* 作用：渲染 Motor 真循迹运行页。 */
static void Menu_RenderMotorTrack(void)
{
    char phaseLine[MENU_LINE_BUFFER_SIZE];
    char stepLine[MENU_LINE_BUFFER_SIZE];
    char yawLine[MENU_LINE_BUFFER_SIZE];
    char *write = phaseLine;
    char *end = &phaseLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "P");
    write = Menu_AppendUnsigned(write, end, MotorTrack_GetPhase());
    write = Menu_AppendText(write, end, " ");
    (void)Menu_AppendText(write, end, MotorTrack_GetStateName());

    write = stepLine;
    end = &stepLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Step ");
    (void)Menu_AppendUnsigned(write, end, MotorTrack_GetTravelSteps());

    write = yawLine;
    end = &yawLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Yaw ");
    (void)Menu_AppendSigned(write, end, MotorTrack_GetYawDeltaDeg());

    Menu_RenderLines("Track Run", phaseLine, stepLine, yawLine);
}

/* 作用：渲染 Motor NO YAW 运行页。 */
static void Menu_RenderMotorNoYaw(void)
{
    char phaseLine[MENU_LINE_BUFFER_SIZE];
    char maskLine[MENU_LINE_BUFFER_SIZE];
    char errorLine[MENU_LINE_BUFFER_SIZE];
    char *write = phaseLine;
    char *end = &phaseLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "P");
    write = Menu_AppendUnsigned(write, end, MotorNoYaw_GetPhase());
    write = Menu_AppendText(write, end, " ");
    (void)Menu_AppendText(write, end, MotorNoYaw_GetStateName());

    write = maskLine;
    end = &maskLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Mask ");
    (void)Menu_AppendUnsigned(write, end, MotorNoYaw_GetDigitalMask());

    write = errorLine;
    end = &errorLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Err ");
    (void)Menu_AppendSigned(write, end, MotorNoYaw_GetLineError());

    Menu_RenderLines("NO YAW", phaseLine, maskLine, errorLine);
}

/* 作用：渲染 Motor 灰度 bitmask 测试页，只观察灭灯对应的 mask 值。 */
static void Menu_RenderMotorGrayTest(void)
{
    char maskLine[MENU_LINE_BUFFER_SIZE];
    char bitsLine[MENU_LINE_BUFFER_SIZE];
    char exitLine[MENU_LINE_BUFFER_SIZE];
    char *write = maskLine;
    char *end = &maskLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "Mask ");
    (void)Menu_AppendUnsigned(write, end, Gray_GetDigitalMask());

    write = bitsLine;
    end = &bitsLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "S1-7 ");
    (void)Menu_AppendGrayBits(write, end);

    write = exitLine;
    end = &exitLine[MENU_LINE_BUFFER_SIZE - 1U];
    (void)Menu_AppendText(write, end, "K2 Long Exit");

    Menu_RenderLines("Gray Test", maskLine, bitsLine, exitLine);
}

/* 作用：渲染云台电机 yaw/pitch SPS 测试运行页。 */
static void Menu_RenderGimbalMotorTest(void)
{
    char yawLine[MENU_LINE_BUFFER_SIZE];
    char pitchLine[MENU_LINE_BUFFER_SIZE];
    char *write = yawLine;
    char *end = &yawLine[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "Yaw ");
    (void)Menu_AppendUnsigned(write, end, GimbalMotorTest_GetYawSpeedSps());

    write = pitchLine;
    end = &pitchLine[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Pitch ");
    (void)Menu_AppendUnsigned(write, end,
        GimbalMotorTest_GetPitchSpeedSps());

    Menu_RenderLines("Gimbal", yawLine, pitchLine, "K2 Long Exit");
}

/* 作用：渲染视觉云台测试页，保留给旧入口调试。 */
static void Menu_RenderGimbalTest(void)
{
    Menu_RenderLines("Vision",
        (GimbalTest_HasVision() != 0U) ? "Tracking" : "Waiting Link",
        "K2 Long Exit", "");
}

/* 作用：渲染四电机 EN 使能测试页，保留给旧入口调试。 */
static void Menu_RenderMotorEnableTest(void)
{
    Menu_RenderLines("Enable", MotorEnableTest_GetEnableLine(),
        "No STEP", "K2 Long Exit");
}

/* 作用：渲染任务运行页，目前只显示任务编号。 */
static void Menu_RenderMission(void)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char *write = line1;
    char *end = &line1[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, "Mission ");
    (void)Menu_AppendUnsigned(write, end, StateMachine_GetMissionId());
    Menu_RenderLines("Mission", line1, "K2 Long Exit", "");
}

/* 作用：渲染只有标题和一行状态的简单页面。 */
static void Menu_RenderSimpleState(const char *title, const char *line1)
{
    Menu_RenderLines(title, line1, "K1 Menu", "K2 Back");
}

/*
 * 作用：根据整车状态选择具体页面。
 * 使用场景：Menu_Task 刷新 OLED 时调用。
 */
static void Menu_RenderByState(CarState state)
{
    switch (state) {
    case CAR_STATE_MENU:
        if (g_menuPage == MENU_PAGE_GIMBAL) {
            Menu_RenderGimbalMenu();
        } else if (g_menuPage == MENU_PAGE_MOTOR) {
            Menu_RenderMotorMenu();
        } else {
            Menu_RenderMainMenu();
        }
        break;
    case CAR_STATE_GRAY_CALIBRATION:
        Menu_RenderGrayCalibration();
        break;
    case CAR_STATE_TRACKING_TEST:
        Menu_RenderMotorTest();
        break;
    case CAR_STATE_MOTOR_TRACK:
        Menu_RenderMotorTrack();
        break;
    case CAR_STATE_MOTOR_NO_YAW:
        Menu_RenderMotorNoYaw();
        break;
    case CAR_STATE_MOTOR_GRAY_TEST:
        Menu_RenderMotorGrayTest();
        break;
    case CAR_STATE_GIMBAL_TEST:
        Menu_RenderGimbalTest();
        break;
    case CAR_STATE_GIMBAL_MOTOR_TEST:
        Menu_RenderGimbalMotorTest();
        break;
    case CAR_STATE_MOTOR_ENABLE_TEST:
        Menu_RenderMotorEnableTest();
        break;
    case CAR_STATE_MISSION:
        Menu_RenderMission();
        break;
    case CAR_STATE_TRACKING:
        Menu_RenderSimpleState("Tracking", "Running");
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

/*
 * 作用：初始化菜单内部选择状态。
 * 使用场景：App_Init 调用一次；状态机返回菜单时也会重置到第一项。
 */
void Menu_Init(void)
{
    g_menuPage = MENU_PAGE_MAIN;
    g_mainIndex = MENU_MAIN_GIMBAL;
    g_gimbalIndex = (uint8_t)StaticConfig_GetActiveTask();
    g_motorIndex = MENU_MOTOR_STEP_TEST;
    g_grayActionIndex = MENU_GRAY_SAVE_EXIT;
    g_forceRefresh = 1U;
    g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    g_lastState = CAR_STATE_INIT;
}

/* 作用：请求下一轮 Menu_Task 立即刷新 OLED。 */
void Menu_RequestRefresh(void)
{
    g_forceRefresh = 1U;
}

/*
 * 作用：切换主菜单选中项。
 * 使用场景：菜单态下 K1 按键触发。
 */
void Menu_Next(void)
{
    if (g_menuPage == MENU_PAGE_GIMBAL) {
        g_gimbalIndex = (uint8_t)((g_gimbalIndex + 1U) %
            (uint8_t)STATICCONFIG_TASK_COUNT);
        LOG_RAW("menu: gimbal select ");
        LOG_LINE(g_gimbalItems[g_gimbalIndex]);
    } else if (g_menuPage == MENU_PAGE_MOTOR) {
        g_motorIndex = (uint8_t)((g_motorIndex + 1U) %
            (uint8_t)MENU_MOTOR_COUNT);
        LOG_RAW("menu: motor select ");
        LOG_LINE(g_motorItems[g_motorIndex]);
    } else {
        g_mainIndex = (uint8_t)((g_mainIndex + 1U) % MENU_MAIN_COUNT);
        LOG_RAW("menu: select ");
        LOG_LINE(g_mainItems[g_mainIndex]);
    }
    Menu_RequestRefresh();
}

/*
 * 作用：确认当前主菜单选项并返回状态机事件。
 * 使用场景：菜单态下 K2 按键触发。
 */
CarEvent Menu_Confirm(void)
{
    if (g_menuPage == MENU_PAGE_GIMBAL) {
        StaticConfig_SetActiveTask((StaticConfigTaskId)g_gimbalIndex);
        return CAR_EVENT_GIMBAL_TEST_START;
    }
    if (g_menuPage == MENU_PAGE_MOTOR) {
        if (g_motorIndex == MENU_MOTOR_TRACK_RUN) {
            return CAR_EVENT_MOTOR_TRACK_START;
        }
        if (g_motorIndex == MENU_MOTOR_NO_YAW) {
            return CAR_EVENT_MOTOR_NO_YAW_START;
        }
        if (g_motorIndex == MENU_MOTOR_GRAY_TEST) {
            return CAR_EVENT_MOTOR_GRAY_TEST_START;
        }
        return CAR_EVENT_TRACKING_TEST_START;
    }

    switch (g_mainIndex) {
    case MENU_MAIN_GIMBAL:
        g_menuPage = MENU_PAGE_GIMBAL;
        g_gimbalIndex = (uint8_t)StaticConfig_GetActiveTask();
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case MENU_MAIN_MOTOR:
        g_menuPage = MENU_PAGE_MOTOR;
        g_motorIndex = MENU_MOTOR_STEP_TEST;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    case MENU_MAIN_MISSION:
        return CAR_EVENT_MISSION_1_START;
    default:
        return CAR_EVENT_NONE;
    }
}

/* 作用：从菜单子页返回主菜单，供菜单态 K2 长按使用。 */
uint8_t Menu_Back(void)
{
    if (g_menuPage == MENU_PAGE_MAIN) {
        return 0U;
    }

    g_menuPage = MENU_PAGE_MAIN;
    LOG_LINE("menu: back main");
    Menu_RequestRefresh();
    return 1U;
}

/*
 * 作用：灰度校准页切换退出动作。
 * 使用场景：灰度校准状态下 K1 按键触发。
 */
void Menu_GrayCalibrationNext(void)
{
    g_grayActionIndex = (uint8_t)((g_grayActionIndex + 1U) %
        MENU_GRAY_ACTION_COUNT);
    Menu_RequestRefresh();
}

/*
 * 作用：确认灰度校准页当前动作。
 * 使用场景：灰度校准状态下 K2 按键触发。
 * 说明：校准未完成时不允许 Save Exit，避免把无效阈值应用到运行参数。
 */
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

/*
 * 作用：按节流周期刷新 OLED 菜单/状态页。
 * 使用场景：App_Task 每轮调用。
 * 说明：本函数会访问 OLED，不允许放进中断里调用。
 */
void Menu_Task(CarState state)
{
    if (state != g_lastState) {
        if (state == CAR_STATE_MENU) {
            g_menuPage = MENU_PAGE_MAIN;
            g_mainIndex = MENU_MAIN_GIMBAL;
            g_gimbalIndex = (uint8_t)StaticConfig_GetActiveTask();
            g_motorIndex = MENU_MOTOR_STEP_TEST;
        }
        g_lastState = state;
        g_forceRefresh = 1U;
        g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    } else if (g_refreshTicks < CAR_MENU_REFRESH_TICKS) {
        ++g_refreshTicks;
    }

    if ((g_forceRefresh == 0U) &&
        (g_refreshTicks < CAR_MENU_REFRESH_TICKS)) {
        return;
    }

    g_forceRefresh = 0U;
    g_refreshTicks = 0U;
    Menu_RenderByState(state);
}
