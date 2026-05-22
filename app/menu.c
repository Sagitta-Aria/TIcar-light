#include "menu.h"

#include "board_config.h"
#include "encoder.h"
#include "gray.h"
#include "link.h"
#include "motor_test.h"
#include "oled.h"
#include "route.h"
#include "speed_control.h"
#include "tracking_exception.h"

#define MENU_OLED_FONT_SIZE        (12U)
#define MENU_OLED_MAX_CHARS        (21U)
#define MENU_MONO_START_X          (6U)
#define MENU_MONO_START_Y          (16U)
#define MENU_MONO_LINE_STEP        (12U)
#define MENU_MONO_MAX_CHARS        (18U)
#define MENU_LINE_BUFFER_SIZE      (32U)
#define MENU_LINK_BUFFER_SIZE      (128U)

typedef enum {
    MENU_PAGE_MAIN = 0,
    MENU_PAGE_TEST
} MenuPage;

typedef enum {
    MENU_MAIN_CALIB = 0,
    MENU_MAIN_TEST,
    MENU_MAIN_MISSION,
    MENU_MAIN_COUNT
} MenuMainItem;

typedef enum {
    MENU_TEST_MOTOR_DIR = 0,
    MENU_TEST_TRACK_ONLY,
    MENU_TEST_PID_DATA,
    MENU_TEST_GRAY_DATA,
    MENU_TEST_EXCHANGE_DATA,
    MENU_TEST_ENCODER_DATA,
    MENU_TEST_BACK,
    MENU_TEST_COUNT
} MenuTestItem;

static const char *const g_mainItems[MENU_MAIN_COUNT] = {
    "1.Calib",
    "2.Test",
    "3.Mission"
};

static const char *const g_testItems[MENU_TEST_COUNT] = {
    "Motor Dir",
    "Track Only",
    "PID Data",
    "Gray Data",
    "Exchange",
    "Encoder",
    "Back"
};

static MenuPage g_menuPage;
static uint8_t g_mainIndex;
static uint8_t g_testIndex;
static uint8_t g_forceRefresh;
static uint16_t g_refreshTicks;
static uint16_t g_linkPrintTicks;
static CarState g_lastState;

/*
 * 作用：向字符串缓冲区追加普通文本。
 * 使用场景：构造 OLED 行和串口调试行。
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
 * 使用场景：构造选择箭头、符号、分隔符。
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
 * 使用场景：OLED 和串口显示 ADC、编码器、PID 参数。
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
 * 使用场景：显示误差、电机输出、速度增量。
 */
static char *Menu_AppendSigned(char *write, char *end, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        write = Menu_AppendChar(write, end, '-');
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }

    return Menu_AppendUnsigned(write, end, magnitude);
}

/*
 * 作用：向字符串缓冲区追加 2 位十六进制数。
 * 使用场景：显示 7 路灰度数字量 mask。
 */
static char *Menu_AppendHexByte(char *write, char *end, uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    write = Menu_AppendChar(write, end, hex[(value >> 4U) & 0x0FU]);
    write = Menu_AppendChar(write, end, hex[value & 0x0FU]);
    return write;
}

/*
 * 作用：发送一行调试文本到现有串口输出。
 * 使用场景：监视页面周期打印数据。
 */
static void Menu_SendLine(const char *text)
{
    Link_SendString(text);
    Link_SendString("\r\n");
}

/*
 * 作用：把一行文本补齐到 21 个 ASCII 字符后写入 OLED。
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
 * 使用场景：双色 OLED 上方常为黄色区域，菜单页避开上方区域后显示更统一。
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
 * 作用：刷新 5 行 OLED 文本。
 * 使用场景：菜单页、校准页、测试监视页。
 */
static void Menu_RenderLines(const char *line0, const char *line1,
    const char *line2, const char *line3, const char *line4)
{
    Menu_ShowPaddedLine(0U, line0);
    Menu_ShowPaddedLine(1U, line1);
    Menu_ShowPaddedLine(2U, line2);
    Menu_ShowPaddedLine(3U, line3);
    Menu_ShowPaddedLine(4U, line4);
    OLED_Refresh();
}

/*
 * 作用：刷新 4 行菜单文本，并避开 OLED 顶部黄区。
 * 使用场景：主菜单和测试菜单这类长期停留页面。
 */
static void Menu_RenderMonoMenuLines(const char *line0, const char *line1,
    const char *line2, const char *line3)
{
    Menu_ShowPaddedLine(0U, "");
    Menu_ShowPaddedLine(1U, "");
    Menu_ShowPaddedLine(2U, "");
    Menu_ShowPaddedLine(3U, "");
    Menu_ShowPaddedLine(4U, "");

    Menu_ShowMonoLine(0U, line0);
    Menu_ShowMonoLine(1U, line1);
    Menu_ShowMonoLine(2U, line2);
    Menu_ShowMonoLine(3U, line3);
    OLED_Refresh();
}

/*
 * 作用：构造带选择箭头的菜单行。
 * 使用场景：主菜单和测试菜单。
 */
static void Menu_BuildItemLine(char line[MENU_LINE_BUFFER_SIZE],
    uint8_t selected, const char *text)
{
    char *write = line;
    char *end = &line[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendChar(write, end, selected ? '>' : ' ');
    (void)Menu_AppendText(write, end, text);
}

static void Menu_RenderMainMenu(void)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    char line3[MENU_LINE_BUFFER_SIZE];

    Menu_BuildItemLine(line1, (g_mainIndex == MENU_MAIN_CALIB),
        g_mainItems[MENU_MAIN_CALIB]);
    Menu_BuildItemLine(line2, (g_mainIndex == MENU_MAIN_TEST),
        g_mainItems[MENU_MAIN_TEST]);
    Menu_BuildItemLine(line3, (g_mainIndex == MENU_MAIN_MISSION),
        g_mainItems[MENU_MAIN_MISSION]);

    Menu_RenderMonoMenuLines(line1, line2, line3, "K1 OK K2 Next");
}

static void Menu_RenderTestMenu(void)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    char line3[MENU_LINE_BUFFER_SIZE];
    uint8_t index0 = g_testIndex;
    uint8_t index1 = (uint8_t)((g_testIndex + 1U) % MENU_TEST_COUNT);
    uint8_t index2 = (uint8_t)((g_testIndex + 2U) % MENU_TEST_COUNT);

    Menu_BuildItemLine(line1, 1U, g_testItems[index0]);
    Menu_BuildItemLine(line2, 0U, g_testItems[index1]);
    Menu_BuildItemLine(line3, 0U, g_testItems[index2]);
    Menu_RenderMonoMenuLines(line1, line2, line3, "K1 OK K2 Next");
}

static void Menu_BuildPairLine(char line[MENU_LINE_BUFFER_SIZE],
    const char *title, int32_t left, int32_t right)
{
    char *write = line;
    char *end = &line[MENU_LINE_BUFFER_SIZE - 1U];

    write = Menu_AppendText(write, end, title);
    write = Menu_AppendText(write, end, " L");
    write = Menu_AppendSigned(write, end, left);
    write = Menu_AppendText(write, end, " R");
    (void)Menu_AppendSigned(write, end, right);
}

static void Menu_RenderGrayCalibration(void)
{
    Menu_RenderLines("Gray Calib", "Move black/white", "Keep sampling",
        "K1 Apply", "K2 Back");
}

static void Menu_RenderTracking(uint8_t routeEnabled)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    char line3[MENU_LINE_BUFFER_SIZE];

    if (routeEnabled) {
        (void)Menu_AppendText(line1, &line1[MENU_LINE_BUFFER_SIZE - 1U],
            Route_GetStageName(Route_GetStage()));
    } else {
        (void)Menu_AppendText(line1, &line1[MENU_LINE_BUFFER_SIZE - 1U],
            "Route off");
    }

    (void)Menu_AppendText(line2, &line2[MENU_LINE_BUFFER_SIZE - 1U],
        TrackingException_GetStateName(TrackingException_GetState()));
    Menu_BuildPairLine(line3, "Spd", SpeedControl_GetLeftActual(),
        SpeedControl_GetRightActual());

    Menu_RenderLines(routeEnabled ? "Tracking" : "Track Only", line1,
        line2, line3, "K2 Back");
}

static void Menu_RenderMotorTest(void)
{
    Menu_RenderLines("Motor Test", MotorTest_GetStepName(),
        "K1 Next Step", "K2 Back", "Lift car first");
}

static void Menu_RenderPidMonitor(uint8_t printNow)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    char line3[MENU_LINE_BUFFER_SIZE];
    char log[MENU_LINK_BUFFER_SIZE];
    char *write;
    char *end;

    write = line1;
    end = &line1[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Kp");
    write = Menu_AppendSigned(write, end, CAR_SPEED_KP);
    write = Menu_AppendText(write, end, " Ki");
    (void)Menu_AppendSigned(write, end, CAR_SPEED_KI);

    Menu_BuildPairLine(line2, "Act", SpeedControl_GetLeftActual(),
        SpeedControl_GetRightActual());
    Menu_BuildPairLine(line3, "Out", SpeedControl_GetLeftOutput(),
        SpeedControl_GetRightOutput());
    Menu_RenderLines("PID Data", line1, line2, line3, "K2 Back");

    if (printNow) {
        write = log;
        end = &log[MENU_LINK_BUFFER_SIZE - 1U];
        write = Menu_AppendText(write, end, "PID kp=");
        write = Menu_AppendSigned(write, end, CAR_SPEED_KP);
        write = Menu_AppendText(write, end, " ki=");
        write = Menu_AppendSigned(write, end, CAR_SPEED_KI);
        write = Menu_AppendText(write, end, " actL=");
        write = Menu_AppendSigned(write, end, SpeedControl_GetLeftActual());
        write = Menu_AppendText(write, end, " actR=");
        write = Menu_AppendSigned(write, end, SpeedControl_GetRightActual());
        write = Menu_AppendText(write, end, " outL=");
        write = Menu_AppendSigned(write, end, SpeedControl_GetLeftOutput());
        write = Menu_AppendText(write, end, " outR=");
        (void)Menu_AppendSigned(write, end, SpeedControl_GetRightOutput());
        Menu_SendLine(log);
    }
}

static void Menu_RenderGrayMonitor(uint8_t printNow)
{
    uint16_t raw[GRAY_SENSOR_COUNT];
    int16_t error = 0;
    uint8_t mask;
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    char line3[MENU_LINE_BUFFER_SIZE];
    char log[MENU_LINK_BUFFER_SIZE];
    char *write;
    char *end;
    uint8_t i;

    Gray_ReadRaw(raw);
    mask = Gray_GetDigitalMask();
    (void)Gray_GetWeightedLineError(&error);

    write = line1;
    end = &line1[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Mask 0x");
    (void)Menu_AppendHexByte(write, end, mask);

    write = line2;
    end = &line2[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Err ");
    (void)Menu_AppendSigned(write, end, error);

    write = line3;
    end = &line3[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "R1 ");
    write = Menu_AppendUnsigned(write, end, raw[0]);
    write = Menu_AppendText(write, end, " R2 ");
    (void)Menu_AppendUnsigned(write, end, raw[1]);

    Menu_RenderLines("Gray Data", line1, line2, line3, "K2 Back");

    if (printNow) {
        write = log;
        end = &log[MENU_LINK_BUFFER_SIZE - 1U];
        write = Menu_AppendText(write, end, "GRAY mask=0x");
        write = Menu_AppendHexByte(write, end, mask);
        write = Menu_AppendText(write, end, " err=");
        write = Menu_AppendSigned(write, end, error);
        write = Menu_AppendText(write, end, " raw=");
        for (i = 0U; i < GRAY_SENSOR_COUNT; ++i) {
            write = Menu_AppendUnsigned(write, end, raw[i]);
            if (i + 1U < GRAY_SENSOR_COUNT) {
                write = Menu_AppendChar(write, end, ',');
            }
        }
        Menu_SendLine(log);
    }
}

static void Menu_RenderExchangeMonitor(uint8_t printNow)
{
    Menu_RenderLines("Exchange Data", "Vision UART", "Parser not ready",
        "No change now", "K2 Back");

    if (printNow) {
        Menu_SendLine("EXCHANGE monitor placeholder: vision parser not ready");
    }
}

static void Menu_RenderEncoderMonitor(uint8_t printNow)
{
    char line1[MENU_LINE_BUFFER_SIZE];
    char line2[MENU_LINE_BUFFER_SIZE];
    char line3[MENU_LINE_BUFFER_SIZE];
    char log[MENU_LINK_BUFFER_SIZE];
    char *write;
    char *end;

    write = line1;
    end = &line1[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Left ");
    (void)Menu_AppendSigned(write, end, Encoder_GetLeft());

    write = line2;
    end = &line2[MENU_LINE_BUFFER_SIZE - 1U];
    write = Menu_AppendText(write, end, "Right ");
    (void)Menu_AppendSigned(write, end, Encoder_GetRight());

    Menu_BuildPairLine(line3, "Act", SpeedControl_GetLeftActual(),
        SpeedControl_GetRightActual());
    Menu_RenderLines("Encoder Data", line1, line2, line3, "K2 Back");

    if (printNow) {
        write = log;
        end = &log[MENU_LINK_BUFFER_SIZE - 1U];
        write = Menu_AppendText(write, end, "ENC left=");
        write = Menu_AppendSigned(write, end, Encoder_GetLeft());
        write = Menu_AppendText(write, end, " right=");
        write = Menu_AppendSigned(write, end, Encoder_GetRight());
        write = Menu_AppendText(write, end, " actL=");
        write = Menu_AppendSigned(write, end, SpeedControl_GetLeftActual());
        write = Menu_AppendText(write, end, " actR=");
        (void)Menu_AppendSigned(write, end, SpeedControl_GetRightActual());
        Menu_SendLine(log);
    }
}

static void Menu_RenderMission(void)
{
    Menu_RenderLines("Mission", "Task placeholder", "Tell me later",
        "Framework ready", "K2 Back");
}

static void Menu_RenderSimpleState(const char *title, const char *line1)
{
    Menu_RenderLines(title, line1, "K1 Menu", "K2 Back", "");
}

static void Menu_RenderByState(CarState state, uint8_t printNow)
{
    switch (state) {
    case CAR_STATE_MENU:
        if (g_menuPage == MENU_PAGE_TEST) {
            Menu_RenderTestMenu();
        } else {
            Menu_RenderMainMenu();
        }
        break;
    case CAR_STATE_GRAY_CALIBRATION:
        Menu_RenderGrayCalibration();
        break;
    case CAR_STATE_TRACKING:
        Menu_RenderTracking(1U);
        break;
    case CAR_STATE_TRACKING_TEST:
        Menu_RenderTracking(0U);
        break;
    case CAR_STATE_MOTOR_TEST:
        Menu_RenderMotorTest();
        break;
    case CAR_STATE_PID_MONITOR:
        Menu_RenderPidMonitor(printNow);
        break;
    case CAR_STATE_GRAY_MONITOR:
        Menu_RenderGrayMonitor(printNow);
        break;
    case CAR_STATE_EXCHANGE_MONITOR:
        Menu_RenderExchangeMonitor(printNow);
        break;
    case CAR_STATE_ENCODER_MONITOR:
        Menu_RenderEncoderMonitor(printNow);
        break;
    case CAR_STATE_MISSION:
        Menu_RenderMission();
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
    g_mainIndex = MENU_MAIN_CALIB;
    g_testIndex = MENU_TEST_MOTOR_DIR;
    g_forceRefresh = 1U;
    g_refreshTicks = CAR_MENU_REFRESH_TICKS;
    g_linkPrintTicks = CAR_MENU_LINK_PRINT_TICKS;
    g_lastState = CAR_STATE_INIT;
}

void Menu_Next(void)
{
    if (g_menuPage == MENU_PAGE_MAIN) {
        g_mainIndex = (uint8_t)((g_mainIndex + 1U) % MENU_MAIN_COUNT);
    } else {
        g_testIndex = (uint8_t)((g_testIndex + 1U) % MENU_TEST_COUNT);
    }
    g_forceRefresh = 1U;
}

CarEvent Menu_Confirm(void)
{
    if (g_menuPage == MENU_PAGE_MAIN) {
        switch (g_mainIndex) {
        case MENU_MAIN_CALIB:
            return CAR_EVENT_GRAY_CALIBRATION_START;
        case MENU_MAIN_TEST:
            g_menuPage = MENU_PAGE_TEST;
            g_testIndex = MENU_TEST_MOTOR_DIR;
            g_forceRefresh = 1U;
            return CAR_EVENT_NONE;
        case MENU_MAIN_MISSION:
            return CAR_EVENT_MISSION_START;
        default:
            return CAR_EVENT_NONE;
        }
    }

    switch (g_testIndex) {
    case MENU_TEST_MOTOR_DIR:
        return CAR_EVENT_MOTOR_TEST_NEXT;
    case MENU_TEST_TRACK_ONLY:
        return CAR_EVENT_TRACKING_TEST_START;
    case MENU_TEST_PID_DATA:
        return CAR_EVENT_PID_MONITOR_START;
    case MENU_TEST_GRAY_DATA:
        return CAR_EVENT_GRAY_MONITOR_START;
    case MENU_TEST_EXCHANGE_DATA:
        return CAR_EVENT_EXCHANGE_MONITOR_START;
    case MENU_TEST_ENCODER_DATA:
        return CAR_EVENT_ENCODER_MONITOR_START;
    case MENU_TEST_BACK:
        g_menuPage = MENU_PAGE_MAIN;
        g_mainIndex = MENU_MAIN_TEST;
        g_forceRefresh = 1U;
        return CAR_EVENT_NONE;
    default:
        return CAR_EVENT_NONE;
    }
}

void Menu_Task(CarState state)
{
    uint8_t printNow;

    if (state != g_lastState) {
        if (state == CAR_STATE_MENU) {
            g_menuPage = MENU_PAGE_MAIN;
            g_mainIndex = MENU_MAIN_CALIB;
        }
        g_lastState = state;
        g_forceRefresh = 1U;
        g_refreshTicks = CAR_MENU_REFRESH_TICKS;
        g_linkPrintTicks = CAR_MENU_LINK_PRINT_TICKS;
    } else {
        if (g_refreshTicks < CAR_MENU_REFRESH_TICKS) {
            ++g_refreshTicks;
        }
        if (g_linkPrintTicks < CAR_MENU_LINK_PRINT_TICKS) {
            ++g_linkPrintTicks;
        }
    }

    if (!g_forceRefresh && (g_refreshTicks < CAR_MENU_REFRESH_TICKS)) {
        return;
    }

    printNow = (g_linkPrintTicks >= CAR_MENU_LINK_PRINT_TICKS) ? 1U : 0U;
    g_forceRefresh = 0U;
    g_refreshTicks = 0U;
    if (printNow) {
        g_linkPrintTicks = 0U;
    }

    Menu_RenderByState(state, printNow);
}
