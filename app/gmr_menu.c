#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "menu.h"

#include "board.h"
#include "car_display.h"
#include "control_config.h"
#include "encoder_motor.h"
#include "gray.h"
#include "m0_attitude_link.h"
#include "task_registry.h"

#define GMR_MENU_LINE_SIZE  (24U)
#define GMR_MENU_ROW_COUNT  (5U)

typedef enum {
    GMR_MENU_PAGE_MAIN = 0,
    GMR_MENU_PAGE_TASK4_LEFT,
    GMR_MENU_PAGE_TASK4_RIGHT
} GmrMenuPage;

static uint8_t g_forceRefresh;
static CarState g_lastState;
static uint8_t g_selectedTaskIndex;
static GmrMenuPage g_menuPage;
static uint16_t g_task4LeftTarget;
static uint16_t g_task4RightTarget;

static char *GmrMenu_AppendText(char *write, char *end, const char *text)
{
    while ((text != 0) && (*text != '\0') && (write < end)) {
        *write++ = *text++;
    }
    *write = '\0';
    return write;
}

static char *GmrMenu_AppendChar(char *write, char *end, char value)
{
    if (write < end) {
        *write++ = value;
    }
    *write = '\0';
    return write;
}

static char *GmrMenu_AppendUnsigned(char *write, char *end, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < (uint8_t)sizeof(digits)));
    while ((count > 0U) && (write < end)) {
        *write++ = digits[--count];
    }
    *write = '\0';
    return write;
}

static char *GmrMenu_AppendSigned(char *write, char *end, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        write = GmrMenu_AppendChar(write, end, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return GmrMenu_AppendUnsigned(write, end, magnitude);
}

static char *GmrMenu_AppendFixedX100(char *write, char *end, int32_t value)
{
    uint32_t magnitude;
    uint32_t fraction;

    if (value < 0) {
        write = GmrMenu_AppendChar(write, end, '-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    fraction = magnitude % 100U;
    write = GmrMenu_AppendUnsigned(write, end, magnitude / 100U);
    write = GmrMenu_AppendChar(write, end, '.');
    if (fraction < 10U) {
        write = GmrMenu_AppendChar(write, end, '0');
    }
    return GmrMenu_AppendUnsigned(write, end, fraction);
}

static void GmrMenu_BuildFixedLine(char line[GMR_MENU_LINE_SIZE],
    const char *label, int32_t value, const char *unit)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, label);
    write = GmrMenu_AppendFixedX100(write, end, value);
    (void)GmrMenu_AppendText(write, end, unit);
}

static void GmrMenu_BuildUnsignedLine(char line[GMR_MENU_LINE_SIZE],
    const char *label, uint32_t value)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, label);
    (void)GmrMenu_AppendUnsigned(write, end, value);
}

static void GmrMenu_BuildSignedLine(char line[GMR_MENU_LINE_SIZE],
    const char *label, int32_t value)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, label);
    (void)GmrMenu_AppendSigned(write, end, value);
}

static void GmrMenu_BuildTaskLine(char line[GMR_MENU_LINE_SIZE],
    uint8_t taskIndex)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendChar(write, end,
        (taskIndex == g_selectedTaskIndex) ? '>' : ' ');
    (void)GmrMenu_AppendText(write, end, TaskRegistry_GetName(taskIndex));
}

static void GmrMenu_BuildSignedPair(char line[GMR_MENU_LINE_SIZE],
    const char *label, int32_t left, int32_t right)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendText(write, end, label);
    write = GmrMenu_AppendSigned(write, end, left);
    write = GmrMenu_AppendChar(write, end, '/');
    (void)GmrMenu_AppendSigned(write, end, right);
}

static void GmrMenu_BuildGrayMaskLine(char line[GMR_MENU_LINE_SIZE],
    uint8_t grayMask)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];
    uint8_t index;

    write = GmrMenu_AppendText(write, end, "Gray ");
    for (index = 0U; index < GRAY_SENSOR_COUNT; ++index) {
        write = GmrMenu_AppendChar(write, end,
            ((grayMask & (uint8_t)(1U <<
                ((GRAY_SENSOR_COUNT - 1U) - index))) != 0U) ? '1' : '0');
    }
}

static uint16_t GmrMenu_NextTask4Target(uint16_t target)
{
    if (target >= GMR_MISSION4_SPEED_MAX_COUNTS_PER_PERIOD) {
        return GMR_MISSION4_SPEED_MIN_COUNTS_PER_PERIOD;
    }
    return (uint16_t)(target +
        GMR_MISSION4_SPEED_STEP_COUNTS_PER_PERIOD);
}

static uint16_t GmrMenu_PreviousTask4Target(uint16_t target)
{
    if (target <= GMR_MISSION4_SPEED_MIN_COUNTS_PER_PERIOD) {
        return GMR_MISSION4_SPEED_MAX_COUNTS_PER_PERIOD;
    }
    return (uint16_t)(target -
        GMR_MISSION4_SPEED_STEP_COUNTS_PER_PERIOD);
}

static void GmrMenu_ShowPage(const char *line0, const char *line1,
    const char *line2, const char *line3, const char *line4)
{
    static const char *const empty = "";
    const char *lines[GMR_MENU_ROW_COUNT];
    uint8_t row;

    if (Board_IsDisplayAvailable() == 0U) {
        return;
    }
    lines[0] = (line0 != 0) ? line0 : empty;
    lines[1] = (line1 != 0) ? line1 : empty;
    lines[2] = (line2 != 0) ? line2 : empty;
    lines[3] = (line3 != 0) ? line3 : empty;
    lines[4] = (line4 != 0) ? line4 : empty;
    for (row = 0U; row < GMR_MENU_ROW_COUNT; ++row) {
        CarDisplay_ShowLine(row, lines[row]);
    }
    for (row = GMR_MENU_ROW_COUNT; row < 10U; ++row) {
        CarDisplay_ShowLine(row, empty);
    }
    CarDisplay_Refresh();
}

static void GmrMenu_RenderAttitude(void)
{
    M0AttitudeLinkSnapshot snapshot;
    char yawLine[GMR_MENU_LINE_SIZE];
    char rateLine[GMR_MENU_LINE_SIZE];
    char sequenceLine[GMR_MENU_LINE_SIZE];
    char frameLine[GMR_MENU_LINE_SIZE];
    char *write;
    char *end;

    if (M0AttitudeLink_GetSnapshot(&snapshot) == 0U) {
        GmrMenu_ShowPage("Attitude Monitor", "Waiting M0 data",
            "UART3 PB2/PB3", "115200 8-N-1", "K3 Exit");
        return;
    }
    GmrMenu_BuildFixedLine(yawLine, "Yaw ", snapshot.yawUnwrappedX100,
        " deg");
    GmrMenu_BuildFixedLine(rateLine, "Rate ",
        snapshot.yawRateX100PerSec, " d/s");
    GmrMenu_BuildUnsignedLine(sequenceLine, "Seq ", snapshot.sequence);
    write = frameLine;
    end = &frameLine[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendText(write, end, "F/B ");
    write = GmrMenu_AppendUnsigned(write, end, snapshot.frameCount);
    write = GmrMenu_AppendChar(write, end, '/');
    (void)GmrMenu_AppendUnsigned(write, end, snapshot.badFrameCount);
    GmrMenu_ShowPage("Attitude Monitor", yawLine, rateLine,
        sequenceLine, frameLine);
}

static void GmrMenu_RenderEncoder(void)
{
    int32_t leftCount;
    int32_t rightCount;
    char leftLine[GMR_MENU_LINE_SIZE];
    char rightLine[GMR_MENU_LINE_SIZE];

    EncoderMotor_GetTotalCounts(&leftCount, &rightCount);
    GmrMenu_BuildSignedLine(leftLine, "Left ", leftCount);
    GmrMenu_BuildSignedLine(rightLine, "Right ", rightCount);
    GmrMenu_ShowPage("Encoder Monitor", leftLine, rightLine,
        "Counts reset at start", "K3 Exit");
}

static void GmrMenu_RenderTask4Config(void)
{
    char leftLine[GMR_MENU_LINE_SIZE];
    char rightLine[GMR_MENU_LINE_SIZE];
    char *write;
    char *end;

    write = leftLine;
    end = &leftLine[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendChar(write, end,
        (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) ? '>' : ' ');
    write = GmrMenu_AppendText(write, end, "Left ");
    (void)GmrMenu_AppendUnsigned(write, end, g_task4LeftTarget);

    write = rightLine;
    end = &rightLine[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendChar(write, end,
        (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) ? '>' : ' ');
    write = GmrMenu_AppendText(write, end, "Right ");
    (void)GmrMenu_AppendUnsigned(write, end, g_task4RightTarget);

    GmrMenu_ShowPage("Task 4 Targets", leftLine, rightLine,
        "K1 +1  K4 -1",
        (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) ?
            "K2 Right K3 Back" : "K2 Start K3 Back");
}

static void GmrMenu_RenderDrive(void)
{
    EncoderMotorSnapshot snapshot;
    char targetLine[GMR_MENU_LINE_SIZE];
    char feedbackLine[GMR_MENU_LINE_SIZE];
    char pwmLine[GMR_MENU_LINE_SIZE];

    EncoderMotor_GetSnapshot(&snapshot);
    GmrMenu_BuildSignedPair(targetLine, "Target ",
        snapshot.targetCounts[ENCODER_MOTOR_LEFT],
        snapshot.targetCounts[ENCODER_MOTOR_RIGHT]);
    GmrMenu_BuildSignedPair(feedbackLine, "Actual ",
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT],
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT]);
    GmrMenu_BuildSignedPair(pwmLine, "PWM ",
        snapshot.outputPwm[ENCODER_MOTOR_LEFT],
        snapshot.outputPwm[ENCODER_MOTOR_RIGHT]);
    GmrMenu_ShowPage("Drive Closed Loop", targetLine, feedbackLine, pwmLine,
        "K3 Exit");
}

static void GmrMenu_RenderDirection(void)
{
    EncoderMotorSnapshot snapshot;
    CarChassisDriveMode mode = StateMachine_GetMissionDriveMode(5U);
    uint16_t speed = StateMachine_GetMissionDriveSpeed(5U);
    char modeLine[GMR_MENU_LINE_SIZE];
    char feedbackLine[GMR_MENU_LINE_SIZE];
    char pwmLine[GMR_MENU_LINE_SIZE];
    char *write = modeLine;
    char *end = &modeLine[GMR_MENU_LINE_SIZE - 1U];

    EncoderMotor_GetSnapshot(&snapshot);
    if (mode == CAR_CHASSIS_DRIVE_OPEN_LOOP) {
        write = GmrMenu_AppendText(write, end, "PWM +20% ");
    } else {
        write = GmrMenu_AppendText(write, end, "Closed +20 ");
    }
    (void)GmrMenu_AppendText(write, end, (speed == 0U) ? "STOP" : "RUN");
    GmrMenu_BuildSignedPair(feedbackLine, "Actual ",
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT],
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT]);
    GmrMenu_BuildSignedPair(pwmLine, "PWM ",
        snapshot.outputPwm[ENCODER_MOTOR_LEFT],
        snapshot.outputPwm[ENCODER_MOTOR_RIGHT]);
    GmrMenu_ShowPage("Direction +20", modeLine, feedbackLine, pwmLine,
        "K1 Mode K2 Run K3 X");
}

static void GmrMenu_RenderGrayDifferential(const char *title)
{
    EncoderMotorSnapshot snapshot;
    char grayLine[GMR_MENU_LINE_SIZE];
    char targetLine[GMR_MENU_LINE_SIZE];
    char feedbackLine[GMR_MENU_LINE_SIZE];
    char pwmLine[GMR_MENU_LINE_SIZE];

    EncoderMotor_GetSnapshot(&snapshot);
    GmrMenu_BuildGrayMaskLine(grayLine, Gray_GetDigitalMask());
    GmrMenu_BuildSignedPair(targetLine, "Target ",
        snapshot.targetCounts[ENCODER_MOTOR_LEFT],
        snapshot.targetCounts[ENCODER_MOTOR_RIGHT]);
    GmrMenu_BuildSignedPair(feedbackLine, "Actual ",
        snapshot.feedbackCounts[ENCODER_MOTOR_LEFT],
        snapshot.feedbackCounts[ENCODER_MOTOR_RIGHT]);
    GmrMenu_BuildSignedPair(pwmLine, "PWM ",
        snapshot.outputPwm[ENCODER_MOTOR_LEFT],
        snapshot.outputPwm[ENCODER_MOTOR_RIGHT]);
    GmrMenu_ShowPage(title, grayLine, targetLine,
        feedbackLine, pwmLine);
}

static void GmrMenu_RenderMission2Timer(void)
{
    CarDisplay_ShowTimer(StateMachine_GetMission2ElapsedMs());
    CarDisplay_Refresh();
}

static void GmrMenu_Render(CarState state)
{
    char taskLines[4][GMR_MENU_LINE_SIZE];
    uint8_t firstTaskIndex;
    uint8_t row;
    uint8_t taskCount;

    if (state == CAR_STATE_MENU) {
        if (g_menuPage != GMR_MENU_PAGE_MAIN) {
            GmrMenu_RenderTask4Config();
            return;
        }
        taskCount = TaskRegistry_GetCount();
        firstTaskIndex = (g_selectedTaskIndex < 4U) ? 0U :
            (uint8_t)(g_selectedTaskIndex - 3U);
        for (row = 0U; row < 4U; ++row) {
            if ((uint8_t)(firstTaskIndex + row) < taskCount) {
                GmrMenu_BuildTaskLine(taskLines[row],
                    (uint8_t)(firstTaskIndex + row));
            } else {
                taskLines[row][0] = '\0';
            }
        }
        GmrMenu_ShowPage("GMR K1/K4 K2 Go", taskLines[0], taskLines[1],
            taskLines[2], taskLines[3]);
    } else if (state == CAR_STATE_MISSION) {
        if (StateMachine_GetMissionId() == 7U) {
            GmrMenu_RenderGrayDifferential("Line Follow");
        } else if (StateMachine_GetMissionId() == 6U) {
            GmrMenu_RenderGrayDifferential("Gray Differential");
        } else if (StateMachine_GetMissionId() == 5U) {
            GmrMenu_RenderDirection();
        } else if (StateMachine_GetMissionId() == 4U) {
            GmrMenu_RenderDrive();
        } else if (StateMachine_GetMissionId() == 3U) {
            GmrMenu_RenderEncoder();
        } else if (StateMachine_GetMissionId() == 2U) {
            GmrMenu_RenderMission2Timer();
        } else {
            GmrMenu_RenderAttitude();
        }
    } else if (state == CAR_STATE_STOP) {
        GmrMenu_ShowPage("Stopped", "K3 Back", "H7 START=1..7", "", "");
    } else if (state == CAR_STATE_ERROR) {
        GmrMenu_ShowPage("Error", "K3 Back", "", "", "");
    } else if (state == CAR_STATE_FINISHED) {
        if (StateMachine_GetMissionId() == 2U) {
            GmrMenu_RenderMission2Timer();
        } else {
            GmrMenu_ShowPage("Finished", "K3 Back", "", "", "");
        }
    } else {
        GmrMenu_ShowPage("Init", "", "", "", "");
    }
}

void Menu_Init(void)
{
    g_forceRefresh = 1U;
    g_lastState = CAR_STATE_INIT;
    g_selectedTaskIndex = 0U;
    g_menuPage = GMR_MENU_PAGE_MAIN;
    g_task4LeftTarget = GMR_MISSION4_LEFT_SPEED_COUNTS_PER_PERIOD;
    g_task4RightTarget = GMR_MISSION4_RIGHT_SPEED_COUNTS_PER_PERIOD;
}

void Menu_Next(void)
{
    uint8_t count;

    if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_task4LeftTarget = GmrMenu_NextTask4Target(g_task4LeftTarget);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        g_task4RightTarget = GmrMenu_NextTask4Target(g_task4RightTarget);
    } else {
        count = TaskRegistry_GetCount();
        if (count > 0U) {
            g_selectedTaskIndex =
                (uint8_t)((g_selectedTaskIndex + 1U) % count);
        }
    }
    Menu_RequestRefresh();
}

void Menu_Previous(void)
{
    uint8_t count;

    if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_task4LeftTarget = GmrMenu_PreviousTask4Target(g_task4LeftTarget);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        g_task4RightTarget = GmrMenu_PreviousTask4Target(g_task4RightTarget);
    } else {
        count = TaskRegistry_GetCount();
        if (count > 0U) {
            g_selectedTaskIndex = (g_selectedTaskIndex == 0U) ?
                (uint8_t)(count - 1U) :
                (uint8_t)(g_selectedTaskIndex - 1U);
        }
    }
    Menu_RequestRefresh();
}

CarEvent Menu_Confirm(void)
{
    if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_menuPage = GMR_MENU_PAGE_TASK4_RIGHT;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        StateMachine_SetMission4Targets(g_task4LeftTarget,
            g_task4RightTarget);
        g_menuPage = GMR_MENU_PAGE_MAIN;
        return CAR_EVENT_MISSION_4_START;
    }
    if (TaskRegistry_GetMissionId(g_selectedTaskIndex) == 4U) {
        g_menuPage = GMR_MENU_PAGE_TASK4_LEFT;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    return TaskRegistry_GetStartEvent(g_selectedTaskIndex);
}

uint8_t Menu_Back(void)
{
    if (g_menuPage == GMR_MENU_PAGE_MAIN) {
        return 0U;
    }
    g_menuPage = (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) ?
        GMR_MENU_PAGE_TASK4_LEFT : GMR_MENU_PAGE_MAIN;
    Menu_RequestRefresh();
    return 1U;
}

void Menu_RequestRefresh(void)
{
    g_forceRefresh = 1U;
}

void Menu_Task(CarState state)
{
    if (state != g_lastState) {
        if (state == CAR_STATE_MENU) {
            g_menuPage = GMR_MENU_PAGE_MAIN;
        }
        g_lastState = state;
        g_forceRefresh = 1U;
    }
    if ((g_forceRefresh == 0U) && (state != CAR_STATE_MISSION)) {
        return;
    }
    g_forceRefresh = 0U;
    GmrMenu_Render(state);
    if (Board_IsDisplayAvailable() == 0U) {
        g_forceRefresh = 1U;
    }
}

#endif
