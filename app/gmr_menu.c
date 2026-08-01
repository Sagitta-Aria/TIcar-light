#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "menu.h"

#include "board.h"
#include "car_display.h"
#include "control_config.h"
#include "encoder_motor.h"
#include "gray.h"
#include "gmr_stop_count_store.h"
#include "m0_attitude_link.h"
#include "task_registry.h"

#define GMR_MENU_LINE_SIZE  (24U)
#define GMR_MENU_ROW_COUNT  (5U)
#define GMR_MENU_MAIN_ITEM_COUNT  (2U)
#define GMR_MENU_TASK_ITEM_COUNT  (6U)
#define GMR_MENU_OTHER_TASK_ITEM_COUNT (9U)
#define GMR_MENU_OTHER_STOP_COUNT_INDEX (9U)
#define GMR_MENU_OTHER_ITEM_COUNT (10U)

typedef enum {
    GMR_MENU_PAGE_MAIN = 0,
    GMR_MENU_PAGE_TASK,
    GMR_MENU_PAGE_OTHER,
    GMR_MENU_PAGE_TASK4_LEFT,
    GMR_MENU_PAGE_TASK4_RIGHT,
    GMR_MENU_PAGE_STOP_COUNT
} GmrMenuPage;

typedef enum {
    GMR_STOP_COUNT_EDITING = 0,
    GMR_STOP_COUNT_SAVE_OK,
    GMR_STOP_COUNT_SAVE_FAILED
} GmrStopCountSaveResult;

static const uint8_t g_taskMenuTaskIndices[GMR_MENU_TASK_ITEM_COUNT] = {
    1U, 7U, 9U, 10U, 11U, 12U
};
static const char *const g_taskMenuLabels[GMR_MENU_TASK_ITEM_COUNT] = {
    "1 Task2 Track A-A",
    "2 BMI Ball Debug",
    "3 Task3 Ball",
    "4 Task4 Track+Ball",
    "5 Task5 Track+Ball",
    "6 Task6 Track+Ball"
};
static const uint8_t g_otherMenuTaskIndices[
    GMR_MENU_OTHER_TASK_ITEM_COUNT] = {
    0U, 2U, 3U, 4U, 5U, 6U, 8U, 13U, 14U
};
static const char *const g_stopCountLabels[GMR_STOP_COUNT_ITEM_COUNT] = {
    "Task1 Left ",
    "Task1 Right ",
    "Task4 Left ",
    "Task4 Right ",
    "Task5 Left ",
    "Task5 Right "
};

static uint8_t g_forceRefresh;
static CarState g_lastState;
static uint8_t g_mainSelection;
static uint8_t g_taskSelection;
static uint8_t g_otherSelection;
static GmrMenuPage g_menuPage;
static uint16_t g_task4LeftTarget;
static uint16_t g_task4RightTarget;
static GmrStopCountValues g_stopCountDraft;
static uint8_t g_stopCountSelection;
static GmrStopCountSaveResult g_stopCountSaveResult;

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

static void GmrMenu_BuildSelectionLine(char line[GMR_MENU_LINE_SIZE],
    uint8_t selected, const char *label)
{
    char *write = line;
    char *end = &line[GMR_MENU_LINE_SIZE - 1U];

    write = GmrMenu_AppendChar(write, end,
        (selected != 0U) ? '>' : ' ');
    (void)GmrMenu_AppendText(write, end, label);
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

    write = GmrMenu_AppendText(write, end, "IR ");
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

static uint32_t GmrMenu_IncreaseStopCount(uint32_t value)
{
    if (value >= (GMR_STOP_COUNT_MAX_VALUE -
        GMR_STOP_COUNT_MENU_STEP)) {
        return GMR_STOP_COUNT_MAX_VALUE;
    }
    return value + GMR_STOP_COUNT_MENU_STEP;
}

static uint32_t GmrMenu_DecreaseStopCount(uint32_t value)
{
    if (value <= (GMR_STOP_COUNT_MIN_VALUE +
        GMR_STOP_COUNT_MENU_STEP)) {
        return GMR_STOP_COUNT_MIN_VALUE;
    }
    return value - GMR_STOP_COUNT_MENU_STEP;
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

static void GmrMenu_RenderTaskTimer(void)
{
    CarDisplay_ShowTimer(StateMachine_GetTaskElapsedMs());
    CarDisplay_Refresh();
}

static void GmrMenu_RenderH7StepperTest(void)
{
    char targetLine[GMR_MENU_LINE_SIZE];
    char speedLine[GMR_MENU_LINE_SIZE];
    char accelerationLine[GMR_MENU_LINE_SIZE];

    GmrMenu_BuildSignedLine(targetLine, "Target ",
        StateMachine_GetMission9TargetPosition());
    GmrMenu_BuildUnsignedLine(speedLine, "Speed RPM ",
        GMR_MISSION9_SPEED_RPM);
    GmrMenu_BuildUnsignedLine(accelerationLine, "Accel ",
        GMR_MISSION9_ACCELERATION);
    GmrMenu_ShowPage("H7 Step Test", targetLine, speedLine,
        accelerationLine, "K5 Move K3 Stop");
}

static void GmrMenu_RenderRampTiltTest(CarState state)
{
    char positionLine[GMR_MENU_LINE_SIZE];
    char speedLine[GMR_MENU_LINE_SIZE];
    char elapsedLine[GMR_MENU_LINE_SIZE];
    const char *statusLine;

    GmrMenu_BuildSignedLine(positionLine, "Stepper steps ",
        StateMachine_GetRampTiltTestPosition());
    GmrMenu_BuildSignedLine(speedLine, "Speed ",
        StateMachine_GetRampTiltTestSpeedTarget());
    GmrMenu_BuildUnsignedLine(elapsedLine, "Elapsed ms ",
        StateMachine_GetTaskElapsedMs());
    if (state == CAR_STATE_MISSION) {
        statusLine = "RUN K3 Stop";
    } else if (state == CAR_STATE_FINISHED) {
        statusLine = "DONE pipe -> zero";
    } else {
        statusLine = "STOP pipe -> zero";
    }
    GmrMenu_ShowPage("Ramp Tilt Test", positionLine, speedLine,
        elapsedLine, statusLine);
}

static void GmrMenu_RenderMainMenu(void)
{
    char taskLine[GMR_MENU_LINE_SIZE];
    char otherLine[GMR_MENU_LINE_SIZE];

    GmrMenu_BuildSelectionLine(taskLine,
        (uint8_t)(g_mainSelection == 0U), "TASK");
    GmrMenu_BuildSelectionLine(otherLine,
        (uint8_t)(g_mainSelection == 1U), "OTHER");
    GmrMenu_ShowPage("MAIN MENU", taskLine, otherLine,
        "K2 Enter", "");
}

static void GmrMenu_RenderTaskMenu(void)
{
    char taskLines[4][GMR_MENU_LINE_SIZE];
    uint8_t firstItem;
    uint8_t item;
    uint8_t row;

    firstItem = (g_taskSelection < 4U) ? 0U :
        (uint8_t)(g_taskSelection - 3U);
    for (row = 0U; row < 4U; ++row) {
        item = (uint8_t)(firstItem + row);
        if (item < GMR_MENU_TASK_ITEM_COUNT) {
            GmrMenu_BuildSelectionLine(taskLines[row],
                (uint8_t)(item == g_taskSelection),
                g_taskMenuLabels[item]);
        } else {
            taskLines[row][0] = '\0';
        }
    }
    GmrMenu_ShowPage("TASK  K2 Go K3 Back", taskLines[0], taskLines[1],
        taskLines[2], taskLines[3]);
}

static void GmrMenu_RenderOtherMenu(void)
{
    char taskLines[4][GMR_MENU_LINE_SIZE];
    uint8_t firstItem;
    uint8_t item;
    uint8_t row;

    firstItem = (g_otherSelection < 4U) ? 0U :
        (uint8_t)(g_otherSelection - 3U);
    for (row = 0U; row < 4U; ++row) {
        item = (uint8_t)(firstItem + row);
        if (item < GMR_MENU_OTHER_ITEM_COUNT) {
            GmrMenu_BuildSelectionLine(taskLines[row],
                (uint8_t)(item == g_otherSelection),
                (item == GMR_MENU_OTHER_STOP_COUNT_INDEX) ?
                    "Stop Count Flash" :
                    TaskRegistry_GetName(g_otherMenuTaskIndices[item]));
        } else {
            taskLines[row][0] = '\0';
        }
    }
    GmrMenu_ShowPage("OTHER  K3 Back", taskLines[0], taskLines[1],
        taskLines[2], taskLines[3]);
}

static void GmrMenu_RenderStopCountConfig(void)
{
    char itemLine[GMR_MENU_LINE_SIZE];
    char valueLine[GMR_MENU_LINE_SIZE];
    char *write;
    char *end;

    if (g_stopCountSaveResult == GMR_STOP_COUNT_SAVE_OK) {
        GmrMenu_ShowPage("Stop Count Flash", "Saved to Flash",
            "Runtime updated", "K2 Back", "K3 Back");
        return;
    }
    if (g_stopCountSaveResult == GMR_STOP_COUNT_SAVE_FAILED) {
        GmrMenu_ShowPage("Stop Count Flash", "SAVE FAILED",
            "Values not changed", "K2 Retry", "K3 Cancel");
        return;
    }

    write = itemLine;
    end = &itemLine[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendText(write, end, "Item ");
    write = GmrMenu_AppendUnsigned(write, end,
        (uint32_t)g_stopCountSelection + 1U);
    write = GmrMenu_AppendChar(write, end, '/');
    (void)GmrMenu_AppendUnsigned(write, end,
        (uint32_t)GMR_STOP_COUNT_ITEM_COUNT);

    write = valueLine;
    end = &valueLine[GMR_MENU_LINE_SIZE - 1U];
    write = GmrMenu_AppendText(write, end,
        g_stopCountLabels[g_stopCountSelection]);
    (void)GmrMenu_AppendUnsigned(write, end,
        g_stopCountDraft.values[g_stopCountSelection]);

    GmrMenu_ShowPage("Stop Count Flash", itemLine, valueLine,
        "K1 +100  K4 -100",
        (g_stopCountSelection == (GMR_STOP_COUNT_ITEM_COUNT - 1U)) ?
            "K2 Save K3 Cancel" : "K2 Next K3 Cancel");
}

static void GmrMenu_Render(CarState state)
{
    uint8_t missionId = StateMachine_GetMissionId();

    if (((missionId == 8U) ||
        (missionId == CAR_MISSION_ID_GMR_TASK3_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_TASK4_TRACK_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_TASK5_TRACK_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_TASK6_TRACK_BALL) ||
        (missionId == CAR_MISSION_ID_GMR_IMU_Y_FF_TEST)) &&
        (state != CAR_STATE_MENU)) {
        /* H7滚球模式运行后由H7本地刷新LCD，M0不再覆盖状态页。 */
        if (StateMachine_IsTaskMenuMission(missionId) != 0U) {
            GmrMenu_RenderTaskTimer();
        }
        return;
    }
    if (state == CAR_STATE_MENU) {
        if (g_menuPage == GMR_MENU_PAGE_STOP_COUNT) {
            GmrMenu_RenderStopCountConfig();
        } else if ((g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) ||
            (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT)) {
            GmrMenu_RenderTask4Config();
        } else if (g_menuPage == GMR_MENU_PAGE_TASK) {
            GmrMenu_RenderTaskMenu();
        } else if (g_menuPage == GMR_MENU_PAGE_OTHER) {
            GmrMenu_RenderOtherMenu();
        } else {
            GmrMenu_RenderMainMenu();
        }
    } else if (state == CAR_STATE_MISSION) {
        if (StateMachine_GetMissionId() ==
            CAR_MISSION_ID_GMR_RAMP_TILT_TEST) {
            GmrMenu_RenderRampTiltTest(state);
        } else if (StateMachine_GetMissionId() == 9U) {
            GmrMenu_RenderH7StepperTest();
        } else if (StateMachine_GetMissionId() == 7U) {
            GmrMenu_RenderGrayDifferential("Line Follow");
        } else if (StateMachine_GetMissionId() == 6U) {
            GmrMenu_RenderGrayDifferential("IR Differential");
        } else if (StateMachine_GetMissionId() == 5U) {
            GmrMenu_RenderDirection();
        } else if (StateMachine_GetMissionId() == 4U) {
            GmrMenu_RenderDrive();
        } else if (StateMachine_GetMissionId() == 3U) {
            GmrMenu_RenderEncoder();
        } else if (StateMachine_GetMissionId() == 2U) {
            GmrMenu_RenderTaskTimer();
        } else {
            GmrMenu_RenderAttitude();
        }
    } else if (state == CAR_STATE_STOP) {
        if (missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST) {
            GmrMenu_RenderRampTiltTest(state);
        } else if (StateMachine_IsTaskMenuMission(missionId) != 0U) {
            GmrMenu_RenderTaskTimer();
        } else {
            GmrMenu_ShowPage("Stopped", "K3 Back", "", "", "");
        }
    } else if (state == CAR_STATE_ERROR) {
        GmrMenu_ShowPage("Error", "K3 Back", "", "", "");
    } else if (state == CAR_STATE_FINISHED) {
        if (missionId == CAR_MISSION_ID_GMR_RAMP_TILT_TEST) {
            GmrMenu_RenderRampTiltTest(state);
        } else if (StateMachine_IsTaskMenuMission(missionId) != 0U) {
            GmrMenu_RenderTaskTimer();
        } else {
            GmrMenu_ShowPage("Finished", "K3 Back", "", "", "");
        }
    } else {
        GmrMenu_ShowPage("Init", "", "", "", "");
    }
}

void Menu_Init(void)
{
    GmrStopCountStore_Init();
    g_forceRefresh = 1U;
    g_lastState = CAR_STATE_INIT;
    g_mainSelection = 0U;
    g_taskSelection = 0U;
    g_otherSelection = 0U;
    g_menuPage = GMR_MENU_PAGE_MAIN;
    g_task4LeftTarget = GMR_MISSION4_LEFT_SPEED_COUNTS_PER_PERIOD;
    g_task4RightTarget = GMR_MISSION4_RIGHT_SPEED_COUNTS_PER_PERIOD;
    g_stopCountSelection = 0U;
    g_stopCountSaveResult = GMR_STOP_COUNT_EDITING;
}

void Menu_Next(void)
{
    if ((g_menuPage == GMR_MENU_PAGE_STOP_COUNT) &&
        (g_stopCountSaveResult == GMR_STOP_COUNT_EDITING)) {
        g_stopCountDraft.values[g_stopCountSelection] =
            GmrMenu_IncreaseStopCount(
                g_stopCountDraft.values[g_stopCountSelection]);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_task4LeftTarget = GmrMenu_NextTask4Target(g_task4LeftTarget);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        g_task4RightTarget = GmrMenu_NextTask4Target(g_task4RightTarget);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK) {
        g_taskSelection = (uint8_t)((g_taskSelection + 1U) %
            GMR_MENU_TASK_ITEM_COUNT);
    } else if (g_menuPage == GMR_MENU_PAGE_OTHER) {
        g_otherSelection = (uint8_t)((g_otherSelection + 1U) %
            GMR_MENU_OTHER_ITEM_COUNT);
    } else {
        g_mainSelection = (uint8_t)((g_mainSelection + 1U) %
            GMR_MENU_MAIN_ITEM_COUNT);
    }
    Menu_RequestRefresh();
}

void Menu_Previous(void)
{
    if ((g_menuPage == GMR_MENU_PAGE_STOP_COUNT) &&
        (g_stopCountSaveResult == GMR_STOP_COUNT_EDITING)) {
        g_stopCountDraft.values[g_stopCountSelection] =
            GmrMenu_DecreaseStopCount(
                g_stopCountDraft.values[g_stopCountSelection]);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_task4LeftTarget = GmrMenu_PreviousTask4Target(g_task4LeftTarget);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        g_task4RightTarget = GmrMenu_PreviousTask4Target(g_task4RightTarget);
    } else if (g_menuPage == GMR_MENU_PAGE_TASK) {
        g_taskSelection = (g_taskSelection == 0U) ?
            (GMR_MENU_TASK_ITEM_COUNT - 1U) :
            (uint8_t)(g_taskSelection - 1U);
    } else if (g_menuPage == GMR_MENU_PAGE_OTHER) {
        g_otherSelection = (g_otherSelection == 0U) ?
            (GMR_MENU_OTHER_ITEM_COUNT - 1U) :
            (uint8_t)(g_otherSelection - 1U);
    } else {
        g_mainSelection = (g_mainSelection == 0U) ?
            (GMR_MENU_MAIN_ITEM_COUNT - 1U) :
            (uint8_t)(g_mainSelection - 1U);
    }
    Menu_RequestRefresh();
}

CarEvent Menu_Confirm(void)
{
    uint8_t taskIndex;

    if (g_menuPage == GMR_MENU_PAGE_MAIN) {
        g_menuPage = (g_mainSelection == 0U) ?
            GMR_MENU_PAGE_TASK : GMR_MENU_PAGE_OTHER;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    if (g_menuPage == GMR_MENU_PAGE_STOP_COUNT) {
        if (g_stopCountSaveResult == GMR_STOP_COUNT_SAVE_OK) {
            g_menuPage = GMR_MENU_PAGE_OTHER;
        } else if (g_stopCountSaveResult == GMR_STOP_COUNT_SAVE_FAILED) {
            g_stopCountSaveResult =
                (GmrStopCountStore_Save(&g_stopCountDraft) != 0U) ?
                    GMR_STOP_COUNT_SAVE_OK :
                    GMR_STOP_COUNT_SAVE_FAILED;
        } else if (g_stopCountSelection <
            (GMR_STOP_COUNT_ITEM_COUNT - 1U)) {
            ++g_stopCountSelection;
        } else {
            g_stopCountSaveResult =
                (GmrStopCountStore_Save(&g_stopCountDraft) != 0U) ?
                    GMR_STOP_COUNT_SAVE_OK :
                    GMR_STOP_COUNT_SAVE_FAILED;
        }
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_menuPage = GMR_MENU_PAGE_TASK4_RIGHT;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        StateMachine_SetMission4Targets(g_task4LeftTarget,
            g_task4RightTarget);
        g_menuPage = GMR_MENU_PAGE_OTHER;
        return CAR_EVENT_MISSION_4_START;
    }
    if (g_menuPage == GMR_MENU_PAGE_TASK) {
        taskIndex = g_taskMenuTaskIndices[g_taskSelection];
    } else {
        if (g_otherSelection == GMR_MENU_OTHER_STOP_COUNT_INDEX) {
            GmrStopCountStore_GetValues(&g_stopCountDraft);
            g_stopCountSelection = 0U;
            g_stopCountSaveResult = GMR_STOP_COUNT_EDITING;
            g_menuPage = GMR_MENU_PAGE_STOP_COUNT;
            Menu_RequestRefresh();
            return CAR_EVENT_NONE;
        }
        taskIndex = g_otherMenuTaskIndices[g_otherSelection];
    }
    if (TaskRegistry_GetMissionId(taskIndex) == 4U) {
        g_menuPage = GMR_MENU_PAGE_TASK4_LEFT;
        Menu_RequestRefresh();
        return CAR_EVENT_NONE;
    }
    return TaskRegistry_GetStartEvent(taskIndex);
}

uint8_t Menu_Back(void)
{
    if (g_menuPage == GMR_MENU_PAGE_MAIN) {
        return 0U;
    }
    if (g_menuPage == GMR_MENU_PAGE_TASK4_RIGHT) {
        g_menuPage = GMR_MENU_PAGE_TASK4_LEFT;
    } else if (g_menuPage == GMR_MENU_PAGE_TASK4_LEFT) {
        g_menuPage = GMR_MENU_PAGE_OTHER;
    } else if (g_menuPage == GMR_MENU_PAGE_STOP_COUNT) {
        g_menuPage = GMR_MENU_PAGE_OTHER;
    } else {
        g_menuPage = GMR_MENU_PAGE_MAIN;
    }
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
