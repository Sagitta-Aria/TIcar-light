#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "menu.h"

#include "board.h"
#include "car_display.h"
#include "m0_attitude_link.h"
#include "task_registry.h"

#define GMR_MENU_LINE_SIZE  (24U)
#define GMR_MENU_ROW_COUNT  (5U)

static uint8_t g_forceRefresh;
static CarState g_lastState;

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
            "UART3 PB2/PB3", "115200 8-N-1", "Hold K2 to stop");
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

static void GmrMenu_Render(CarState state)
{
    if (state == CAR_STATE_MENU) {
        GmrMenu_ShowPage("GMR Tianmeng", "> Attitude",
            "K2 or H7 Start", "H7 UART2 115200", "M0 UART3 115200");
    } else if (state == CAR_STATE_MISSION) {
        GmrMenu_RenderAttitude();
    } else if (state == CAR_STATE_STOP) {
        GmrMenu_ShowPage("Stopped", "K2 Back", "H7 @START=1", "", "");
    } else if (state == CAR_STATE_ERROR) {
        GmrMenu_ShowPage("Error", "K2 Back", "", "", "");
    } else if (state == CAR_STATE_FINISHED) {
        GmrMenu_ShowPage("Finished", "K2 Back", "", "", "");
    } else {
        GmrMenu_ShowPage("Init", "", "", "", "");
    }
}

void Menu_Init(void)
{
    g_forceRefresh = 1U;
    g_lastState = CAR_STATE_INIT;
}

void Menu_Next(void)
{
    Menu_RequestRefresh();
}

CarEvent Menu_Confirm(void)
{
    return TaskRegistry_GetStartEvent(0U);
}

uint8_t Menu_Back(void)
{
    return 0U;
}

void Menu_RequestRefresh(void)
{
    g_forceRefresh = 1U;
}

void Menu_Task(CarState state)
{
    if (state != g_lastState) {
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
