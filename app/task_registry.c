#include "task_registry.h"

#include "profile_select.h"

typedef struct {
    const char *name;
    uint8_t missionId;
    CarEvent startEvent;
} TaskRegistryEntry;

#if CAR_PROFILE_IS_GMR
static const TaskRegistryEntry g_tasks[] = {
    { "Attitude", 1U, CAR_EVENT_MISSION_1_START },
    { "Task2 A-A", 2U, CAR_EVENT_MISSION_2_START },
    { "Encoder", 3U, CAR_EVENT_MISSION_3_START },
    { "Drive Adjustable", 4U, CAR_EVENT_MISSION_4_START },
    { "Direction +20", 5U, CAR_EVENT_MISSION_5_START },
    { "IR Differential", 6U, CAR_EVENT_MISSION_6_START },
    { "Line Follow", 7U, CAR_EVENT_MISSION_7_START },
    { "H7 BMI Ball", 8U, CAR_EVENT_MISSION_8_START },
    { "H7 Step Test", 9U, CAR_EVENT_MISSION_9_START },
    { "Task3 Ball", CAR_MISSION_ID_GMR_TASK3_BALL,
        CAR_EVENT_MISSION_10_START },
    { "Task4 Track+Ball", CAR_MISSION_ID_GMR_TASK4_TRACK_BALL,
        CAR_EVENT_MISSION_11_START },
    { "Task5 Track+Ball", CAR_MISSION_ID_GMR_TASK5_TRACK_BALL,
        CAR_EVENT_MISSION_12_START },
    { "Task6 Track+Ball", CAR_MISSION_ID_GMR_TASK6_TRACK_BALL,
        CAR_EVENT_MISSION_15_START },
    { "BMI Y FF Test", CAR_MISSION_ID_GMR_IMU_Y_FF_TEST,
        CAR_EVENT_MISSION_13_START },
    { "Ramp Tilt Test", CAR_MISSION_ID_GMR_RAMP_TILT_TEST,
        CAR_EVENT_MISSION_14_START }
};
#else
static const TaskRegistryEntry g_tasks[] = {
    { "Task 1",         1U, CAR_EVENT_MISSION_1_START },
    { "Task 2",         2U, CAR_EVENT_MISSION_2_START },
    { "Task 3",         3U, CAR_EVENT_MISSION_3_START },
    { "Task 4",         4U, CAR_EVENT_MISSION_4_START },
    { "Task 5 PID",     5U, CAR_EVENT_MISSION_5_START },
    { "Task 6 Drive",   6U, CAR_EVENT_MISSION_6_START },
    { "Task 7 Circle",  7U, CAR_EVENT_MISSION_7_START },
    { "Task 8 IMU",     8U, CAR_EVENT_MISSION_8_START },
    { "Task 9 Encoder", 9U, CAR_EVENT_MISSION_9_START }
};
#endif

#define TASK_REGISTRY_COUNT \
    ((uint8_t)(sizeof(g_tasks) / sizeof(g_tasks[0])))

uint8_t TaskRegistry_GetCount(void)
{
    return TASK_REGISTRY_COUNT;
}

const char *TaskRegistry_GetName(uint8_t taskIndex)
{
    return (taskIndex < TASK_REGISTRY_COUNT) ? g_tasks[taskIndex].name : "";
}

uint8_t TaskRegistry_GetMissionId(uint8_t taskIndex)
{
    return (taskIndex < TASK_REGISTRY_COUNT) ?
        g_tasks[taskIndex].missionId : 0U;
}

CarEvent TaskRegistry_GetStartEvent(uint8_t taskIndex)
{
    return (taskIndex < TASK_REGISTRY_COUNT) ?
        g_tasks[taskIndex].startEvent : CAR_EVENT_NONE;
}
