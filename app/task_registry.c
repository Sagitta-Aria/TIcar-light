#include "task_registry.h"

#include "profile_select.h"

typedef struct {
    const char *name;
    uint8_t missionId;
    CarEvent startEvent;
} TaskRegistryEntry;

#if CAR_PROFILE_IS_GMR
static const TaskRegistryEntry g_tasks[] = {
    { "Task 1 Drive",   1U, CAR_EVENT_MISSION_1_START },
    { "Task 2 PID",     2U, CAR_EVENT_MISSION_2_START },
    { "Task 3 Encoder", 3U, CAR_EVENT_MISSION_3_START },
    { "Task 4 Line",    4U, CAR_EVENT_MISSION_4_START },
    { "Task 5 M0 Yaw",  5U, CAR_EVENT_MISSION_5_START },
    { "Task 6 BT Replay", 6U, CAR_EVENT_MISSION_6_START }
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
