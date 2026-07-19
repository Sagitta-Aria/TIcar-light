#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

/* CarState：比赛正式版顶层状态。 */
typedef enum {
    CAR_STATE_INIT = 0,
    CAR_STATE_MENU,
    CAR_STATE_MISSION,
    CAR_STATE_FINISHED,
    CAR_STATE_STOP,
    CAR_STATE_ERROR
} CarState;

/* CarEvent：按键、任务完成和异常统一转成事件。 */
typedef enum {
    CAR_EVENT_NONE = 0,
    CAR_EVENT_MISSION_1_START,
    CAR_EVENT_MISSION_2_START,
    CAR_EVENT_MISSION_3_START,
    CAR_EVENT_MISSION_4_START,
    CAR_EVENT_MISSION_5_START,
    CAR_EVENT_MISSION_6_START,
    CAR_EVENT_MISSION_7_START,
    CAR_EVENT_MISSION_8_START,
    CAR_EVENT_MISSION_9_START,
    CAR_EVENT_FINISHED,
    CAR_EVENT_STOP,
    CAR_EVENT_MENU,
    CAR_EVENT_ERROR,
    CAR_EVENT_CLEAR_ERROR
} CarEvent;

/* CarMission4Stage：Task4 内部阶段，不增加顶层状态。 */
typedef enum {
    CAR_MISSION4_STAGE_IDLE = 0,
    CAR_MISSION4_STAGE_TRACK,
    CAR_MISSION4_STAGE_LINE
} CarMission4Stage;

/* Task4 三条比赛路线；点/圆决定视觉误差源，圈数决定目标转向次数。 */
typedef enum {
    CAR_MISSION4_POINT_ONE_LAP = 0,
    CAR_MISSION4_POINT_TWO_LAPS,
    CAR_MISSION4_CIRCLE_ONE_LAP,
    CAR_MISSION4_ROUTE_COUNT
} CarMission4Route;

/* Task6 底盘调试模式；开环速度单位为 PWM%，闭环为 count/控制周期。 */
typedef enum {
    CAR_CHASSIS_DRIVE_OPEN_LOOP = 0,
    CAR_CHASSIS_DRIVE_CLOSED_LOOP
} CarChassisDriveMode;

/* StateMachine_Init：初始化状态机，默认进入菜单。 */
void StateMachine_Init(void);

/* StateMachine_Dispatch：处理外部事件并完成状态切换。 */
void StateMachine_Dispatch(CarEvent event);

/* StateMachine_Task：执行当前状态的周期任务。 */
void StateMachine_Task(void);

/* StateMachine_ChassisControlPeriod：在20ms底盘周期先生成本拍目标速度。 */
void StateMachine_ChassisControlPeriod(void);

/* StateMachine_HandleChassisFastEvent：立即消费灰度中断形成的左右入弯和回线事件。 */
void StateMachine_HandleChassisFastEvent(void);

/* StateMachine_GetState：读取当前顶层状态。 */
CarState StateMachine_GetState(void);

/* StateMachine_GetStateName：把状态转成短字符串，供 OLED 显示。 */
const char *StateMachine_GetStateName(CarState state);

/* StateMachine_GetMissionId：读取当前任务编号，0 表示未进入任务。 */
uint8_t StateMachine_GetMissionId(void);

/* StateMachine_SetMission1LapCount：设置 Task 1 要跑的圈数，范围 1~5。 */
void StateMachine_SetMission1LapCount(uint8_t lapCount);

/* StateMachine_GetMission1LapCount：读取 Task 1 当前目标圈数。 */
uint8_t StateMachine_GetMission1LapCount(void);

/* StateMachine_SetMission2Distance：设置 Task 2 的打靶距离，0~2 对应近/中/远。 */
void StateMachine_SetMission2Distance(uint8_t distance);

/* StateMachine_GetMission2Distance：读取 Task 2 当前打靶距离，0~2 对应近/中/远。 */
uint8_t StateMachine_GetMission2Distance(void);

/* StateMachine_SetMission3Distance：设置 Task 3 的打靶距离，0~2 对应近/中/远。 */
void StateMachine_SetMission3Distance(uint8_t distance);

/* StateMachine_GetMission3Distance：读取 Task 3 当前打靶距离，0~2 对应近/中/远。 */
uint8_t StateMachine_GetMission3Distance(void);

/* 设置/读取 Task4 子菜单路线。 */
void StateMachine_SetMission4Route(CarMission4Route route);
CarMission4Route StateMachine_GetMission4Route(void);

/* StateMachine_SetMission7Distance：设置 Task 7 的圆点追踪距离。 */
void StateMachine_SetMission7Distance(uint8_t distance);

/* StateMachine_GetMission7Distance：读取 Task 7 当前圆点追踪距离。 */
uint8_t StateMachine_GetMission7Distance(void);

/* 设置 Task6 的开闭环模式和对应速度；仅 missionId=6 有效。 */
void StateMachine_SetMissionDriveConfig(uint8_t missionId,
    CarChassisDriveMode mode, uint16_t speed);

/* 读取 Task6 当前选择的底盘控制模式。 */
CarChassisDriveMode StateMachine_GetMissionDriveMode(uint8_t missionId);

/* 读取 Task6 当前速度；单位由控制模式决定。 */
uint16_t StateMachine_GetMissionDriveSpeed(uint8_t missionId);

/* StateMachine_GetMission4Stage：读取 Task4 当前内部阶段。 */
CarMission4Stage StateMachine_GetMission4Stage(void);

/* StateMachine_GetMission4Flag：读取 Task4 当前位置 flag，等于已完成转向次数。 */
uint32_t StateMachine_GetMission4Flag(void);

/* StateMachine_IsMissionGimbalPrepDone：读取 Task3/Task4 前置 yaw 搜索是否完成。 */
uint8_t StateMachine_IsMissionGimbalPrepDone(void);

#endif
