#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

/* CarState：比赛正式版顶层状态；任务细分阶段不应再增加顶层状态。 */
typedef enum {
    CAR_STATE_INIT = 0, /* 应用初始化尚未完成。 */
    CAR_STATE_MENU,     /* 等待K1/K2选择比赛任务。 */
    CAR_STATE_MISSION,  /* 某个Task正在运行。 */
    CAR_STATE_FINISHED, /* 任务正常达到完成条件。 */
    CAR_STATE_STOP,     /* 用户长按停止或任务安全停车。 */
    CAR_STATE_ERROR     /* 板级致命错误，禁止启动电机。 */
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

/* 底盘调试模式；GMR Task1 / Full Task6使用。 */
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

/* StateMachine_ChassisControlPeriod：在10ms底盘周期先生成本拍目标速度。 */
void StateMachine_ChassisControlPeriod(void);

/* StateMachine_HandleChassisFastEvent：立即消费灰度中断形成的左右入弯和回线事件。 */
void StateMachine_HandleChassisFastEvent(void);

/* StateMachine_GetState：读取当前顶层状态。 */
CarState StateMachine_GetState(void);

/* StateMachine_GetStateName：把状态转成短字符串，供 OLED 显示。 */
const char *StateMachine_GetStateName(CarState state);

/* StateMachine_GetMissionId：读取当前任务编号，0 表示未进入任务。 */
uint8_t StateMachine_GetMissionId(void);

/* 按 library_config.h 的当前组合判断某个比赛任务是否可以进入。 */
uint8_t StateMachine_IsMissionAvailable(uint8_t missionId);

/* StateMachine_SetMission1LapCount：设置 Task 1 要跑的圈数，范围 1~5。 */
void StateMachine_SetMission1LapCount(uint8_t lapCount);

/* StateMachine_GetMission1LapCount：读取 Task 1 当前目标圈数。 */
uint8_t StateMachine_GetMission1LapCount(void);

/* 设置/读取 Task4 子菜单路线。 */
void StateMachine_SetMission4Route(CarMission4Route route);
CarMission4Route StateMachine_GetMission4Route(void);

/* 设置底盘调试任务的开闭环模式和速度；有效missionId由Profile决定。 */
void StateMachine_SetMissionDriveConfig(uint8_t missionId,
    CarChassisDriveMode mode, uint16_t speed);

/* 读取底盘调试任务当前选择的控制模式。 */
CarChassisDriveMode StateMachine_GetMissionDriveMode(uint8_t missionId);

/* 读取底盘调试任务当前速度；单位由控制模式决定。 */
uint16_t StateMachine_GetMissionDriveSpeed(uint8_t missionId);

/* StateMachine_GetMission4Stage：读取 Task4 当前内部阶段。 */
CarMission4Stage StateMachine_GetMission4Stage(void);

/* StateMachine_GetMission4Flag：读取 Task4 当前位置 flag，等于已完成转向次数。 */
uint32_t StateMachine_GetMission4Flag(void);

/* StateMachine_IsMissionGimbalPrepDone：读取 Task3/Task4 前置 yaw 搜索是否完成。 */
uint8_t StateMachine_IsMissionGimbalPrepDone(void);

#endif
