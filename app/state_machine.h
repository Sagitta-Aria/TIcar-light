#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

/* CarState：整车顶层状态，负责决定当前跑哪个任务。 */
typedef enum {
    CAR_STATE_INIT = 0,
    CAR_STATE_IDLE,
    CAR_STATE_MENU,
    CAR_STATE_GRAY_CALIBRATION,
    CAR_STATE_TRACKING,
    CAR_STATE_TRACKING_TEST,
    CAR_STATE_GIMBAL_TEST,
    CAR_STATE_GIMBAL_MOTOR_TEST,
    CAR_STATE_MOTOR_ENABLE_TEST,
    CAR_STATE_MISSION,
    CAR_STATE_FINISHED,
    CAR_STATE_STOP,
    CAR_STATE_ERROR
} CarState;

/* CarEvent：状态机外部输入事件，按键、菜单、异常都转成事件。 */
typedef enum {
    CAR_EVENT_NONE = 0,
    CAR_EVENT_START,
    CAR_EVENT_STOP,
    CAR_EVENT_MENU,
    CAR_EVENT_BACK,
    CAR_EVENT_GRAY_CALIBRATION_START,
    CAR_EVENT_GRAY_CALIBRATION_SAMPLE,
    CAR_EVENT_GRAY_CALIBRATION_APPLY,
    CAR_EVENT_TRACKING_TEST_START,
    CAR_EVENT_GIMBAL_TEST_START,
    CAR_EVENT_GIMBAL_MOTOR_TEST_START,
    CAR_EVENT_MOTOR_ENABLE_TEST_START,
    CAR_EVENT_MISSION_1_START,
    CAR_EVENT_MISSION_2_START,
    CAR_EVENT_MISSION_3_START,
    CAR_EVENT_MISSION_4_START,
    CAR_EVENT_TRACKING_DONE,
    CAR_EVENT_ERROR,
    CAR_EVENT_CLEAR_ERROR
} CarEvent;

/* StateMachine_Init：初始化顶层状态机，默认进入菜单状态。 */
void StateMachine_Init(void);

/* StateMachine_Dispatch：向状态机发送一个事件。 */
void StateMachine_Dispatch(CarEvent event);

/* StateMachine_Task：执行当前状态对应的周期任务。 */
void StateMachine_Task(void);

/* StateMachine_GetState：读取当前顶层状态。 */
CarState StateMachine_GetState(void);

/* StateMachine_GetStateName：把状态转成字符串，便于串口/OLED 调试。 */
const char *StateMachine_GetStateName(CarState state);

/* StateMachine_GetMissionId：读取当前任务编号，0 表示还没有选择任务。 */
uint8_t StateMachine_GetMissionId(void);

#endif
