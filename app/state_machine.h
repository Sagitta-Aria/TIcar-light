#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

typedef enum {
    CAR_STATE_INIT = 0,
    CAR_STATE_IDLE,
    CAR_STATE_TRACKING,
    CAR_STATE_MOTOR_TEST,
    CAR_STATE_STOP,
    CAR_STATE_ERROR
} CarState;

typedef enum {
    CAR_EVENT_NONE = 0,
    CAR_EVENT_START,
    CAR_EVENT_STOP,
    CAR_EVENT_MOTOR_TEST_NEXT,
    CAR_EVENT_ERROR,
    CAR_EVENT_CLEAR_ERROR
} CarEvent;

void StateMachine_Init(void);
void StateMachine_Dispatch(CarEvent event);
void StateMachine_Task(void);
CarState StateMachine_GetState(void);
const char *StateMachine_GetStateName(CarState state);

#endif
