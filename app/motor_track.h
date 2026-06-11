#ifndef MOTOR_TRACK_H
#define MOTOR_TRACK_H

#include <stdint.h>

typedef enum {
    MOTOR_TRACK_STATE_IDLE = 0,
    MOTOR_TRACK_STATE_LINE_FAST,
    MOTOR_TRACK_STATE_LINE_SLOW,
    MOTOR_TRACK_STATE_TURNING_SPIN,
    MOTOR_TRACK_STATE_TURN_EXIT_SLOW,
    MOTOR_TRACK_STATE_STOP
} MotorTrackState;

/* MotorTrack_Init：初始化 Motor 菜单真循迹状态，默认不输出底盘命令。 */
void MotorTrack_Init(void);

/* MotorTrack_Start：开始真循迹，flag 清零，速度 3000，重新记录 STEP 起点。 */
void MotorTrack_Start(void);

/* MotorTrack_Stop：停止真循迹并停止底盘。 */
void MotorTrack_Stop(void);

/* MotorTrack_Task：执行一轮红外循迹、右转灰度判断和 yaw 转向确认。 */
void MotorTrack_Task(void);

/* MotorTrack_IsRunning：返回真循迹是否仍在运行。 */
uint8_t MotorTrack_IsRunning(void);

/* MotorTrack_GetState/GetStateName：读取当前真循迹状态，供 OLED 和日志显示。 */
MotorTrackState MotorTrack_GetState(void);
const char *MotorTrack_GetStateName(void);

/* MotorTrack_GetPhase：读取 flag % 4，后续云台参数选择从这里接。 */
uint8_t MotorTrack_GetPhase(void);

/* MotorTrack_GetTravelSteps：读取当前段从起点开始的左右平均 STEP。 */
uint32_t MotorTrack_GetTravelSteps(void);

/* MotorTrack_GetYawDeltaDeg：读取本次右转相对 yawStart 的绝对角度变化。 */
int16_t MotorTrack_GetYawDeltaDeg(void);

/* MotorTrack_IsTurning：返回当前是否处于右转或出弯慢行阶段。 */
uint8_t MotorTrack_IsTurning(void);

#endif
