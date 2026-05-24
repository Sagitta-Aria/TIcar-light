#ifndef POSE_SOLVER_H
#define POSE_SOLVER_H

#include <stdint.h>

/*
 * PoseSolverPose：车体相对启动点的二维位姿。
 * 坐标约定：yaw=0 时，y 为车头前进方向，x 为车体右侧方向。
 */
typedef struct {
    int32_t xMm;
    int32_t yMm;
    int32_t travelMm;
    int32_t leftSteps;
    int32_t rightSteps;
    int16_t rollDeg;
    int16_t pitchDeg;
    int16_t yawDeg;
    uint8_t hasImu;
} PoseSolverPose;

/* PoseSolver_Init：初始化位姿解算状态。 */
void PoseSolver_Init(void);

/* PoseSolver_Reset：把当前位置作为新的位姿零点。 */
void PoseSolver_Reset(void);

/* PoseSolver_Task：周期更新位姿，不阻塞、不访问 OLED/串口。 */
void PoseSolver_Task(void);

/* PoseSolver_GetPose：复制当前位姿快照。 */
void PoseSolver_GetPose(PoseSolverPose *pose);

int32_t PoseSolver_GetXmm(void);
int32_t PoseSolver_GetYmm(void);
int32_t PoseSolver_GetTravelMm(void);
int16_t PoseSolver_GetYawDeg(void);
uint8_t PoseSolver_HasImu(void);

#endif
