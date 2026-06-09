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

/* PoseSolver_GetXmm：返回相对零点的 x 方向位移，单位毫米。 */
int32_t PoseSolver_GetXmm(void);

/* PoseSolver_GetYmm：返回相对零点的 y 方向位移，单位毫米。 */
int32_t PoseSolver_GetYmm(void);

/* PoseSolver_GetTravelMm：返回累计前进里程估计，单位毫米。 */
int32_t PoseSolver_GetTravelMm(void);

/* PoseSolver_GetYawDeg：返回相对零点的 yaw 角，单位度。 */
int16_t PoseSolver_GetYawDeg(void);

/* PoseSolver_HasImu：返回当前是否有可用 JY61P 姿态数据。 */
uint8_t PoseSolver_HasImu(void);

#endif
