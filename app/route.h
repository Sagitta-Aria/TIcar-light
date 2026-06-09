#ifndef ROUTE_H
#define ROUTE_H

#include <stdint.h>

/* RouteStage：路线外环状态，负责直道、入弯、转弯和出弯。 */
typedef enum {
    ROUTE_STAGE_IDLE = 0,
    ROUTE_STAGE_STRAIGHT,
    ROUTE_STAGE_APPROACH_CORNER,
    ROUTE_STAGE_TURNING,
    ROUTE_STAGE_EXIT_CORNER
} RouteStage;

/* Route_Init：初始化路线外环，默认进入空闲状态。 */
void Route_Init(void);

/* Route_Start：开始新一圈或新一段路线规划。 */
void Route_Start(void);

/* Route_Stop：停止路线规划，并恢复默认速度参数。 */
void Route_Stop(void);

/* Route_Task：周期更新路线状态和当前速度参数。 */
void Route_Task(void);

/* Route_IsRunning：返回路线外环是否处于工作状态。 */
uint8_t Route_IsRunning(void);

/* Route_GetStage：返回当前路线状态。 */
RouteStage Route_GetStage(void);

/* Route_GetStageName：返回状态名称，便于串口调试。 */
const char *Route_GetStageName(RouteStage stage);

/* Route_GetBaseSpeedSps：返回当前推荐的基础底盘速度，单位 step/s。 */
uint16_t Route_GetBaseSpeedSps(void);

/* Route_GetTurnLimit：返回当前允许的最大转向修正。 */
uint16_t Route_GetTurnLimit(void);

/* Route_GetCornerIndex：返回当前已经通过的拐角编号。 */
uint8_t Route_GetCornerIndex(void);

#endif
