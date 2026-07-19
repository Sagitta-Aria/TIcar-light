#include "app.h"
#include "board.h"
#include "board_config.h"
#include "rtos_app.h"

int main(void)
{
    /*
     * 启动阶段尚未进入 FreeRTOS：先初始化时钟、GPIO、通信外设和中断。
     * Board_Init() 返回后，中断可能已经产生，但任务句柄尚未创建；RTOS
     * 通知封装会检查空句柄，因此此阶段不会误唤醒尚不存在的任务。
     */
    Board_Init();

#if (CAR_RECOVERY_SAFE_BUILD != 0U)
    /* 恢复版本故意不启动 RTOS，只保留板级探针和故障恢复循环。 */
    for (;;) {
        Board_Task();
    }
#else
    if (Board_HasFatalError() != 0U) {
        /* 致命板级错误下不启动业务任务，避免电机等外设带故障运行。 */
        for (;;) {
            Board_Task();
        }
    }
    /*
     * App_Init() 只初始化业务模块；随后一次性创建所有静态 RTOS 对象并
     * 启动抢占式调度。调度器正常启动后，RtosApp_StartScheduler() 不返回。
     */
    App_Init();
    RtosApp_StartScheduler();
#endif

    return 0;
}
