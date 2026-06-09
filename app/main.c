#include "app.h"
#include "board.h"
#include "board_config.h"

/*
 * 作用：整机固件入口。
 * 使用场景：上电或复位后由启动文件跳转到这里。
 * 说明：main 只负责 Board/App 两级调度；具体外设初始化和业务状态机不要堆在这里。
 */
int main(void)
{
    Board_Init();
#if (CAR_RECOVERY_SAFE_BUILD == 0U)
    /*
     * 正常固件才进入 App。
     * 恢复安全固件只跑 Board_Task 心跳，避免 OLED/电机等外设干扰救板。
     */
    if (Board_HasFatalError() == 0U) {
        App_Init();
    }
#endif

    while (1) {
        Board_Task();
#if (CAR_RECOVERY_SAFE_BUILD == 0U)
        /* 出现致命板级错误后停止 App 层，PA14 由 Board_Task 给出常亮信号。 */
        if (Board_HasFatalError() == 0U) {
            App_Task();
        }
#endif
    }
}
