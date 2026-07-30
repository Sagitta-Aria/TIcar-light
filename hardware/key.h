#ifndef KEY_H
#define KEY_H

#include <stdint.h>

/* 按键稳定编号，和菜单中的任务编号无关。 */
typedef enum {
    KEY_ID_1 = 0,
    KEY_ID_2,
    KEY_ID_3,
    KEY_ID_4,
    KEY_ID_5,
    KEY_ID_COUNT
} KeyId;

/* 按键松开后投递给应用层的短按事件。 */
typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_1,
    KEY_EVENT_2,
    KEY_EVENT_3,
    KEY_EVENT_4,
    KEY_EVENT_5
} KeyEvent;

/* 初始化按键状态；GPIO本身由SysConfig建立，边沿中断由Interrupt_Init打开。 */
void Key_Init(void);

/* Input任务轮询入口：检测按键松开并把短按事件放入静态队列。 */
void Key_Task(void);

/* 读取某键当前消抖后的按下状态；非法编号返回0。 */
uint8_t Key_IsPressed(KeyId key);

/* 软件事件队列非空时返回1，供Input任务决定是否继续运行。 */
uint8_t Key_HasPendingEvent(void);

/* 取出最早一个按键事件；队列为空时返回KEY_EVENT_NONE。 */
KeyEvent Key_PopEvent(void);

/* GPIO中断入口：只锁存边沿并唤醒Input任务；有相关边沿时返回1。 */
uint8_t Key_HandleGPIOInterrupt(void);

#endif
