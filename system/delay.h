#ifndef DELAY_H
#define DELAY_H

#include <stdint.h>

/*
 * 忙等毫秒延时，只允许上电初始化阶段使用。
 * RTOS调度启动后应使用vTaskDelay，ISR和10ms控制路径严禁调用。
 */
void delay_ms(uint32_t ms);

#endif
