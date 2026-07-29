#ifndef INTERRUPT_H
#define INTERRUPT_H

/*
 * 设置GPIO、UART和定时器中断优先级并使能NVIC。
 * 必须在外设初始化完成、启动FreeRTOS调度器之前调用一次。
 */
void Interrupt_Init(void);

#endif
