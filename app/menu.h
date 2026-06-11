#ifndef MENU_H
#define MENU_H

#include "state_machine.h"

/* Menu_Init：初始化 OLED 菜单选择状态。 */
void Menu_Init(void);

/* Menu_Next：菜单状态下切换到下一个选项，到底后回到第一个。 */
void Menu_Next(void);

/* Menu_Confirm：菜单状态下确认当前选项，返回需要发送给状态机的事件。 */
CarEvent Menu_Confirm(void);

/* Menu_Back：菜单状态下返回上一级页面；已经在主菜单时返回 0。 */
uint8_t Menu_Back(void);

/* Menu_RequestRefresh：请求下一轮菜单任务立即刷新 OLED。 */
void Menu_RequestRefresh(void);

/* Menu_GrayCalibrationNext：灰度校准页切换保存/不保存退出动作。 */
void Menu_GrayCalibrationNext(void);

/* Menu_GrayCalibrationConfirm：灰度校准页确认当前退出动作。 */
CarEvent Menu_GrayCalibrationConfirm(void);

/*
 * Menu_Task：根据当前整车状态刷新 OLED 页面。
 * 说明：内部带刷新节流，调用者可以每轮主循环调用，不要在中断中调用。
 */
void Menu_Task(CarState state);

#endif
