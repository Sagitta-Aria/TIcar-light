#ifndef MENU_H
#define MENU_H

#include "state_machine.h"

/* Menu_Init：初始化 OLED 菜单选择状态。 */
void Menu_Init(void);

/* Menu_Next：菜单状态下切换到下一个选项，到底后回到第一个。 */
void Menu_Next(void);

/* Menu_Confirm：菜单状态下确认当前选项，返回需要发送给状态机的事件。 */
CarEvent Menu_Confirm(void);

/* Menu_Task：根据当前整车状态刷新 OLED 页面，并在监视页打印数据。 */
void Menu_Task(CarState state);

#endif
