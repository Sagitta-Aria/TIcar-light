#ifndef MENU_H
#define MENU_H

#include "state_machine.h"

/* Menu_Init：初始化Task1至Task9菜单和默认选项，不直接访问具体屏幕。 */
void Menu_Init(void);

/* Menu_Next：菜单状态下切换任务入口或子项。 */
void Menu_Next(void);

/* Menu_Confirm：确认当前任务入口，返回状态机事件。 */
CarEvent Menu_Confirm(void);

/* Menu_Back：菜单子页返回主任务列表；已经在主列表时返回 0。 */
uint8_t Menu_Back(void);

/* Menu_RequestRefresh：请求UI任务下一轮立即刷新所有已启用显示后端。 */
void Menu_RequestRefresh(void);

/* Menu_Task：根据当前状态刷新菜单；不要在中断或10ms控制路径调用。 */
void Menu_Task(CarState state);

#endif
