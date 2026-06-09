#ifndef APP_H
#define APP_H

/* App_Init：初始化应用层模块；Board_Init 成功后由 main 调用一次。 */
void App_Init(void);

/* App_Task：应用层主循环任务；内部调度按键、状态机、菜单、Link 和运动模块。 */
void App_Task(void);

#endif
