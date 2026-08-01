#ifndef GMR_STOP_COUNT_STORE_H
#define GMR_STOP_COUNT_STORE_H

#include <stdint.h>

/* 菜单调节范围和步长；六个停车计数共用，单位都是编码器累计count。 */
#define GMR_STOP_COUNT_MIN_VALUE              (1000U)
#define GMR_STOP_COUNT_MAX_VALUE            (200000U)
#define GMR_STOP_COUNT_MENU_STEP               (100U)

typedef enum {
    GMR_STOP_COUNT_TASK1_LEFT = 0,
    GMR_STOP_COUNT_TASK1_RIGHT,
    GMR_STOP_COUNT_TASK4_LEFT,
    GMR_STOP_COUNT_TASK4_RIGHT,
    GMR_STOP_COUNT_TASK5_LEFT,
    GMR_STOP_COUNT_TASK5_RIGHT,
    GMR_STOP_COUNT_ITEM_COUNT
} GmrStopCountItem;

typedef struct {
    uint32_t values[GMR_STOP_COUNT_ITEM_COUNT];
} GmrStopCountValues;

/* 启动时读取内部Flash；记录无效时自动使用各任务头文件中的默认宏。 */
void GmrStopCountStore_Init(void);

/* 读取当前RAM运行值；状态机只使用该值，不在控制周期访问Flash。 */
uint32_t GmrStopCountStore_GetValue(GmrStopCountItem item);
void GmrStopCountStore_GetValues(GmrStopCountValues *values);

/* 擦写内部Flash末扇区并写后校验；成功后才更新RAM运行值。 */
uint8_t GmrStopCountStore_Save(const GmrStopCountValues *values);

/* 返回1表示本次启动成功装载过有效Flash记录，返回0表示正在使用默认宏。 */
uint8_t GmrStopCountStore_IsFlashLoaded(void);

#endif
