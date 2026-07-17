#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "ti_msp_dl_config.h"

#define configENABLE_MPU                        0
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (CPUCLK_FREQ)
#define configTICK_RATE_HZ                      ((TickType_t)1000U)
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                ((uint16_t)96U)
#define configMAX_TASK_NAME_LEN                 16
#define configIDLE_SHOULD_YIELD                 0

#define configSUPPORT_STATIC_ALLOCATION         1
#define configKERNEL_PROVIDED_STATIC_MEMORY     1
#define configSUPPORT_DYNAMIC_ALLOCATION        0

#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configCHECK_HANDLER_INSTALLATION        0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIMERS                        0
#define configUSE_EVENT_GROUPS                  0
#define configUSE_STREAM_BUFFERS                0
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_CO_ROUTINES                    0
#define configQUEUE_REGISTRY_SIZE               0

#define INCLUDE_vTaskDelete                     0
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_xTaskGetSchedulerState          0
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_vTaskSuspend                    1

void App_FreeRtosAssert(const char *file, int line);
#define configASSERT(condition)                 \
    do {                                        \
        if ((condition) == 0) {                 \
            App_FreeRtosAssert(__FILE__, __LINE__); \
        }                                       \
    } while (0)

#endif
