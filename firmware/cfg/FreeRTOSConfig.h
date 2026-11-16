#ifndef DCU_FREERTOS_CONFIG_H
#define DCU_FREERTOS_CONFIG_H

#include "board.h"

#define configMTIME_BASE_ADDRESS HPM_MCHTMR_BASE
#define configMTIMECMP_BASE_ADDRESS (HPM_MCHTMR_BASE + 8UL)
#define configCPU_CLOCK_HZ 24000000UL
#define configTICK_RATE_HZ 1000U
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0
#define configUSE_TICKLESS_IDLE 0
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE 256U
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 0
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configUSE_TASK_NOTIFICATIONS 1
#define configUSE_MUTEXES 0
#define configUSE_CO_ROUTINES 0
#define configUSE_TIMERS 0
/* The SDK port supplies this static-memory hook even with timers disabled. */
#define configTIMER_TASK_STACK_DEPTH configMINIMAL_STACK_SIZE
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 0
#define configUSE_TRACE_FACILITY 0
#define configGENERATE_RUN_TIME_STATS 0
#define configHSP_ENABLE 0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 3U
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetSchedulerState 1

#ifndef __ASSEMBLER__
void dcu_rtos_assert(const char *file, unsigned long line);
#define configASSERT(condition) do { if (!(condition)) dcu_rtos_assert(__FILE__, __LINE__); } while (0)
#endif

#endif
