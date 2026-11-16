#ifndef TEST_FREERTOS_H
#define TEST_FREERTOS_H
#include <stddef.h>
#include <stdint.h>
typedef int BaseType_t;
typedef uint32_t StackType_t;
typedef struct { uint32_t unused; } StaticTask_t;
typedef void *TaskHandle_t;
#define configMAX_PRIORITIES 5U
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 3U
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
void dcu_rtos_assert(const char *file, unsigned long line);
#define configASSERT(x) do { if (!(x)) dcu_rtos_assert(__FILE__, __LINE__); } while (0)
#define portYIELD_FROM_ISR(wake) ((void)(wake))
#endif
