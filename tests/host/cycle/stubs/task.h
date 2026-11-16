#ifndef TEST_TASK_H
#define TEST_TASK_H
#include "FreeRTOS.h"
TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name, uint32_t depth,
                              void *arg, uint32_t priority, StackType_t *stack, StaticTask_t *tcb);
void vTaskStartScheduler(void);
uint32_t ulTaskNotifyTake(BaseType_t clear, uint32_t wait);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *wake);
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t task);
#endif
