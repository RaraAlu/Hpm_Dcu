#include "FreeRTOS.h"
#include "task.h"

static StaticTask_t worker_tcb;
static StackType_t worker_stack[2048] __attribute__((aligned(16)));
static TaskHandle_t worker_handle;
volatile unsigned fixture_phase, fixture_failure, fixture_nested_hits;

void dcu_rtos_assert(const char *file, unsigned long line)
{
    (void)file;
    fixture_failure = line ? line : 1;
    for (;;)
        __asm volatile("nop");
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    dcu_rtos_assert(name, __LINE__);
}

void freertos_exception_handler(void)
{
    dcu_rtos_assert("unexpected trap", __LINE__);
}

void irq_handler_trap(void)
{
    dcu_rtos_assert("unexpected non-vector IRQ", __LINE__);
}

__attribute__((noinline)) void fixture_done(void)
{
    for (;;)
        __asm volatile("nop");
}

static void fixture_worker(void *argument)
{
    (void)argument;
    fixture_phase = 1;
    for (unsigned i = 0; i < 2; ++i) {
        configASSERT(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 1);
        ++fixture_phase;
    }
    fixture_done();
}

void irq_handler_wrapper_1(void)
{
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(worker_handle, &wake);
    portYIELD_FROM_ISR(wake);
}

void irq_handler_wrapper_5(void)
{
    ++fixture_nested_hits;
}

void fixture_start(void)
{
    worker_handle = xTaskCreateStatic(fixture_worker, "port-test", 2048, NULL,
                                     configMAX_PRIORITIES - 1, worker_stack, &worker_tcb);
    configASSERT(worker_handle != NULL);
    vTaskStartScheduler();
    dcu_rtos_assert("scheduler returned", __LINE__);
}
