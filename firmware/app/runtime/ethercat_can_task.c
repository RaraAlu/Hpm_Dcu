#include "ethercat_can_task.h"
#include "detail/ethercat_can_task_state.h"
#include "ethercat_cycle.h"
#include "cycle_timers.h"
#include "cfg/cycle_timer_map.h"
#include "cfg/ethercat_can_task_config.h"
#include "cycle_config.h"
#include "can_transport.h"
#include "device_config.h"
#include "realtime_port.h"
#include "FreeRTOS.h"
#include "task.h"

static dcu_ethercat_can_task_state_t worker_task;
static volatile dcu_rtos_diagnostics_t rtos_diagnostics;

void dcu_rtos_read_diagnostics(dcu_rtos_diagnostics_t *snapshot)
{
    *snapshot = rtos_diagnostics;
}

_Static_assert(DCU_WORKER_TASK_PRIORITY < configMAX_PRIORITIES,
               "Worker task priority exceeds the FreeRTOS ready lists");
_Static_assert(configMAX_SYSCALL_INTERRUPT_PRIORITY == DCU_IRQ_KERNEL_PRIORITY,
               "Kernel and cycle IRQ masks must use the same threshold");

SDK_DECLARE_EXT_ISR_M(DCU_WORKER_TIMER_IRQ, dcu_worker_timer_isr)
void dcu_worker_timer_isr(void)
{
    if (!dcu_worker_timer_ack()) return;
    dcu_cycle_watchdog_isr();
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(worker_task.handle, &wake);
    portYIELD_FROM_ISR(wake);
}
static void worker(void *argument)
{
    (void)argument;
    dcu_worker_timer_start();
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        dcu_cycle_step();
        uint32_t now = dcu_time_now_ms();
        if ((uint32_t)(now - worker_task.last_stack_check_ms) >= DCU_STACK_CHECK_PERIOD_MS) {
            worker_task.last_stack_check_ms = now;
            rtos_diagnostics.worker_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        }
    }
}
void dcu_rtos_start(void)
{
    worker_task.handle = xTaskCreateStatic(worker, "ecat-can", DCU_WORKER_STACK_WORDS, NULL,
                                          DCU_WORKER_TASK_PRIORITY, worker_task.stack, &worker_task.tcb);
    configASSERT(worker_task.handle != NULL);
    vTaskStartScheduler();
    dcu_rtos_assert(__FILE__, __LINE__);
}
void dcu_rtos_assert(const char *file, unsigned long line)
{
    (void)disable_global_irq(CSR_MSTATUS_MIE_MASK);
    rtos_diagnostics.fatal_file = file;
    rtos_diagnostics.fatal_line = line;
    dcu_tx_inhibit();
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) dcu_can_cancel(c);
    for (;;) __asm volatile("nop");
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    dcu_rtos_assert(name, __LINE__);
}
