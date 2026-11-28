#include "ethercat_can_task.h"
#include "ethercat_cycle.h"
#include "gateway_flow.h"
#include "cfg/ethercat_can_task_config.h"
#include "cycle_timers.h"
#include "cfg/cycle_timer_map.h"
#include "cycle_config.h"
#include "can_transport.h"
#include "realtime_port.h"
#include "FreeRTOS.h"
#include "task.h"

static volatile dcu_rtos_diagnostics_t rtos_diagnostics;
#if defined(CONFIG_CAN_BATCH_SAMPLE)
static uint32_t last_wake_us;
uint32_t dcu_worker_wake_us(void) { return last_wake_us; }
#endif

void dcu_rtos_read_diagnostics(dcu_rtos_diagnostics_t *snapshot)
{
    *snapshot = rtos_diagnostics;
}

_Static_assert(configMAX_SYSCALL_INTERRUPT_PRIORITY == DCU_IRQ_KERNEL_PRIORITY,
               "Kernel and cycle IRQ masks must use the same threshold");

void ecat_can_task(void *argument)
{
    (void)argument;
    uint32_t last_stack_check_ms = 0;
    dcu_worker_timer_start();
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        const uint32_t start_us = dcu_time_now_us();
#if defined(CONFIG_CAN_BATCH_SAMPLE)
        last_wake_us = start_us;
#endif
        uint32_t pass_us = start_us;
        while (dcu_cycle_service_pdo_can(pass_us) == DCU_CYCLE_WAIT_CAN_FEEDBACK) {
            /* While this image is open, stay on CAN. Each pass checks the
             * absolute cutoff; CAN pacing and the independent watchdog remain IRQ-driven. */
            pass_us = dcu_time_now_us();
        }
        dcu_cycle_service_ethercat_al();
        dcu_cycle_record_worker_duration(start_us);

        const uint32_t now_ms = dcu_time_now_ms();
        if (now_ms - last_stack_check_ms >= DCU_STACK_CHECK_PERIOD_MS) {
            last_stack_check_ms = now_ms;
            rtos_diagnostics.worker_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        }
    }
}

dcu_gateway_state_t dcu_gateway_next(dcu_gateway_state_t state, dcu_gateway_event_t event,
                                     dcu_gateway_reason_t reason)
{
    const bool lost = event == GW_AUTHORIZATION_LOST &&
                      (reason == DCU_GATEWAY_REASON_CAN_OFFLINE || reason == DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT ||
                       reason == DCU_GATEWAY_REASON_MASTER_TIMEOUT || reason == DCU_GATEWAY_REASON_PROTOCOL_MISMATCH);
    if (event == GW_FATAL) return DCU_GATEWAY_FAULT;
    switch (state) {
    case DCU_GATEWAY_BOOT:
        if (event == GW_BOOT_VALID) return DCU_GATEWAY_PREPARING;
        break;
    case DCU_GATEWAY_PREPARING:
        if (lost) return DCU_GATEWAY_HOLD;
        if (event == GW_FRESH_FEEDBACK) return DCU_GATEWAY_READY;
        break;
    case DCU_GATEWAY_READY:
        if (lost) return DCU_GATEWAY_HOLD;
        if (event == GW_POLICY_APPLIED) return DCU_GATEWAY_PREPARING;
        if (event == GW_OP_READY) return DCU_GATEWAY_RUNNING;
        break;
    case DCU_GATEWAY_RUNNING:
        if (lost) return DCU_GATEWAY_HOLD;
        break;
    case DCU_GATEWAY_HOLD:
        if (event == GW_CAN_HEALTHY || event == GW_POLICY_APPLIED) return DCU_GATEWAY_PREPARING;
        break;
    case DCU_GATEWAY_FAULT:
        break;
    }
    return state;
}

dcu_layout_phase_t dcu_layout_next(dcu_layout_phase_t phase, dcu_layout_event_t event,
                                   dcu_pdo_mode_t mode, bool topology_pending, bool operational)
{
    const bool request = (event == PDO_SDO_2101_01_REQUEST && !topology_pending) ||
                         (event == PDO_SDO_2103_01_SELECT && !operational);
    const bool confirm = event == PDO_SDO_2101_02_CONFIRM && !operational;
    const bool map = event == PDO_PREOP_MAPPING && !topology_pending && !operational;
    const bool commit_topology = event == PDO_SDO_2103_06_COMMIT && topology_pending && !operational;
    const dcu_layout_phase_t mapped = mode == DCU_PDO_MOTOR ? PDO_DISABLED : PDO_WAIT_OP;
    /* IgH rebuilds mailboxes via INIT before OP. A mapped fixed PDO is not
     * an active session; never preserve a negotiation or a debug lease. */
    if (event == PDO_FIXED_MAILBOX_STOP && phase == PDO_WAIT_OP &&
        mode == DCU_PDO_MOTOR_WITH_RAW && !topology_pending && !operational) return PDO_WAIT_OP;
    if (event == PDO_ABORT || event == PDO_FIXED_MAILBOX_STOP)
        return phase == PDO_DISABLED ? PDO_DISABLED : PDO_FAULTED;
    switch (phase) {
    case PDO_DISABLED:
    case PDO_WAIT_OP:
    case PDO_ACTIVE:
        if (request) return PDO_DRAINING;
        if (map) return mapped;
        if (event == PDO_ENTER_OP && phase == PDO_WAIT_OP) return PDO_ACTIVE;
        if (event == PDO_DROP_LIMIT && phase == PDO_ACTIVE) return PDO_FAULTED;
        if (event == PDO_LEAVE_OP) return phase == PDO_DISABLED ? PDO_DISABLED : PDO_FAULTED;
        break;
    case PDO_DRAINING:
    case PDO_DRAINING_CONFIRMED:
        if (event == PDO_DRAIN_TIMEOUT || event == PDO_DROP_LIMIT) return PDO_FAULTED;
        if (event == PDO_CAN_DRAINED)
            return phase == PDO_DRAINING_CONFIRMED ? PDO_WAIT_COMMIT : PDO_WAIT_CONFIRM;
        if (event == PDO_LEAVE_OP) return phase;
        if (event == PDO_ENTER_OP) return PDO_DRAINING;
        break;
    case PDO_WAIT_CONFIRM:
        if (event == PDO_TOPOLOGY_TIMEOUT || event == PDO_DROP_LIMIT) return PDO_FAULTED;
        if (event == PDO_LEAVE_OP) return phase;
        if (confirm) return PDO_WAIT_COMMIT;
        break;
    case PDO_WAIT_COMMIT:
        if (event == PDO_TOPOLOGY_TIMEOUT || event == PDO_DROP_LIMIT) return PDO_FAULTED;
        if (confirm) return PDO_WAIT_COMMIT;
        if (event == PDO_LEAVE_OP) return phase;
        if (event == PDO_ENTER_OP) return PDO_WAIT_CONFIRM;
        if (map || commit_topology) return mapped;
        break;
    case PDO_FAULTED:
    case PDO_RECOVERY_PERMITTED:
        if (confirm) return PDO_RECOVERY_PERMITTED;
        if (phase == PDO_RECOVERY_PERMITTED && request && !operational) return PDO_DRAINING_CONFIRMED;
        if (event == PDO_ENTER_OP) return PDO_FAULTED;
        if (map) return phase; /* Re-map old motor layout without granting raw access. */
        if (event == PDO_LEAVE_OP) return PDO_FAULTED;
        break;
    case PDO_REFUSED:
        break;
    }
    return PDO_REFUSED;
}

DCU_FAST_CODE dcu_session_phase_t dcu_session_next(dcu_session_phase_t phase, dcu_session_event_t event,
                                                   bool operational_and_gateway_running)
{
    if (event == PDO_REVOKE_SESSION) return PDO_WAIT_BASELINE;
    if (!operational_and_gateway_running) return PDO_FRAME_REFUSED;
    switch (phase) {
    case PDO_WAIT_BASELINE:
        if (event == PDO_ZERO_ENABLE_BASELINE || event == PDO_EMPTY_FD_BASELINE) return PDO_CYCLIC;
        break;
    case PDO_CYCLIC:
        if (event == PDO_STOP_WITH_TARGETS) return PDO_WAIT_BASELINE;
        return PDO_CYCLIC;
    case PDO_FRAME_REFUSED:
        break;
    }
    return PDO_FRAME_REFUSED;
}

SDK_DECLARE_EXT_ISR_M(DCU_WORKER_TIMER_IRQ, dcu_worker_timer_isr)
void dcu_worker_timer_isr(void)
{
    const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
    const bool periodic = dcu_worker_timer_ack();
    const bool gap = dcu_can_wake_timer_ack();
    if (gap) dcu_can_gap_elapsed_isr();
    portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    if (periodic) dcu_cycle_watchdog_isr();
    if (periodic) dcu_notify_ecat_can_from_isr();
}
void dcu_rtos_assert(const char *file, unsigned long line)
{
    (void)disable_global_irq(CSR_MSTATUS_MIE_MASK);
    rtos_diagnostics.fatal_file = file;
    rtos_diagnostics.fatal_line = line;
    dcu_tx_inhibit();
    for (unsigned c = 0; c < DCU_CAN_BUS_COUNT; ++c) dcu_can_cancel_from_isr(c);
    for (;;) __asm volatile("nop");
}
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    dcu_rtos_assert(name, __LINE__);
}
