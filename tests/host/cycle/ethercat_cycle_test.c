#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include "hpm_gptmr_drv.h"
#include "ethercat_can_task.c"

/* Include the production implementation to exercise its private single-step
 * worker without adding test-only entry points to the firmware API. */
#include "ethercat_cycle.c"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

test_sync_parameters_t sSyncManOutPar = {SYNCTYPE_DCSYNC0};
test_sync_parameters_t sSyncManInPar = {SYNCTYPE_DCSYNC0};
bool bDcSyncActive = true;
uint16_t nPdOutputSize = 784;
uint32_t test_esc_period = 2000000U, test_esc_activation = 0x300U;
test_gptmr_t test_timebase;
#define time_us test_timebase.CHANNEL[0].CNT
static uint32_t prepare_cost_us, mapping_cost_us;
static uint32_t sync_calls, tick_calls, output_calls, prepare_calls, mapping_calls, fault_calls;
static uint32_t cancel_calls;
static uint8_t last_output[784];
static uint8_t requested_state;
static uint16_t requested_error;
static bool sync_during_mapping;
static jmp_buf worker_exit;
static unsigned worker_iterations;

uint32_t dcu_time_now_us(void) { return time_us; }
bool dcu_worker_timer_ack(void) { return true; }
void dcu_worker_timer_start(void) {}
uint32_t dcu_time_now_ms(void) { return time_us / 1000U; }
void dcu_can_cancel(unsigned channel) { CHECK(channel < 4); ++cancel_calls; }
void motor_control_output(const void *data) { ++output_calls; memcpy(last_output, data, sizeof(last_output)); }
void motor_control_prepare_input(void) { ++prepare_calls; time_us += prepare_cost_us; }
void motor_control_process(void) {}
void motor_control_cycle_fault(void) { ++fault_calls; }
void Sync0_Isr(void) { ++sync_calls; }
void Sync1_Isr(void) {}
void ECAT_CheckTimer(void) { ++tick_calls; }
void MainLoop(void) {}
void PDO_InputMapping(void)
{
    ++mapping_calls;
    time_us += mapping_cost_us;
    if (sync_during_mapping)
        dcu_cycle_sync0_isr();
}
void ECAT_StateChange(uint8_t state, uint16_t error)
{
    requested_state = state;
    requested_error = error;
}
void intc_m_disable_irq(uint32_t irq) { (void)irq; }
void intc_m_enable_irq_with_priority(uint32_t irq, uint32_t priority) { (void)irq; (void)priority; }
uint32_t disable_global_irq(uint32_t mask) { return mask; }
void clock_add_to_group(uint32_t clock, uint32_t group) { (void)clock; (void)group; }
uint32_t clock_get_frequency(uint32_t clock) { return clock == clock_gptmr1 ? 1000000U : 24000000U; }
int clock_set_source_divider(uint32_t clock, uint32_t source, uint32_t divider)
{ (void)clock; (void)source; CHECK(divider == 24U); return 0; }
uint32_t gptmr_channel_get_counter(void *timer, uint32_t channel, uint32_t type)
{ (void)timer; (void)channel; (void)type; return time_us; }
bool gptmr_check_status(void *timer, uint32_t mask) { (void)timer; (void)mask; return true; }
void gptmr_clear_status(void *timer, uint32_t mask) { (void)timer; (void)mask; }
void gptmr_enable_irq(void *timer, uint32_t mask) { (void)timer; (void)mask; }
void gptmr_start_counter(void *timer, uint32_t channel) { (void)timer; (void)channel; }
void gptmr_channel_get_default_config(void *timer, gptmr_channel_config_t *config) { (void)timer; config->reload = 0; }
void gptmr_channel_config(void *timer, uint32_t channel, gptmr_channel_config_t *config, bool start)
{ (void)timer; (void)channel; (void)config; (void)start; }
TaskHandle_t xTaskCreateStatic(void (*entry)(void *), const char *name, uint32_t depth,
                              void *arg, uint32_t priority, StackType_t *stack, StaticTask_t *tcb)
{ (void)entry; (void)name; (void)depth; (void)arg; (void)priority; (void)stack; return tcb; }
void vTaskStartScheduler(void) {}
uint32_t ulTaskNotifyTake(BaseType_t clear, uint32_t wait)
{
    (void)clear; (void)wait;
    if (++worker_iterations > 1) longjmp(worker_exit, 1);
    return 1;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *wake) { (void)task; *wake = pdTRUE; }
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t task) { (void)task; return 100; }

static void start(void)
{
    dcu_cycle_init();
    CHECK(dcu_cycle_configure() == ALSTATUSCODE_NOERROR);
    CHECK(dcu_cycle_start_outputs() == ALSTATUSCODE_NOERROR);
}

static void command_boundary(uint32_t time)
{
    uint8_t command[784];
    memset(command, 0xA5, sizeof(command));
    dcu_cycle_stage(command);
    time_us = time;
    dcu_cycle_sync0_isr();
}

static void check_stopped(uint32_t expected_fault)
{
    dcu_cycle_diagnostics_t snapshot;
    dcu_cycle_read_diagnostics(&snapshot);
    CHECK(fault_calls == 1);
    CHECK(snapshot.faults & expected_fault);
    CHECK(requested_state == STATE_SAFEOP && requested_error == ALSTATUSCODE_SYNCERROR);
    CHECK(!runtime.outputs_running);
    CHECK(dcu_tx_inhibited());
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "diagnostics")) {
        dcu_rtos_diagnostics_t rtos;
        dcu_cycle_diagnostics_t first, second;
        dcu_rtos_read_diagnostics(&rtos);
        CHECK(!rtos.worker_stack_free_words && !rtos.fatal_line && !rtos.fatal_file);
        start();
        dcu_cycle_stop_outputs();
        time_us = 1000000U;
        if (!setjmp(worker_exit)) worker(NULL);
        dcu_rtos_read_diagnostics(&rtos);
        CHECK(rtos.worker_stack_free_words == 100);
        dcu_cycle_read_diagnostics(&first);
        CHECK(first.configured_period_ns == test_esc_period);
        first.configured_period_ns = 0;
        rtos.worker_stack_free_words = 0;
        dcu_cycle_read_diagnostics(&second);
        dcu_rtos_read_diagnostics(&rtos);
        CHECK(second.configured_period_ns == test_esc_period && rtos.worker_stack_free_words == 100);
        return 0;
    }
    if (!strcmp(argv[1], "configuration")) {
        bDcSyncActive = false;
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_SYNCHRONNOTSUPPORTED);
        bDcSyncActive = true;
        test_esc_activation = 0x700U;
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        test_esc_activation = 0x300U;
        test_esc_period = 5000001U;
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_DCSYNC0CYCLETIME);
        test_esc_period = 5000000U;
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_NOERROR);
        test_esc_period = 1000000U;
        CHECK(dcu_cycle_start_outputs() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        test_esc_period = 5000000U;
        nPdOutputSize = 0;
        CHECK(dcu_cycle_start_outputs() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        CHECK(!runtime.outputs_running);
        return 0;
    }
    start();
    command_boundary(100);
    CHECK(output_calls == 0 && sync_calls == 0);
    dcu_cycle_timer_isr();
    CHECK(tick_calls == 0);
    dcu_cycle_step();
    CHECK(output_calls == 1 && sync_calls == 1 && tick_calls == 1);
    for (unsigned i = 0; i < sizeof(last_output); ++i)
        CHECK(last_output[i] == 0xA5);

    if (!strcmp(argv[1], "normal")) {
        time_us = 1750;
        dcu_cycle_step();
        dcu_cycle_diagnostics_t snapshot;
        dcu_cycle_read_diagnostics(&snapshot);
        CHECK(mapping_calls == 1 && snapshot.published_epoch == 1);
        command_boundary(2100);
        dcu_cycle_step();
        CHECK(output_calls == 2 && fault_calls == 0);
        return 0;
    }
    if (!strcmp(argv[1], "reconfigure_running")) {
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        CHECK(runtime.outputs_running && pipeline.epoch == 1);
        time_us = 1750;
        dcu_cycle_step();
        dcu_cycle_diagnostics_t snapshot;
        dcu_cycle_read_diagnostics(&snapshot);
        CHECK(fault_calls == 0 && snapshot.published_epoch == 1);
        return 0;
    }
    if (!strcmp(argv[1], "prepare_late")) {
        time_us = 1750;
        prepare_cost_us = 50;
        mapping_cost_us = 51;
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_TX_LATE);
        return 0;
    }
    if (!strcmp(argv[1], "crossed_epoch")) {
        time_us = 1750;
        mapping_cost_us = 350;
        sync_during_mapping = true;
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_TX_LATE);
        return 0;
    }
    if (!strcmp(argv[1], "missing")) {
        time_us = 1750;
        dcu_cycle_step();
        time_us = 2100;
        dcu_cycle_sync0_isr();
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_RX_MISSING);
        CHECK(output_calls == 1);
        return 0;
    }
    if (!strcmp(argv[1], "sync_lost")) {
        time_us = 1750;
        dcu_cycle_step();
        time_us = 4101;
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_SYNC_LOST);
        return 0;
    }
    if (!strcmp(argv[1], "hot_change")) {
        test_esc_period = 1000000U;
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_CONFIG_CHANGED);
        return 0;
    }
    if (!strcmp(argv[1], "worker_stalled")) {
        time_us = 1851;
        dcu_worker_timer_isr();
        CHECK(dcu_tx_inhibited());
        CHECK(cancel_calls == 4 && fault_calls == 0);
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_TX_LATE);
        dcu_cycle_stop_inputs();
        CHECK(!dcu_tx_inhibited());
        return 0;
    }
    if (!strcmp(argv[1], "restart")) {
        uint8_t command[784] = {0};
        dcu_cycle_stage(command);
        dcu_cycle_stop_outputs();
        dcu_cycle_sync0_isr();
        dcu_cycle_sync0_isr();
        CHECK(dcu_cycle_start_outputs() == ALSTATUSCODE_NOERROR);
        time_us = 2100;
        dcu_cycle_sync0_isr();
        dcu_cycle_step();
        CHECK(output_calls == 1 && fault_calls == 0);
        dcu_cycle_stop_inputs();
        CHECK(dcu_cycle_start_outputs() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        return 0;
    }
    return 2;
}
