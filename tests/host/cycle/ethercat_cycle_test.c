#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>
#include "hpm_gptmr_drv.h"
#include "port/detail/esc_pdo_copy.h"
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
bool bEcatInputUpdateRunning = true;
uint16_t nPdOutputSize = 784;
uint16_t nPdInputSize = 1072, nEscAddrInputData = 0x1400;
uint16_t LatchInputSync0Value = 1, LatchInputSync0Counter;
uint8_t nAlStatus = 8;
uint8_t u8LocalErrorState;
uint16_t u16LocalErrorCode;
bool bEcatWaitForAlControlRes;
uint32_t test_esc_period = 2000000U, test_esc_activation = 0x300U;
uint16_t test_al_event;
test_gptmr_t test_timebase;
#define time_us test_timebase.CHANNEL[0].CNT
static uint32_t prepare_cost_us, mapping_cost_us;
static uint32_t sync_calls, tick_calls, output_calls, prepare_calls, mapping_calls, fault_calls;
static uint32_t cancel_calls;
static uint8_t last_output[784];
static uint8_t processed_output, published_output;
static bool command_pending, defer_process;
static bool neutral_feedback_pending;
static uint32_t cyclic_required, cyclic_missing, cyclic_period_us, cyclic_deadline_us;
#if defined(CONFIG_PDO_PHASE_TRACE)
static uint32_t observed_retries;
uint32_t dcu_can_observed_retx_count(void) { return observed_retries; }
#endif
static uint32_t neutral_period_ns, neutral_cutoff_us, neutral_deadline_calls;
static uint8_t requested_state;
static uint16_t requested_error;
static bool sync_during_mapping;
static jmp_buf worker_exit;
static unsigned worker_iterations;
static uint32_t fake_esc_output[784 / 4];
static unsigned direct_reads;
static bool test_event_pdo;
bool motor_control_event_pdo(void) { return test_event_pdo; }
uint32_t motor_control_event_lease_us(void) { return 10000; }

static uint32_t read_sm2_into_slot(void *destination, uint16_t address, uint16_t size)
{
    CHECK(address == 0x1000 && ((uintptr_t)destination & 3U) == 0U);
    ++direct_reads;
    CHECK(dcu_esc_pdo_read_words(destination, fake_esc_output, 4));
    dcu_cycle_sync0_isr();
    pdo_pipeline_view_t half_written;
    CHECK(!pdo_pipeline_acquire(&pipeline, &half_written));
    CHECK(dcu_esc_pdo_read_words((uint8_t *)destination + 4, fake_esc_output + 1, size - 4));
    return time_us + 1U;
}

uint32_t dcu_time_now_us(void) { return time_us; }
uint32_t dcu_time_ticks(void) { return time_us * 24U; }
uint32_t dcu_time_tick_hz(void) { return 24000000U; }
bool dcu_worker_timer_ack(void) { return true; }
bool dcu_can_wake_timer_ack(void) { return false; }
void dcu_can_gap_elapsed_isr(void) {}
void dcu_worker_timer_start(void) {}
uint32_t dcu_time_now_ms(void) { return time_us / 1000U; }
void dcu_can_cancel_from_isr(unsigned channel) { CHECK(channel < 4); ++cancel_calls; }
void motor_control_preview(const void *data) { (void)data; }
void motor_control_output(const void *data)
{
    ++output_calls;
    memcpy(last_output, data, nPdOutputSize);
    command_pending = true;
}
void motor_control_prepare_input(void) { ++prepare_calls; time_us += prepare_cost_us; }
bool motor_control_neutral_feedback_pending(void) { return neutral_feedback_pending; }
void motor_control_set_neutral_window(uint32_t period_ns, uint32_t cutoff_us)
{
    neutral_period_ns = period_ns;
    neutral_cutoff_us = cutoff_us;
}
void motor_control_neutral_feedback_deadline(void)
{
    ++neutral_deadline_calls;
    neutral_feedback_pending = false;
}
void motor_control_set_cyclic_feedback_window(uint32_t period_us, uint32_t deadline_us)
{
    cyclic_period_us = period_us;
    cyclic_deadline_us = deadline_us;
}
uint32_t motor_control_cyclic_feedback_required(void) { return cyclic_required; }
uint32_t motor_control_cyclic_feedback_missing(void) { return cyclic_missing; }
void motor_control_fault_cyclic_feedback(uint32_t missing) { CHECK(missing == cyclic_missing); }
void *dcu_ssc_txpdo_buffer(uint16_t *capacity)
{
    static uint32_t buffer[1072 / 4];
    *capacity = sizeof(buffer);
    return buffer;
}
bool hw_esc_publish_pdo_before(uint16_t address, const void *src, uint16_t size, uint32_t deadline_us)
{
    CHECK(address == nEscAddrInputData && src && size == nPdInputSize);
    if ((int32_t)(deadline_us - time_us) <= (int32_t)mapping_cost_us) {
        time_us += mapping_cost_us;
        return false;
    }
    PDO_InputMapping();
    return true;
}
#if defined(CONFIG_PDO_PHASE_TRACE)
void dcu_pdo_phase_record_rx(const void *data, uint32_t stage_start_us, uint32_t stage_commit_us,
                             uint32_t direct_to_slot, uint32_t direct_read_end_us)
{
    (void)data;
    (void)stage_start_us;
    (void)stage_commit_us;
    (void)direct_to_slot;
    (void)direct_read_end_us;
}
#endif
bool motor_control_process(void)
{
    if (command_pending && defer_process) return false;
    if (command_pending) processed_output = last_output[0];
    command_pending = false;
    return true;
}
void motor_control_cycle_fault(void) { ++fault_calls; }
void Sync0_Isr(void)
{
    ++sync_calls;
    if (LatchInputSync0Value && ++LatchInputSync0Counter == LatchInputSync0Value) {
        PDO_InputMapping();
        LatchInputSync0Counter = 0;
    }
}
void Sync1_Isr(void) {}
void ECAT_CheckTimer(void) { ++tick_calls; }
void MainLoop(void) {}
void PDO_InputMapping(void)
{
    ++mapping_calls;
    test_al_event &= ~PROCESS_INPUT_EVENT;
    published_output = processed_output;
    time_us += mapping_cost_us;
    if (sync_during_mapping)
        dcu_cycle_sync0_isr();
}
void ECAT_StateChange(uint8_t state, uint16_t error)
{
    requested_state = state;
    requested_error = error;
    u8LocalErrorState = error ? state : 0;
    u16LocalErrorCode = error;
}
void intc_m_disable_irq(uint32_t irq) { (void)irq; }
void intc_m_enable_irq_with_priority(uint32_t irq, uint32_t priority) { (void)irq; (void)priority; }
uint32_t disable_global_irq(uint32_t mask) { return mask; }
void restore_global_irq(uint32_t mask) { (void)mask; }
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

static void select_sm2(void)
{
    bDcSyncActive = false;
    test_esc_activation = 0;
    sSyncManOutPar.u16SyncType = SYNCTYPE_SM_SYNCHRON;
    sSyncManInPar.u16SyncType = SYNCTYPE_SM2_SYNCHRON;
}

static void sm2_frame(uint32_t arrival, uint8_t value)
{
    time_us = arrival;
    dcu_cycle_sm2_event_isr();
    time_us += 132;
    uint8_t command[784];
    memset(command, value, sizeof(command));
    dcu_cycle_stage(command);
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
    if (argc == 2 && (!strcmp(argv[1], "raw_events") || !strcmp(argv[1], "raw_stalled"))) {
        test_event_pdo = true;
        nPdOutputSize = DCU_SINGLE_FD_PDO_BYTES;
        dcu_cycle_init();
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_SYNCHRONNOTSUPPORTED);
        bDcSyncActive = false;
        test_esc_activation = 0;
        sSyncManOutPar.u16SyncType = SYNCTYPE_SM_SYNCHRON;
        sSyncManInPar.u16SyncType = SYNCTYPE_SM2_SYNCHRON;
        CHECK(!dcu_cycle_configure() && !dcu_cycle_start_outputs());
        uint8_t a[DCU_SINGLE_FD_PDO_BYTES] = {1}, b[DCU_SINGLE_FD_PDO_BYTES] = {2};
        dcu_cycle_stage(a);
        dcu_cycle_stage(b);
        dcu_cycle_step();
        CHECK(output_calls == 1 && published_output == 1 && mapping_calls == 1);
        time_us += 3500;
        dcu_cycle_step();
        CHECK(output_calls == 2 && processed_output == 2 && mapping_calls == 1 && !fault_calls);
        time_us += 3500;
        test_al_event |= PROCESS_INPUT_EVENT;
        dcu_cycle_step();
        CHECK(output_calls == 2 && published_output == 2 && mapping_calls == 2 && !fault_calls);
        if (!strcmp(argv[1], "raw_stalled")) {
            time_us += 10000;
            dcu_cycle_watchdog_isr();
            CHECK(dcu_tx_inhibited() && cancel_calls == 4 && !fault_calls);
            dcu_cycle_step();
            CHECK(fault_calls == 1 && requested_state == STATE_SAFEOP);
        }
        return 0;
    }
    CHECK(argc == 2);
    if (!strncmp(argv[1], "cyclic_", 7)) {
        start();
        const uint32_t required = 0xFU;
        cyclic_required = cyclic_missing = required;
        command_boundary(100);
        const uint32_t prepares = prepare_calls;
        dcu_cycle_step();
        CHECK(cyclic_period_us == 2000 && cyclic_deadline_us == 1700);
        time_us = 1300;
        cyclic_missing = 0x8;
        dcu_cycle_step();
        CHECK(!mapping_calls && prepare_calls == prepares && !fault_calls);
        if (!strcmp(argv[1], "cyclic_missing")) {
            time_us = 1701;
            dcu_cycle_watchdog_isr();
            CHECK(dcu_tx_inhibited());
            dcu_cycle_step();
            check_stopped(PDO_PIPELINE_CAN_FEEDBACK_MISSING);
            CHECK(cycle_diagnostics.missing_can_devices == 0x8 && !mapping_calls && prepare_calls == prepares);
        } else {
            cyclic_missing = 0;
            prepare_cost_us = 100;
            mapping_cost_us = !strcmp(argv[1], "cyclic_write_late") ? 301 : 200;
            dcu_cycle_step();
            if (mapping_cost_us == 301) {
                check_stopped(PDO_PIPELINE_TX_LATE);
                CHECK(!mapping_calls);
            } else {
                CHECK(mapping_calls == 1 && cycle_diagnostics.published_epoch == 1 && !fault_calls);
                CHECK(prepare_calls == prepares + 1 && published_output == 0xA5 && time_us == 1600);
            }
        }
        return 0;
    }
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (!strcmp(argv[1], "neutral_wait") || !strcmp(argv[1], "neutral_wait_late")) {
        test_esc_period = 2000000U;
        start();
        time_us = 85;
        observed_retries = UINT32_MAX;
        command_boundary(120);
        dcu_cycle_step();
        CHECK(neutral_period_ns == 2000000U && neutral_cutoff_us == 1270U);
        neutral_feedback_pending = true;
        time_us = 1269;
        dcu_cycle_step();
        volatile const dcu_pdo_phase_sample_t *sample = &pdo_phase_trace.sample[0];
        CHECK(!sample->neutral_wait_checks && !mapping_calls && neutral_feedback_pending);
        CHECK(sample->last_consumed_offset_from_sync0 == UINT32_MAX);
        dcu_trace_neutral_consumed(600);
        dcu_trace_neutral_consumed(1000);
        CHECK(sample->last_consumed_offset_from_sync0 == 880);
        observed_retries = 1;
        if (!strcmp(argv[1], "neutral_wait_late")) {
            time_us = 1871;
            dcu_cycle_step();
            check_stopped(PDO_PIPELINE_TX_LATE);
            CHECK(!mapping_calls && !sample->publish_begin_us);
            CHECK(pdo_phase_trace.fault_epoch == sample->epoch && pdo_phase_trace.fault_us == 1871);
            CHECK(pdo_phase_trace.fault_flags == PDO_PIPELINE_TX_LATE);
        } else {
            time_us = 1270;
            prepare_cost_us = 50;
            mapping_cost_us = 300;
            dcu_cycle_step();
            CHECK(mapping_calls == 1 && !fault_calls && !pdo_phase_trace.fault_flags);
            CHECK(neutral_deadline_calls == 1 && !neutral_feedback_pending);
            CHECK(sample->neutral_wait_checks == 1 && sample->neutral_last_wait_us == 1270);
            CHECK(sample->retx_count == 2);
            CHECK(sample->publish_begin_us == 1270 && sample->pack_end_us == 1320);
            CHECK(sample->esc_begin_us == 1320 && sample->esc_end_us == 1620 && sample->published_us == 1620);
        }
        return 0;
    }
#endif
    if (!strcmp(argv[1], "sm2_config")) {
        select_sm2();
        sSyncManInPar.u16SyncType = SYNCTYPE_DCSYNC0;
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_SYNCHRONNOTSUPPORTED);
        sSyncManInPar.u16SyncType = SYNCTYPE_SM2_SYNCHRON;
        test_esc_activation = ESC_DC_SYNC_UNIT_AUTO_ACTIVE_MASK;
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        test_esc_activation = 0;
        start();
        CHECK(pipeline.period_ns == 1000000U);
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_DCINVALIDSYNCCFG);
        test_esc_activation = ESC_DC_SYNC0_ACTIVE_MASK;
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_CONFIG_CHANGED);
        return 0;
    }
    if (!strcmp(argv[1], "sm2_exchange")) {
        select_sm2();
        time_us = 900;
        start();
        memset(fake_esc_output, 0xA5, sizeof(fake_esc_output));
        time_us = 1000;
        dcu_cycle_sm2_event_isr();
        time_us = 1132;
        CHECK(dcu_cycle_stage_read(0x1000, 784, read_sm2_into_slot));
        CHECK(output_calls == 0 && !mapping_calls && !fault_calls);
        dcu_cycle_step();
        CHECK(output_calls == 1 && sync_calls == 0 && !mapping_calls);
        CHECK(!memcmp(last_output, fake_esc_output, 784));
        time_us = 1150;
        dcu_cycle_step();
        CHECK(mapping_calls == 1 && published_output == 0xA5);

        defer_process = true;
        sm2_frame(2000, 0xB6);
        CHECK(mapping_calls == 1 && published_output == 0xA5);
        time_us = 2150;
        dcu_cycle_step();
        CHECK(output_calls == 2 && command_pending && last_output[0] == 0xB6);
        CHECK(mapping_calls == 2 && published_output == 0xA5);
        CHECK(!fault_calls && !dcu_tx_inhibited());
        return 0;
    }
    if (!strcmp(argv[1], "sm2_deadline") || !strcmp(argv[1], "sm2_missing")) {
        select_sm2();
        time_us = 900;
        start();
        sm2_frame(1000, 0xA5);
        time_us = 1150;
        if (!strcmp(argv[1], "sm2_deadline")) {
            prepare_cost_us = DCU_TX_BUILD_BUDGET_US + 1;
            dcu_cycle_step();
            CHECK(mapping_calls == 0);
            check_stopped(PDO_PIPELINE_TX_LATE);
        } else {
            dcu_cycle_step();
            time_us = 2000 + DCU_WORKER_PERIOD_US + 1;
            dcu_cycle_watchdog_isr();
            CHECK(dcu_tx_inhibited() && cancel_calls == 4 && !fault_calls);
            dcu_cycle_step();
            check_stopped(PDO_PIPELINE_RX_MISSING);
            CHECK(output_calls == 1 && mapping_calls == 1);
        }
        return 0;
    }
    if (!strcmp(argv[1], "esc_sm2_direct_stage")) {
        const uint16_t sizes[] = {544, 736, 784};
        uint8_t *bytes = (uint8_t *)fake_esc_output;
        for (unsigned i = 0; i < sizeof(fake_esc_output); ++i)
            bytes[i] = (uint8_t)((i * 59U) ^ (i >> 4U));
        for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
            nPdOutputSize = sizes[i];
            time_us = 100;
            start();
            CHECK(!dcu_cycle_stage_read(0x1000, sizes[i] - 4, read_sm2_into_slot));
            CHECK(dcu_cycle_stage_read(0x1000, sizes[i], read_sm2_into_slot));
            CHECK(direct_reads == i + 1);
            CHECK(pdo_pipeline_mark_published(&pipeline, 1, 1800));
            time_us = 2100;
            dcu_cycle_sync0_isr();
            pdo_pipeline_view_t output;
            CHECK(pdo_pipeline_acquire(&pipeline, &output));
            CHECK(output.epoch == 2 && output.sequence == 1 && output.ready_us == 101U);
            CHECK(!memcmp(output.data, bytes, sizes[i]));
            for (unsigned j = sizes[i]; j < sizeof(fake_esc_output); ++j)
                CHECK(output.data[j] == 0U);
            pdo_pipeline_release(&pipeline, &output);
            dcu_cycle_stop_outputs();
            CHECK(!dcu_cycle_stage_read(0x1000, sizes[i], read_sm2_into_slot));
            CHECK(direct_reads == i + 1);
        }
        return 0;
    }
    if (!strcmp(argv[1], "esc_sm2_word_read")) {
        uint32_t source[196];
        _Alignas(4) uint8_t dest[792];
        uint8_t *bytes = (uint8_t *)source;
        const unsigned sizes[] = {4, 8, 544, 736, 784};
        for (unsigned i = 0; i < sizeof(source); ++i)
            bytes[i] = (uint8_t)((i * 73U) ^ (i >> 3));
        for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i)
            for (unsigned offset = 0; offset < 4; ++offset) {
                memset(dest, 0xB5, sizeof(dest));
                CHECK(dcu_esc_pdo_read_words(dest + offset, source, sizes[i]));
                CHECK(!memcmp(dest + offset, bytes, sizes[i]));
                for (unsigned j = 0; j < offset; ++j) CHECK(dest[j] == 0xB5);
                for (unsigned j = offset + sizes[i]; j < sizeof(dest); ++j) CHECK(dest[j] == 0xB5);
            }
        memset(dest, 0xB5, sizeof(dest));
        CHECK(!dcu_esc_pdo_read_words(dest, bytes + 2, 4));
        CHECK(!dcu_esc_pdo_read_words(dest, bytes, 7));
        for (unsigned i = 0; i < sizeof(dest); ++i) CHECK(dest[i] == 0xB5);
        return 0;
    }
    if (!strcmp(argv[1], "esc_pdo_copy")) {
        _Alignas(4) uint8_t src[1080];
        uint32_t dest[271];
        const uint8_t *out = (const uint8_t *)(dest + 1);
        const unsigned sizes[] = {0, 1, 2, 3, 4, 7, 15, 16, 17, 20, 24, 28, 31, 32, 33, 544, 1024, 1072};
        for (unsigned i = 0; i < sizeof(src); ++i)
            src[i] = (uint8_t)(i ^ (i >> 8));
        for (unsigned align = 0; align < 4; ++align)
            for (unsigned i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
                memset(dest, 0xB5, sizeof(dest));
                dcu_esc_pdo_copy(dest + 1, src + align, sizes[i]);
                CHECK(dest[0] == UINT32_C(0xb5b5b5b5));
                CHECK(!memcmp(out, src + align, sizes[i]));
                for (unsigned j = sizes[i]; j < sizeof(dest) - sizeof(dest[0]); ++j)
                    CHECK(out[j] == 0xB5);
            }
        return 0;
    }
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
    if (!strcmp(argv[1], "process_before_publish") || !strcmp(argv[1], "process_deadline")) {
        test_esc_period = 1000000U;
        start();
        time_us = 85;
        command_boundary(120);
        time_us = 280; /* First worker dispatch is already inside the TxPDO window. */
        defer_process = true;
        dcu_cycle_step();
        CHECK(output_calls == 1 && mapping_calls == 0 && runtime.command_pending);
        if (!strcmp(argv[1], "process_deadline")) {
            time_us = 871; /* Sync0 + 750 us is still the deadline. */
            dcu_cycle_step();
            CHECK(mapping_calls == 0);
            check_stopped(PDO_PIPELINE_TX_LATE);
        } else {
            defer_process = false;
            time_us = 330;
            dcu_cycle_step();
            CHECK(mapping_calls == 1 && published_output == 0xA5 && !runtime.command_pending);
            time_us = 400;
            dcu_cycle_step();
            CHECK(mapping_calls == 1 && !fault_calls);
        }
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
        CHECK(mapping_calls == 0);
        time_us = 1250;
        prepare_cost_us = 200;
        mapping_cost_us = 300;
        dcu_cycle_step();
        dcu_cycle_diagnostics_t snapshot;
        dcu_cycle_read_diagnostics(&snapshot);
        CHECK(mapping_calls == 1 && snapshot.published_epoch == 1 && snapshot.publish_max_us == 500);
        CHECK(snapshot.prepare_max_us == 200 && snapshot.esc_write_max_us == 300);
        CHECK(snapshot.publish_start_delay_max_us == 0);
        command_boundary(2100);
        dcu_cycle_step();
        CHECK(output_calls == 2 && fault_calls == 0);
        CHECK(mapping_calls == 1);
        CHECK(cycle_timing.sync_interval.count == 1 && cycle_timing.sync_interval.histogram[2000] == 1);
        CHECK(cycle_timing.rx_ready_lead.count == 2 && cycle_timing.rx_ready_lead.min_us == 100);
        CHECK(cycle_timing.rx_ready_lead.max_us == 350 && cycle_timing.rx_ready_lead.sum_us == 450);
        dcu_cycle_stop_outputs();
        time_us = 4100;
        dcu_cycle_sync0_isr();
        dcu_cycle_step();
        CHECK(mapping_calls == 2);
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
#if defined(CONFIG_PDO_PHASE_TRACE)
        volatile const dcu_pdo_phase_sample_t *sample = &pdo_phase_trace.sample[0];
        CHECK(sample->publish_begin_us == 1750 && sample->pack_end_us == 1800);
        CHECK(sample->esc_begin_us == 1800 && sample->esc_end_us == 1851 && !sample->published_us);
#endif
        return 0;
    }
    if (!strcmp(argv[1], "pack_already_late")) {
        time_us = 1750;
        prepare_cost_us = 101;
        dcu_cycle_step();
        CHECK(mapping_calls == 0);
        check_stopped(PDO_PIPELINE_TX_LATE);
#if defined(CONFIG_PDO_PHASE_TRACE)
        volatile const dcu_pdo_phase_sample_t *sample = &pdo_phase_trace.sample[0];
        CHECK(sample->publish_begin_us == 1750 && sample->pack_end_us == 1851);
        CHECK(!sample->esc_begin_us && !sample->esc_end_us && !sample->published_us);
#endif
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
        dcu_cycle_diagnostics_t snapshot;
        dcu_cycle_read_diagnostics(&snapshot);
        CHECK(snapshot.fault_al_status == 8 && snapshot.fault_handled_us == 2100);
        CHECK(snapshot.first_rx_miss_epoch == 2 && snapshot.first_rx_miss_sync_us == 2100);
        CHECK(snapshot.first_rx_miss_last_stage_us == 0 && !snapshot.first_rx_miss_stage_busy);
        CHECK(output_calls == 1);
        return 0;
    }
    if (!strcmp(argv[1], "preop_recovery")) {
        time_us = 1750;
        dcu_cycle_step();
        time_us = 2100;
        dcu_cycle_sync0_isr();
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_RX_MISSING);
        nAlStatus = STATE_SAFEOP;
        dcu_cycle_step();
        CHECK(u16LocalErrorCode == ALSTATUSCODE_SYNCERROR);
        dcu_cycle_stop_inputs();
        nAlStatus = STATE_PREOP;
        bEcatWaitForAlControlRes = true;
        dcu_cycle_step();
        CHECK(u16LocalErrorCode == ALSTATUSCODE_SYNCERROR);
        bEcatWaitForAlControlRes = false;
        dcu_cycle_step();
        CHECK(u8LocalErrorState == 0 && u16LocalErrorCode == 0);
        CHECK(cycle_diagnostics.local_sync_error_clears == 1);
        CHECK(!runtime.outputs_running && output_calls == 1);
        CHECK(dcu_cycle_configure() == ALSTATUSCODE_NOERROR);
        CHECK(dcu_cycle_start_outputs() == ALSTATUSCODE_NOERROR);
        return 0;
    }
    if (!strcmp(argv[1], "safeop_recovery") || !strcmp(argv[1], "unrelated_error")) {
        time_us = 1750;
        dcu_cycle_step();
        time_us = 2100;
        dcu_cycle_sync0_isr();
        dcu_cycle_step();
        check_stopped(PDO_PIPELINE_RX_MISSING);
        nAlStatus = STATE_SAFEOP;
        const bool unrelated = !strcmp(argv[1], "unrelated_error");
        if (unrelated) u16LocalErrorCode = ALSTATUSCODE_SMWATCHDOG;
        dcu_cycle_step();
        CHECK(u8LocalErrorState == STATE_SAFEOP);
        dcu_cycle_ack_safeop_error();
        bEcatWaitForAlControlRes = true;
        dcu_cycle_step();
        CHECK(u8LocalErrorState == STATE_SAFEOP);
        bEcatWaitForAlControlRes = false;
        dcu_cycle_step();
        CHECK(cycle_diagnostics.local_sync_error_clears == (unrelated ? 0U : 1U));
        CHECK(u16LocalErrorCode == (unrelated ? ALSTATUSCODE_SMWATCHDOG : 0U));
        CHECK(output_calls == 1 && !runtime.outputs_running && dcu_tx_inhibited());
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
