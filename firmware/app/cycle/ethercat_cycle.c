#include "ethercat_cycle.h"
#include "applInterface.h"
#include "board.h"
#include "cfg/ethercat_cycle_policy.h"
#include "cycle_config.h"
#include "detail/ethercat_cycle_state.h"
#include "ecat_def.h"
#include "ecatappl.h"
#include "ecatslv.h"
#include "can_transport.h"
#include "hpm_ecat_hw.h"
#include "motor_ethercat.h"
#include "pdo_pipeline.h"
#include "realtime_port.h"
#if defined(CONFIG_SYNC0_HW_CAPTURE_TEST)
#include "sync0_hw_capture.h"
#endif
#if defined(CONFIG_PDO_PHASE_TRACE)
#include "detail/pdo_phase_trace.h"
#endif
#include <string.h>

static volatile dcu_cycle_diagnostics_t cycle_diagnostics;
/* Task-owned timing statistics, read via the debugger only after stopping. */
static volatile dcu_cycle_timing_t cycle_timing;
static pdo_pipeline_t pipeline __attribute__((section(".fast_ram.bss")));

/* These fields alone cross ISR/task ownership; every access is atomic. */
static dcu_cycle_handoff_t handoff;
static dcu_cycle_runtime_t runtime;

#if defined(CONFIG_PDO_PHASE_TRACE)
static volatile dcu_pdo_phase_trace_t pdo_phase_trace;

static uint32_t phase_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static volatile dcu_pdo_phase_sample_t *phase_record_latch(const pdo_pipeline_view_t *view,
                                                           uint32_t worker_us)
{
    uint32_t i = pdo_phase_trace.count++ % DCU_PDO_PHASE_SAMPLES;
    volatile dcu_pdo_phase_sample_t *sample = &pdo_phase_trace.sample[i];
    *sample = (dcu_pdo_phase_sample_t){.sequence = phase_le32(view->data), .epoch = view->epoch,
        .rx_ready_us = view->ready_us, .sync0_us = view->sync_us, .worker_us = worker_us};
    return sample;
}

static volatile dcu_pdo_phase_sample_t *phase_current(uint32_t epoch)
{
    uint32_t count = pdo_phase_trace.count;
    if (!count) return NULL;
    volatile dcu_pdo_phase_sample_t *sample = &pdo_phase_trace.sample[(count - 1U) % DCU_PDO_PHASE_SAMPLES];
    return sample->epoch == epoch ? sample : NULL;
}

static uint32_t phase_command_after_release(const volatile dcu_pdo_phase_sample_t *sample,
                                            uint32_t begin_us, uint32_t end_us)
{
    const uint32_t release = sample->sync0_us + pipeline.period_us - pipeline.advance_us -
                             DCU_TX_BUILD_BUDGET_US;
    if ((int32_t)(end_us - release) <= 0) return 0U;
    return end_us - ((int32_t)(begin_us - release) < 0 ? release : begin_us);
}

static void phase_record_publish(volatile dcu_pdo_phase_sample_t *sample, uint32_t end_us)
{
    if (!sample) return;
    uint16_t capacity;
    const uint8_t *wire = dcu_ssc_txpdo_buffer(&capacity);
    if (wire && capacity >= 8U) sample->accepted = phase_le32(wire + 4);
    sample->published_us = end_us;
    uint32_t dc_low;
    HW_EscReadDWord(dc_low, ESC_SYSTEMTIME_OFFSET);
    sample->dc_sample_us = dcu_time_now_us();
    sample->dc_low_ns = SWAPDWORD(dc_low);
}
#endif

void dcu_cycle_read_diagnostics(dcu_cycle_diagnostics_t *snapshot)
{
    *snapshot = cycle_diagnostics;
}

static bool dc_configuration_matches(uint32_t period_ns)
{
    uint32_t actual_period = 0, activation = 0;
    HW_EscReadDWord(actual_period, ESC_DC_SYNC0_CYCLETIME_OFFSET);
    HW_EscReadDWord(activation, ESC_DC_UNIT_CONTROL_OFFSET);
    return bDcSyncActive && SWAPDWORD(actual_period) == period_ns &&
           (SWAPDWORD(activation) & (ESC_DC_SYNC_UNIT_ACTIVE_MASK |
                                     ESC_DC_SYNC0_ACTIVE_MASK | ESC_DC_SYNC1_ACTIVE_MASK)) ==
               (ESC_DC_SYNC_UNIT_ACTIVE_MASK | ESC_DC_SYNC0_ACTIVE_MASK) &&
           sSyncManOutPar.u16SyncType == SYNCTYPE_DCSYNC0 &&
           sSyncManInPar.u16SyncType == SYNCTYPE_DCSYNC0;
}

static uint32_t take(uint32_t *value)
{
    return __atomic_exchange_n(value, 0U, __ATOMIC_ACQ_REL);
}

static inline uint32_t read_handoff(const uint32_t *value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

static inline void write_handoff(uint32_t *value, uint32_t next)
{
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

static bool sm2_sync_selected(void)
{
    return !bDcSyncActive && sSyncManOutPar.u16SyncType == SYNCTYPE_SM_SYNCHRON &&
           sSyncManInPar.u16SyncType == SYNCTYPE_SM2_SYNCHRON;
}

static bool sm2_configuration_matches(void)
{
    uint32_t activation = 0;
    HW_EscReadDWord(activation, ESC_DC_UNIT_CONTROL_OFFSET);
    return sm2_sync_selected() &&
           !(SWAPDWORD(activation) & (ESC_DC_SYNC_UNIT_ACTIVE_MASK |
                                     ESC_DC_SYNC_UNIT_AUTO_ACTIVE_MASK |
                                     ESC_DC_SYNC0_ACTIVE_MASK | ESC_DC_SYNC1_ACTIVE_MASK));
}

static bool cycle_configuration_matches(void)
{
    return read_handoff(&handoff.sm2_cycles) ? sm2_configuration_matches() :
                                             dc_configuration_matches(pipeline.period_ns);
}

static void check_sm2_gap(uint32_t now_us)
{
    if (read_handoff(&handoff.sm2_cycles) && read_handoff(&pipeline.enabled) &&
        (int32_t)(now_us - read_handoff(&handoff.sm2_event_us)) >
            (int32_t)(pipeline.period_us + DCU_WORKER_PERIOD_US))
        pdo_pipeline_fault(&pipeline, PDO_PIPELINE_RX_MISSING);
}

static void timing_sample(volatile dcu_cycle_timing_histogram_t *stats, uint32_t us)
{
    if (!stats->count || us < stats->min_us) stats->min_us = us;
    if (us > stats->max_us) stats->max_us = us;
    ++stats->count;
    stats->sum_us += us;
    if (us < DCU_CYCLE_TIMING_BINS) ++stats->histogram[us];
    else ++stats->overflow;
}

static void record_timing(const pdo_pipeline_view_t *view)
{
    if (cycle_timing.rx_ready_lead.count && view->epoch - cycle_timing.previous_epoch == 1U)
        timing_sample(&cycle_timing.sync_interval, view->sync_us - cycle_timing.previous_sync_us);
    timing_sample(&cycle_timing.rx_ready_lead, view->sync_us - view->ready_us);
    cycle_timing.previous_epoch = view->epoch;
    cycle_timing.previous_sync_us = view->sync_us;
}

void dcu_cycle_init(void)
{
    (void)pdo_pipeline_configure(&pipeline, 0, 0, 0);
#if defined(CONFIG_SYNC0_HW_CAPTURE_TEST)
    dcu_sync0_capture_init();
#endif
}

uint16_t dcu_cycle_configure(void)
{
    if (runtime.outputs_running)
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    runtime.configured = false;
    const bool sm2 = sm2_sync_selected();
    uint32_t period_ns = DCU_SM2_CYCLE_NS;
    if (sm2) {
        if (!sm2_configuration_matches())
            return ALSTATUSCODE_DCINVALIDSYNCCFG;
    } else {
        if (!bDcSyncActive || sSyncManOutPar.u16SyncType != SYNCTYPE_DCSYNC0 ||
            sSyncManInPar.u16SyncType != SYNCTYPE_DCSYNC0)
            return ALSTATUSCODE_SYNCHRONNOTSUPPORTED;
        HW_EscReadDWord(period_ns, ESC_DC_SYNC0_CYCLETIME_OFFSET);
        period_ns = SWAPDWORD(period_ns);
        if (!dc_configuration_matches(period_ns))
            return ALSTATUSCODE_DCINVALIDSYNCCFG;
    }
    intc_m_disable_irq(IRQn_ESC_SYNC0);
    runtime.configured = pdo_pipeline_configure(&pipeline, period_ns,
                                                DCU_TX_PUBLISH_ADVANCE_US, nPdOutputSize);
    write_handoff(&handoff.sm2_cycles, runtime.configured && sm2);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
    cycle_diagnostics.configured_period_ns = runtime.configured ? period_ns : 0;
    if (!runtime.configured)
        return ALSTATUSCODE_DCSYNC0CYCLETIME;
    motor_control_prepare_input();
    return ALSTATUSCODE_NOERROR;
}

uint16_t dcu_cycle_start_outputs(void)
{
    if (!runtime.configured)
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    if (!cycle_configuration_matches())
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    memset((void *)&cycle_timing, 0, sizeof(cycle_timing));
    /* A new session must not reuse a command staged before OP. */
    intc_m_disable_irq(IRQn_ESC_SYNC0);
    if (!pdo_pipeline_configure(&pipeline, pipeline.period_ns,
                                pipeline.advance_us, nPdOutputSize)) {
        runtime.configured = false;
        intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    }
    (void)take(&handoff.sync0_pending);
    (void)take(&handoff.sync1_pending);
    write_handoff(&handoff.stage_sequence, 0U);
    write_handoff(&handoff.stage_busy, 0U);
    write_handoff(&handoff.stage_done_us, 0U);
    cycle_diagnostics.faults = 0;
    cycle_diagnostics.latched_sequence = 0;
    cycle_diagnostics.published_epoch = 0;
    cycle_diagnostics.worker_max_us = 0;
    cycle_diagnostics.publish_max_us = 0;
    cycle_diagnostics.publish_start_delay_max_us = 0;
    cycle_diagnostics.prepare_max_us = 0;
    cycle_diagnostics.esc_write_max_us = 0;
    cycle_diagnostics.fault_al_status = 0;
    cycle_diagnostics.fault_handled_us = 0;
    cycle_diagnostics.first_rx_miss_epoch = 0;
    cycle_diagnostics.first_rx_miss_sync_us = 0;
    cycle_diagnostics.first_rx_miss_last_stage_us = 0;
    cycle_diagnostics.first_rx_miss_stage_busy = 0;
    cycle_diagnostics.first_rx_miss_stage_start_us = 0;
    cycle_diagnostics.fault_last_stage_us = 0;
#if defined(CONFIG_PDO_PHASE_TRACE)
    pdo_phase_trace.count = 0;
    pdo_phase_trace.fault_flags = 0;
#endif
    /* Keep SSC's Sync0 watchdog and SM checks, but use only the scheduled
     * SSC write in OP. Sync1 is already excluded by the DC configuration. */
    runtime.ssc_sync0_latch = LatchInputSync0Value;
    LatchInputSync0Value = 0U;
    LatchInputSync0Counter = 0U;
    runtime.command_pending = false;
    runtime.processed_epoch = 0U;
    runtime.outputs_running = true;
    motor_control_set_neutral_window(0, 0);
    dcu_tx_allow();
    pipeline.stage_clock_us = dcu_time_now_us;
#if defined(CONFIG_SYNC0_HW_CAPTURE_TEST)
    if (!read_handoff(&handoff.sm2_cycles))
        dcu_sync0_capture_begin(pipeline.period_ns);
#endif
    if (read_handoff(&handoff.sm2_cycles))
        write_handoff(&handoff.sm2_event_us, dcu_time_now_us());
    pdo_pipeline_enable(&pipeline, true);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
    return ALSTATUSCODE_NOERROR;
}

void dcu_cycle_stop_outputs(void)
{
    pdo_pipeline_enable(&pipeline, false);
    if (runtime.outputs_running) {
        LatchInputSync0Value = runtime.ssc_sync0_latch;
        LatchInputSync0Counter = 0U;
    }
    runtime.outputs_running = false;
    runtime.command_pending = false;
}

void dcu_cycle_stop_inputs(void)
{
    dcu_cycle_stop_outputs();
    runtime.configured = false;
    dcu_tx_allow();
}

void dcu_cycle_ack_safeop_error(void)
{
    if (runtime.local_sync_fault && !runtime.outputs_running)
        runtime.safeop_error_acknowledged = true;
}

static void service_local_sync_recovery(void)
{
    if (!runtime.local_sync_fault || runtime.outputs_running || bEcatWaitForAlControlRes)
        return;
    const uint8_t state = nAlStatus & STATE_MASK;
    if (state != STATE_PREOP && state != STATE_INIT &&
        !(state == STATE_SAFEOP && runtime.safeop_error_acknowledged))
        return;

    /* With no pending transition, the SSC API only clears its local latch.
     * Do not complete a transition or acknowledge an AL error for the master. */
    if (u8LocalErrorState == STATE_SAFEOP && u16LocalErrorCode == ALSTATUSCODE_SYNCERROR) {
        ECAT_StateChange(state, ALSTATUSCODE_NOERROR);
        ++cycle_diagnostics.local_sync_error_clears;
    }
    runtime.local_sync_fault = false;
    runtime.safeop_error_acknowledged = false;
}

static inline __attribute__((always_inline)) uint32_t stage_begin(void)
{
    write_handoff(&handoff.stage_start_us, dcu_time_now_us());
    write_handoff(&handoff.stage_busy, 1U);
    return __atomic_add_fetch(&handoff.stage_sequence, 1U, __ATOMIC_RELAXED);
}

DCU_FAST_CODE void dcu_cycle_sm2_event_isr(void)
{
    if (read_handoff(&handoff.sm2_cycles) && read_handoff(&pipeline.enabled)) {
        const uint32_t now_us = dcu_time_now_us();
        check_sm2_gap(now_us);
        write_handoff(&handoff.sm2_event_us, now_us);
    }
}

static void stage_finish(bool committed, const void *data, bool direct, uint32_t direct_read_end_us)
{
    if (committed) {
        const uint32_t done_us = dcu_time_now_us();
        write_handoff(&handoff.stage_done_us, done_us);
        /* The ready slot is complete. Budget from the SM event, not copy end. */
        if (read_handoff(&handoff.sm2_cycles))
            (void)pdo_pipeline_sync(&pipeline, read_handoff(&handoff.sm2_event_us));
#if defined(CONFIG_PDO_PHASE_TRACE)
        dcu_pdo_phase_record_rx(data, read_handoff(&handoff.stage_start_us), done_us,
                                direct, direct_read_end_us);
#else
        (void)data;
        (void)direct;
        (void)direct_read_end_us;
#endif
    }
    write_handoff(&handoff.stage_busy, 0U);
}

void dcu_cycle_stage(const void *data)
{
    if (!read_handoff(&pipeline.enabled)) return;
    uint32_t sequence = stage_begin();
    stage_finish(pdo_pipeline_stage(&pipeline, data, nPdOutputSize, sequence), data, false, 0U);
}

DCU_FAST_CODE bool dcu_cycle_stage_read(uint16_t address, uint16_t size,
                                        uint32_t (*read_output)(void *, uint16_t, uint16_t))
{
    uint32_t index;
    uint8_t *destination = pdo_pipeline_reserve(&pipeline, size, &index);
    if (!destination) return false;
    uint32_t sequence = stage_begin();
    const uint32_t read_end_us = read_output(destination, address, size);
    const bool committed = pdo_pipeline_commit(&pipeline, index, sequence, read_end_us);
    stage_finish(committed, destination, true, read_end_us);
    return true;
}

DCU_FAST_CODE void dcu_cycle_sync0_isr(void)
{
    if (read_handoff(&handoff.sm2_cycles))
        return;
    const uint32_t sync_us = dcu_time_now_us();
    if (pdo_pipeline_sync(&pipeline, sync_us)) {
        const uint32_t busy = read_handoff(&handoff.stage_busy);
        write_handoff(&handoff.miss_epoch, read_handoff(&pipeline.epoch));
        write_handoff(&handoff.miss_sync_us, sync_us);
        write_handoff(&handoff.miss_last_stage_us, read_handoff(&handoff.stage_done_us));
        write_handoff(&handoff.miss_stage_busy, busy);
        write_handoff(&handoff.miss_stage_start_us, busy ? read_handoff(&handoff.stage_start_us) : 0U);
    }
    __atomic_fetch_add(&handoff.sync0_pending, 1U, __ATOMIC_RELEASE);
}

void dcu_cycle_sync1_isr(void)
{
    if (!read_handoff(&handoff.sm2_cycles))
        __atomic_fetch_add(&handoff.sync1_pending, 1U, __ATOMIC_RELEASE);
}

void dcu_cycle_timer_isr(void)
{
    __atomic_fetch_add(&handoff.timer_pending, 1U, __ATOMIC_RELEASE);
}

static bool check_faults(uint32_t now_us)
{
    check_sm2_gap(now_us);
    uint32_t faults = pdo_pipeline_check(&pipeline, now_us);
    if (!runtime.outputs_running || !faults)
        return false;
    cycle_diagnostics.faults |= faults;
    cycle_diagnostics.fault_al_status = nAlStatus;
    cycle_diagnostics.fault_handled_us = now_us;
    cycle_diagnostics.fault_last_stage_us = read_handoff(&handoff.stage_done_us);
#if defined(CONFIG_PDO_PHASE_TRACE)
    pdo_phase_trace.fault_epoch = read_handoff(&pipeline.epoch);
    pdo_phase_trace.fault_us = now_us;
    pdo_phase_trace.fault_flags = faults;
#endif
    if ((faults & PDO_PIPELINE_RX_MISSING) && !read_handoff(&handoff.sm2_cycles)) {
        cycle_diagnostics.first_rx_miss_epoch = read_handoff(&handoff.miss_epoch);
        cycle_diagnostics.first_rx_miss_sync_us = read_handoff(&handoff.miss_sync_us);
        cycle_diagnostics.first_rx_miss_last_stage_us = read_handoff(&handoff.miss_last_stage_us);
        cycle_diagnostics.first_rx_miss_stage_busy = read_handoff(&handoff.miss_stage_busy);
        cycle_diagnostics.first_rx_miss_stage_start_us = read_handoff(&handoff.miss_stage_start_us);
    }
    dcu_tx_inhibit();
    dcu_cycle_stop_outputs();
    motor_control_cycle_fault();
    runtime.local_sync_fault = true;
    runtime.safeop_error_acknowledged = false;
    ECAT_StateChange(STATE_SAFEOP, ALSTATUSCODE_SYNCERROR);
    return true;
}

static void publish_due(void)
{
    const uint32_t epoch = read_handoff(&pipeline.epoch);
    uint32_t now_us = dcu_time_now_us();
    if (check_faults(now_us) || !pdo_pipeline_publish_due(&pipeline, now_us))
        return;
    uint32_t sync_us = read_handoff(&pipeline.sync_us);
    if (epoch != read_handoff(&pipeline.epoch)) return;
    /* DC carries its processed epoch; SM2 publishes the latest available feedback. */
    const bool sm2 = read_handoff(&handoff.sm2_cycles);
    if (!sm2 && read_handoff(&pipeline.received) && runtime.processed_epoch != epoch)
        return;
#if defined(CONFIG_PDO_PHASE_TRACE)
    volatile dcu_pdo_phase_sample_t *sample = !sm2 ? phase_current(epoch) : NULL;
#endif
    if (motor_control_neutral_feedback_pending()) {
#if defined(CONFIG_PDO_PHASE_TRACE)
        if (sample) {
            ++sample->neutral_wait_checks;
            sample->neutral_last_wait_us = now_us;
        }
#endif
        motor_control_neutral_feedback_deadline();
    }
    uint32_t offset_us = now_us - sync_us;
    uint32_t release_us = pipeline.period_us - pipeline.advance_us - DCU_TX_BUILD_BUDGET_US;
    if (offset_us >= release_us && offset_us - release_us > cycle_diagnostics.publish_start_delay_max_us)
        cycle_diagnostics.publish_start_delay_max_us = offset_us - release_us;
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) sample->publish_begin_us = now_us;
#endif
    motor_control_prepare_input();
    uint32_t packed_us = dcu_time_now_us();
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) sample->pack_end_us = packed_us;
#endif
    if (packed_us - now_us > cycle_diagnostics.prepare_max_us)
        cycle_diagnostics.prepare_max_us = packed_us - now_us;
    if (check_faults(packed_us) || epoch != read_handoff(&pipeline.epoch))
        return;
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) sample->esc_begin_us = dcu_time_now_us();
#endif
    PDO_InputMapping();
    uint32_t end_us = dcu_time_now_us();
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) sample->esc_end_us = end_us;
#endif
    if (end_us - packed_us > cycle_diagnostics.esc_write_max_us)
        cycle_diagnostics.esc_write_max_us = end_us - packed_us;
    if (end_us - now_us > cycle_diagnostics.publish_max_us)
        cycle_diagnostics.publish_max_us = end_us - now_us;
    if (pdo_pipeline_mark_published(&pipeline, epoch, end_us)) {
        cycle_diagnostics.published_epoch = epoch;
#if defined(CONFIG_PDO_PHASE_TRACE)
        phase_record_publish(sample, end_us);
#endif
    }
    (void)check_faults(dcu_time_now_us());
}

void dcu_cycle_step(void)
{
    uint32_t start = dcu_time_now_us();
    const bool sm2 = read_handoff(&handoff.sm2_cycles);
    if (runtime.outputs_running && !cycle_configuration_matches())
        pdo_pipeline_fault(&pipeline, PDO_PIPELINE_CONFIG_CHANGED);
    uint32_t count = take(&handoff.sync0_pending);
    cycle_diagnostics.sync0_count += count;
    if (count) {
#if defined(CONFIG_SYNC0_HW_CAPTURE_TEST)
        if (runtime.outputs_running) dcu_sync0_capture_observe(count);
#endif
        if (count > 1 && runtime.outputs_running)
            pdo_pipeline_fault(&pipeline, PDO_PIPELINE_RX_OVERRUN);
        /* SSC Sync and watchdog handlers share this task, never preempt each other. */
        Sync0_Isr();
    }
    if (take(&handoff.sync1_pending))
        Sync1_Isr();

    (void)check_faults(start);
    pdo_pipeline_view_t view;
    if (pdo_pipeline_acquire(&pipeline, &view)) {
        if (runtime.outputs_running && view.epoch == __atomic_load_n(&pipeline.epoch, __ATOMIC_ACQUIRE)) {
            if (!sm2) record_timing(&view);
            const uint32_t cutoff_us = view.sync_us + pipeline.period_us - pipeline.advance_us -
                                       DCU_TX_BUILD_BUDGET_US;
            motor_control_set_neutral_window(sm2 ? 0U : pipeline.period_ns, cutoff_us);
#if defined(CONFIG_PDO_PHASE_TRACE)
            volatile dcu_pdo_phase_sample_t *sample = sm2 ? NULL : phase_record_latch(&view, start);
            uint32_t output_begin_us = sample ? dcu_time_now_us() : 0U;
#endif
            motor_control_output(view.data);
#if defined(CONFIG_PDO_PHASE_TRACE)
            if (sample) {
                uint32_t output_end_us = dcu_time_now_us();
                sample->output_begin_us = output_begin_us;
                sample->output_end_us = output_end_us;
                sample->command_after_release_us = phase_command_after_release(sample, output_begin_us,
                                                                                output_end_us);
            }
#endif
            runtime.pending_epoch = view.epoch;
            runtime.command_pending = true;
            cycle_diagnostics.latched_sequence = view.sequence;
        }
        pdo_pipeline_release(&pipeline, &view);
    }
    if (!sm2) publish_due();
#if defined(CONFIG_PDO_PHASE_TRACE)
    volatile dcu_pdo_phase_sample_t *sample = !sm2 && runtime.command_pending ?
                                             phase_current(runtime.pending_epoch) : NULL;
    uint32_t process_begin_us = sample ? dcu_time_now_us() : 0U;
#endif
    const bool processed = motor_control_process();
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) {
        uint32_t process_end_us = dcu_time_now_us();
        sample->process_us += process_end_us - process_begin_us;
        ++sample->process_calls;
        sample->command_after_release_us += phase_command_after_release(sample, process_begin_us, process_end_us);
        if (processed) sample->command_done_us = process_end_us;
    }
#endif
    if (processed && runtime.command_pending) {
        runtime.processed_epoch = runtime.pending_epoch;
        runtime.command_pending = false;
    }
    publish_due();

    uint32_t ticks = take(&handoff.timer_pending);
    if (ticks > DCU_MAX_DEFERRED_SSC_TICKS) {
        if (runtime.outputs_running)
            pdo_pipeline_fault(&pipeline, PDO_PIPELINE_SYNC_LOST);
        ticks = DCU_MAX_DEFERRED_SSC_TICKS;
    }
    bool update_idle_input = ticks != 0U;
    while (ticks--)
        ECAT_CheckTimer();
    MainLoop();
    service_local_sync_recovery();
    if (!runtime.outputs_running && update_idle_input) {
        motor_control_prepare_input();
        if (runtime.configured && sm2 && bEcatInputUpdateRunning)
            PDO_InputMapping();
#if defined(ESC_EEPROM_EMULATION) && ESC_EEPROM_EMULATION
        /* A Store request deferred in SAFEOP/OP must still reach flash once
         * PREOP/INIT maintenance resumes. The port enforces the state gate. */
        ecat_flash_eeprom_flush();
#endif
    }
    (void)check_faults(dcu_time_now_us());
    uint32_t duration = dcu_time_now_us() - start;
    if (duration > cycle_diagnostics.worker_max_us)
        cycle_diagnostics.worker_max_us = duration;
}

void dcu_cycle_watchdog_isr(void)
{
    /* Independent of task progress; never mutate SSC or motor objects here. */
    const uint32_t now_us = dcu_time_now_us();
    check_sm2_gap(now_us);
    if (__atomic_load_n(&pipeline.enabled, __ATOMIC_ACQUIRE) &&
        pdo_pipeline_check(&pipeline, now_us)) {
        dcu_tx_inhibit();
        for (unsigned c = 0; c < DCU_CAN_BUS_COUNT; ++c)
            dcu_can_cancel_from_isr(c);
    }
}
