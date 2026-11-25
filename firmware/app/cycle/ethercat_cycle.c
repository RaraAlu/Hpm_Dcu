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
#include "first_pdo_trace.h"
#if defined(CONFIG_SYNC0_HW_CAPTURE_TEST)
#include "sync0_hw_capture.h"
#endif
#if defined(CONFIG_PDO_PHASE_TRACE)
#include "detail/pdo_phase_trace.h"
#include "encos_interface.h"
#endif
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
#include "detail/first_pdo_trace_state.h"
#include "encos_interface.h"
#endif
#include <string.h>

static volatile dcu_cycle_diagnostics_t cycle_diagnostics;
/* Task-owned timing statistics, read via the debugger only after stopping. */
static volatile dcu_cycle_timing_t cycle_timing;
static volatile dcu_cyclic_latency_t cyclic_latency __attribute__((section(".noncacheable.bss")));
static pdo_pipeline_t pipeline __attribute__((section(".fast_ram.bss")));

/* These fields alone cross ISR/task ownership; every access is atomic. */
static dcu_cycle_handoff_t handoff;
static dcu_cycle_runtime_t runtime;
static dcu_raw_pdo_t raw_pdo;

#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
static volatile dcu_first_pdo_trace_t can2_first_pdo_trace;

bool dcu_first_pdo_trace_active(void)
{
    return __atomic_load_n(&can2_first_pdo_trace.sequence, __ATOMIC_ACQUIRE) &&
           dcu_time_now_us() - can2_first_pdo_trace.armed_us <= DCU_FIRST_PDO_CAPTURE_US;
}

void dcu_first_pdo_trace_at(uint32_t time_us, uint32_t event, uint32_t a, uint32_t b, uint32_t c, uint32_t d)
{
    if (!__atomic_load_n(&can2_first_pdo_trace.sequence, __ATOMIC_ACQUIRE) ||
        time_us - can2_first_pdo_trace.armed_us > DCU_FIRST_PDO_CAPTURE_US) return;
    const uint32_t n = __atomic_fetch_add(&can2_first_pdo_trace.count, 1U, __ATOMIC_RELAXED);
    if (n < DCU_FIRST_PDO_EVENT_LIMIT)
        can2_first_pdo_trace.entry[n] = (dcu_first_pdo_trace_entry_t){time_us, event, a, b, c, d};
}

static void trace_first_sm2(const void *data, uint32_t stage_done_us, uint32_t read_end_us)
{
    if (nPdOutputSize < sizeof(mc_board_rx_t)) return;
    mc_board_rx_t board;
    memcpy(&board, data, sizeof(board));
    if (!__atomic_load_n(&can2_first_pdo_trace.sequence, __ATOMIC_ACQUIRE)) {
        if (!board.CycleSeq || board.InterfaceVersion != MC_INTERFACE_VERSION || board.BoardControl != 1U ||
            board.ChannelEnableMask != 4U || board.AxisEnableMask != (15U << 14)) return;
        const mc_axis_rx_t zero = {.ControlWord = MC_AXIS_ARM | MC_AXIS_START_MOTION, .Mode = 4};
        for (unsigned i = 0; i < 4; ++i) {
            const unsigned offset = motor_control_command_offset(14U + i);
            if (offset + sizeof(zero) > nPdOutputSize ||
                memcmp((const uint8_t *)data + offset, &zero, sizeof(zero))) return;
        }
        can2_first_pdo_trace.armed_us = stage_done_us;
        __atomic_store_n(&can2_first_pdo_trace.sequence, board.CycleSeq, __ATOMIC_RELEASE);
    }
    DCU_FIRST_TRACE_AT(stage_done_us, DCU_FIRST_SM2, board.CycleSeq,
                       __atomic_load_n(&handoff.stage_start_us, __ATOMIC_ACQUIRE), read_end_us, board.BoardControl);
}
#endif

#if defined(CONFIG_PDO_PHASE_TRACE)
static volatile dcu_pdo_phase_trace_t pdo_phase_trace;

static uint32_t sync_retx_baseline;
static uint32_t sample_retx_baseline;

static uint32_t phase_le32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static volatile dcu_pdo_phase_sample_t *phase_record_latch(const pdo_pipeline_view_t *view,
                                                           uint32_t worker_us)
{
    const uint32_t baseline = __atomic_load_n(&sync_retx_baseline, __ATOMIC_ACQUIRE);
    if (pdo_phase_trace.count) {
        volatile dcu_pdo_phase_sample_t *prev =
            &pdo_phase_trace.sample[(pdo_phase_trace.count - 1U) % DCU_PDO_PHASE_SAMPLES];
        if (view->epoch - prev->epoch == 1U) prev->retx_count = baseline - sample_retx_baseline;
    }
    sample_retx_baseline = baseline;
    uint32_t i = pdo_phase_trace.count++ % DCU_PDO_PHASE_SAMPLES;
    volatile dcu_pdo_phase_sample_t *sample = &pdo_phase_trace.sample[i];
    *sample = (dcu_pdo_phase_sample_t){.sequence = phase_le32(view->data), .epoch = view->epoch,
        .last_consumed_offset_from_sync0 = UINT32_MAX,
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

void dcu_trace_neutral_consumed(uint32_t consumed_us)
{
    volatile dcu_pdo_phase_sample_t *sample = phase_current(__atomic_load_n(&pipeline.epoch, __ATOMIC_ACQUIRE));
    if (sample && runtime.outputs_running && !sample->published_us)
        sample->last_consumed_offset_from_sync0 = consumed_us - sample->sync0_us;
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
    // 单帧档没有 AcceptedCycleSeq，偏移 4 是 ValidMask。
    if (wire && capacity >= 8U && !motor_control_event_pdo())
        sample->accepted = phase_le32(wire + 4);
    sample->published_us = end_us;
    sample->retx_count = dcu_can_observed_retx_count() - sample_retx_baseline;
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

static DCU_FAST_CODE void raw_pdo_fault(void)
{
    if (__atomic_exchange_n(&raw_pdo.fault, 1U, __ATOMIC_ACQ_REL)) return;
    dcu_tx_inhibit();
    const uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    for (unsigned c = 0; c < DCU_CAN_BUS_COUNT; ++c) dcu_can_cancel_from_isr(c);
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
}

static DCU_FAST_CODE uint32_t *raw_pdo_reserve(uint16_t size, uint32_t *write)
{
    if (!read_handoff(&raw_pdo.enabled) || size != DCU_SINGLE_FD_PDO_BYTES) return NULL;
    *write = read_handoff(&raw_pdo.write);
    if (*write - read_handoff(&raw_pdo.read) >= DCU_RAW_PDO_QUEUE_DEPTH) {
        raw_pdo_fault();
        return NULL;
    }
    return raw_pdo.image[*write % DCU_RAW_PDO_QUEUE_DEPTH];
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
    raw_pdo.configured = false;
    const bool sm2 = sm2_sync_selected();
    const bool single_fd = motor_control_event_pdo();
    if (single_fd && (nPdOutputSize != DCU_SINGLE_FD_PDO_BYTES || nPdInputSize != DCU_SINGLE_FD_PDO_BYTES))
        return ALSTATUSCODE_SYNCHRONNOTSUPPORTED;
    /* Only an explicitly SM-synchronous master uses the legacy event queue.
     * SingleFd with DC uses the same latch, watchdog and cutoff as motor PDOs. */
    if (single_fd && sm2) {
        if (!sm2_configuration_matches())
            return ALSTATUSCODE_SYNCHRONNOTSUPPORTED;
        write_handoff(&handoff.sm2_cycles, 0U);
        raw_pdo.configured = runtime.configured = true;
        raw_pdo.published = false;
        cycle_diagnostics.configured_period_ns = 0;
        motor_control_prepare_input();
        return ALSTATUSCODE_NOERROR;
    }
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
    if (raw_pdo.configured) {
        if (!sm2_configuration_matches()) return ALSTATUSCODE_DCINVALIDSYNCCFG;
        const uint32_t lease = motor_control_event_lease_us();
        if (!lease) return ALSTATUSCODE_DCINVALIDSYNCCFG;
        write_handoff(&raw_pdo.read, 0);
        write_handoff(&raw_pdo.write, 0);
        write_handoff(&raw_pdo.fault, 0);
        write_handoff(&raw_pdo.worker_us, dcu_time_now_us());
        write_handoff(&raw_pdo.lease_us, lease);
        raw_pdo.published = false;
        runtime.command_pending = false;
        runtime.outputs_running = true;
        motor_control_set_neutral_window(0, 0);
        dcu_tx_allow();
        write_handoff(&raw_pdo.enabled, 1);
        return ALSTATUSCODE_NOERROR;
    }
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
    runtime.preview_sequence = 0U;
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
    cycle_diagnostics.missing_can_devices = 0;
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
    __atomic_store_n(&can2_first_pdo_trace.sequence, 0U, __ATOMIC_RELEASE);
    can2_first_pdo_trace.count = 0;
#endif
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
    pipeline.ordered_outputs = motor_control_event_pdo();
    raw_pdo.published = false;
    motor_control_set_neutral_window(0, 0);
    dcu_tx_allow();
    pipeline.stage_clock_us = dcu_time_now_us;
    cyclic_latency = (dcu_cyclic_latency_t){.tick_hz = dcu_time_tick_hz()};
    if (!read_handoff(&handoff.sm2_cycles)) pipeline.latch_clock_ticks = dcu_time_ticks;
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
    write_handoff(&raw_pdo.enabled, 0);
    raw_pdo.published = false;
    pdo_pipeline_enable(&pipeline, false);
    if (runtime.outputs_running && !raw_pdo.configured) {
        LatchInputSync0Value = runtime.ssc_sync0_latch;
        LatchInputSync0Counter = 0U;
    }
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
    if (runtime.outputs_running && dcu_first_pdo_trace_active()) {
        const uint32_t before = dcu_time_now_us();
        uint32_t dc_low;
        HW_EscReadDWord(dc_low, ESC_SYSTEMTIME_OFFSET);
        DCU_FIRST_TRACE_AT(before, DCU_FIRST_DC_ANCHOR, SWAPDWORD(dc_low), dcu_time_now_us(), 0, 0);
    }
#endif
    runtime.outputs_running = false;
    runtime.command_pending = false;
}

void dcu_cycle_stop_inputs(void)
{
    dcu_cycle_stop_outputs();
    runtime.configured = false;
    raw_pdo.configured = false;
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
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
        trace_first_sm2(data, done_us, direct_read_end_us);
#endif
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
    if (read_handoff(&raw_pdo.enabled)) {
        uint32_t write;
        uint32_t *destination = raw_pdo_reserve(nPdOutputSize, &write);
        if (destination) {
            memcpy(destination, data, DCU_SINGLE_FD_PDO_BYTES);
            write_handoff(&raw_pdo.write, write + 1U);
        }
        return;
    }
    if (!read_handoff(&pipeline.enabled)) return;
    uint32_t sequence = stage_begin();
    stage_finish(pdo_pipeline_stage(&pipeline, data, nPdOutputSize, sequence), data, false, 0U);
}

DCU_FAST_CODE bool dcu_cycle_stage_read(uint16_t address, uint16_t size,
                                        uint32_t (*read_output)(void *, uint16_t, uint16_t))
{
    if (read_handoff(&raw_pdo.enabled)) {
        uint32_t write;
        uint32_t *destination = raw_pdo_reserve(size, &write);
        if (!destination) return false;
        (void)read_output(destination, address, size);
        write_handoff(&raw_pdo.write, write + 1U);
        return true;
    }
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
#if defined(CONFIG_PDO_PHASE_TRACE)
    __atomic_store_n(&sync_retx_baseline, dcu_can_observed_retx_count(), __ATOMIC_RELEASE);
#endif
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
    if (faults & PDO_PIPELINE_CAN_FEEDBACK_MISSING)
        cycle_diagnostics.missing_can_devices = read_handoff(&pipeline.cyclic_missing);
    DCU_FIRST_TRACE_AT(now_us, DCU_FIRST_WORKER_FAULT, faults, read_handoff(&pipeline.epoch),
                       cycle_diagnostics.fault_last_stage_us, nAlStatus);
#if defined(CONFIG_PDO_PHASE_TRACE)
    pdo_phase_trace.fault_epoch = read_handoff(&pipeline.epoch);
    volatile dcu_pdo_phase_sample_t *sample = phase_current(pdo_phase_trace.fault_epoch);
    if (sample) sample->retx_count = dcu_can_observed_retx_count() - sample_retx_baseline;
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
    if (read_handoff(&pipeline.cyclic_active))
        motor_control_fault_cyclic_feedback((faults & PDO_PIPELINE_CAN_FEEDBACK_MISSING) ?
                                            cycle_diagnostics.missing_can_devices : 0U);
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
    const bool cyclic = !sm2 && read_handoff(&pipeline.cyclic_active) && read_handoff(&pipeline.cyclic_epoch) == epoch;
    const uint32_t deadline = cyclic ? pdo_pipeline_cyclic_deadline(&pipeline, sync_us) :
                                    sync_us + pipeline.period_us - pipeline.advance_us;
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
    /* Do not pop another raw frame before the master has read the last image. */
    if (pipeline.ordered_outputs && raw_pdo.published &&
        !(SWAPWORD(HW_GetALEventRegister()) & PROCESS_INPUT_EVENT)) {
        if (pdo_pipeline_mark_published(&pipeline, epoch, dcu_time_now_us()))
            cycle_diagnostics.published_epoch = epoch;
        return;
    }
    uint32_t offset_us = now_us - sync_us;
    uint32_t release_us = pipeline.period_us - pipeline.advance_us - DCU_TX_BUILD_BUDGET_US;
    if (offset_us >= release_us && offset_us - release_us > cycle_diagnostics.publish_start_delay_max_us)
        cycle_diagnostics.publish_start_delay_max_us = offset_us - release_us;
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) sample->publish_begin_us = now_us;
#endif
    DCU_FIRST_TRACE_AT(now_us, DCU_FIRST_SM3_BEGIN, epoch, sync_us,
                       deadline, runtime.processed_epoch);
    motor_control_prepare_input();
    uint32_t packed_us = dcu_time_now_us();
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
    if (dcu_first_pdo_trace_active()) {
        uint16_t capacity;
        const uint8_t *wire = dcu_ssc_txpdo_buffer(&capacity);
        mc_board_tx_t header = {0};
        if (wire && capacity >= sizeof(header)) memcpy(&header, wire, sizeof(header));
        DCU_FIRST_TRACE_AT(packed_us, DCU_FIRST_SM3_PACKED, epoch, header.PublishSeq, header.AcceptedCycleSeq,
                           header.BoardStatus);
    }
#endif
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
    uint32_t elapsed_ticks = 0;
    if (cyclic) {
        const uint32_t latch_ticks = read_handoff(&pipeline.latch_ticks);
        uint16_t capacity;
        const void *image = dcu_ssc_txpdo_buffer(&capacity);
        if (capacity < nPdInputSize || !hw_esc_publish_pdo_before(nEscAddrInputData, image, nPdInputSize, deadline)) {
            pdo_pipeline_fault(&pipeline, PDO_PIPELINE_TX_LATE);
            (void)check_faults(dcu_time_now_us());
            return;
        }
        elapsed_ticks = dcu_time_ticks() - latch_ticks;
    } else {
        PDO_InputMapping();
        if (pipeline.ordered_outputs) raw_pdo.published = true;
    }
    uint32_t end_us = dcu_time_now_us();
#if defined(CONFIG_PDO_PHASE_TRACE)
    if (sample) sample->esc_end_us = end_us;
#endif
    if (end_us - packed_us > cycle_diagnostics.esc_write_max_us)
        cycle_diagnostics.esc_write_max_us = end_us - packed_us;
    if (end_us - now_us > cycle_diagnostics.publish_max_us)
        cycle_diagnostics.publish_max_us = end_us - now_us;
    const bool marked = pdo_pipeline_mark_published(&pipeline, epoch, end_us);
    DCU_FIRST_TRACE_AT(end_us, DCU_FIRST_SM3_WRITTEN, epoch, packed_us, 0, 0);
    DCU_FIRST_TRACE(DCU_FIRST_SM3_MARK, epoch, marked, read_handoff(&pipeline.faults), end_us);
    if (marked) {
        if (cyclic) {
            if (!cyclic_latency.count) {
                cyclic_latency.first_epoch = epoch;
                cyclic_latency.min_ticks = elapsed_ticks;
            }
            if (elapsed_ticks < cyclic_latency.min_ticks) cyclic_latency.min_ticks = elapsed_ticks;
            if (elapsed_ticks > cyclic_latency.max_ticks) cyclic_latency.max_ticks = elapsed_ticks;
            cyclic_latency.last_epoch = epoch;
            cyclic_latency.last_sync_us = sync_us;
            cyclic_latency.last_end_us = end_us;
            ++cyclic_latency.count;
        }
        cycle_diagnostics.published_epoch = epoch;
#if defined(CONFIG_PDO_PHASE_TRACE)
        phase_record_publish(sample, end_us);
#endif
    }
    (void)check_faults(dcu_time_now_us());
}

static void raw_pdo_step(void)
{
    if (runtime.outputs_running && !sm2_configuration_matches()) raw_pdo_fault();
    if (runtime.outputs_running && take(&raw_pdo.fault)) {
        dcu_cycle_stop_outputs();
        motor_control_cycle_fault();
        ECAT_StateChange(STATE_SAFEOP, ALSTATUSCODE_UNSPECIFIEDERROR);
    }
    uint32_t read = read_handoff(&raw_pdo.read);
    if (runtime.outputs_running && !runtime.command_pending && read != read_handoff(&raw_pdo.write)) {
        motor_control_output(raw_pdo.image[read % DCU_RAW_PDO_QUEUE_DEPTH]);
        write_handoff(&raw_pdo.read, read + 1U);
        runtime.command_pending = true;
    }
    if (motor_control_process()) runtime.command_pending = false;
    write_handoff(&raw_pdo.worker_us, dcu_time_now_us());
    /* The previous image may contain a CAN frame. Retire it only after SM3 was read. */
    if (bEcatInputUpdateRunning &&
        (!raw_pdo.published || (SWAPWORD(HW_GetALEventRegister()) & PROCESS_INPUT_EVENT))) {
        motor_control_prepare_input();
        PDO_InputMapping();
        raw_pdo.published = true;
    }
}

static void preview_ready_sm2(void)
{
    static uint8_t image[PDO_PIPELINE_RX_MAX] __attribute__((aligned(4)));
    if (!runtime.outputs_running || read_handoff(&handoff.sm2_cycles) ||
        read_handoff(&pipeline.ready) >= PDO_PIPELINE_SLOT_COUNT ||
        runtime.preview_sequence == read_handoff(&handoff.stage_sequence)) return;
    // Only the PDI producer can recycle or write the image. Sync0 remains unmasked.
    intc_m_disable_irq(IRQn_ESC);
    const uint32_t index = read_handoff(&pipeline.ready);
    if (index < PDO_PIPELINE_SLOT_COUNT) {
        runtime.preview_sequence = pipeline.slot[index].sequence;
        memcpy(image, pipeline.slot[index].data, pipeline.output_size);
    }
    intc_m_enable_irq_with_priority(IRQn_ESC, DCU_IRQ_PDI_PRIORITY);
    if (index < PDO_PIPELINE_SLOT_COUNT) motor_control_preview(image);
}

void dcu_cycle_step(void)
{
    uint32_t start = dcu_time_now_us();
    uint32_t ticks;
    if (raw_pdo.configured) {
        raw_pdo_step();
        goto ssc;
    }
    const bool sm2 = read_handoff(&handoff.sm2_cycles);
    const uint32_t current_epoch = read_handoff(&pipeline.epoch);
    if (!sm2 && runtime.outputs_running && read_handoff(&pipeline.cyclic_active) &&
        read_handoff(&pipeline.cyclic_epoch) == current_epoch && !read_handoff(&pipeline.published)) {
        if (!check_faults(start)) {
            (void)motor_control_process();
            pdo_pipeline_update_cyclic(&pipeline, current_epoch, motor_control_cyclic_feedback_missing());
            publish_due();
        }
        if (runtime.outputs_running && !read_handoff(&pipeline.published)) return;
        goto ssc;
    }
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
            motor_control_set_cyclic_feedback_window(sm2 ? 0U : pipeline.period_us,
                                                    pdo_pipeline_cyclic_deadline(&pipeline, view.sync_us));
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
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
            uint32_t sequence;
            memcpy(&sequence, view.data, sizeof(sequence));
            DCU_FIRST_TRACE(DCU_FIRST_LATCH, sequence, view.epoch, view.sync_us, view.ready_us);
#endif
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
        if (!sm2 && motor_control_cyclic_feedback_required())
            pdo_pipeline_update_cyclic(&pipeline, runtime.processed_epoch, motor_control_cyclic_feedback_missing());
    }
    publish_due();
    if (!sm2 && runtime.outputs_running && read_handoff(&pipeline.cyclic_active) &&
        read_handoff(&pipeline.cyclic_epoch) == read_handoff(&pipeline.epoch) && !read_handoff(&pipeline.published)) return;

ssc:
    if (!raw_pdo.configured) preview_ready_sm2();
    ticks = take(&handoff.timer_pending);
    if (ticks > DCU_MAX_DEFERRED_SSC_TICKS) {
        if (runtime.outputs_running && !raw_pdo.configured)
            pdo_pipeline_fault(&pipeline, PDO_PIPELINE_SYNC_LOST);
        ticks = DCU_MAX_DEFERRED_SSC_TICKS;
    }
    bool update_idle_input = ticks != 0U;
    while (ticks--)
        ECAT_CheckTimer();
    MainLoop();
    service_local_sync_recovery();
    if (!runtime.outputs_running && update_idle_input) {
        if (!raw_pdo.configured) motor_control_prepare_input();
        if (runtime.configured && !raw_pdo.configured && read_handoff(&handoff.sm2_cycles) && bEcatInputUpdateRunning)
            PDO_InputMapping();
#if defined(ESC_EEPROM_EMULATION) && ESC_EEPROM_EMULATION
        /* A Store request deferred in SAFEOP/OP must still reach flash once
         * PREOP/INIT maintenance resumes. The port enforces the state gate. */
        ecat_flash_eeprom_flush();
#endif
    }
    if (!raw_pdo.configured) (void)check_faults(dcu_time_now_us());
    uint32_t duration = dcu_time_now_us() - start;
    if (duration > cycle_diagnostics.worker_max_us)
        cycle_diagnostics.worker_max_us = duration;
}

DCU_FAST_CODE bool dcu_cyclic_tx_window_current(uint32_t cutoff_us)
{
    return read_handoff(&pipeline.enabled) && !read_handoff(&pipeline.faults) &&
           !read_handoff(&pipeline.published) &&
           pdo_pipeline_cyclic_deadline(&pipeline, read_handoff(&pipeline.sync_us)) == cutoff_us;
}

void dcu_cycle_watchdog_isr(void)
{
    /* Independent of task progress; never mutate SSC or motor objects here. */
    const uint32_t now_us = dcu_time_now_us();
    if (read_handoff(&raw_pdo.enabled)) {
        if (now_us - read_handoff(&raw_pdo.worker_us) >= read_handoff(&raw_pdo.lease_us))
            raw_pdo_fault();
        return;
    }
    check_sm2_gap(now_us);
    if (__atomic_load_n(&pipeline.enabled, __ATOMIC_ACQUIRE) &&
        pdo_pipeline_check(&pipeline, now_us)) {
        DCU_FIRST_TRACE_AT(now_us, DCU_FIRST_WATCHDOG, read_handoff(&pipeline.faults),
                           read_handoff(&pipeline.epoch), read_handoff(&pipeline.published),
                           read_handoff(&pipeline.published_epoch));
        dcu_tx_inhibit();
        for (unsigned c = 0; c < DCU_CAN_BUS_COUNT; ++c)
            dcu_can_cancel_from_isr(c);
    }
}
