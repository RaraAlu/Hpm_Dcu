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
#include "device_config.h"
#include "hpm_ecat_hw.h"
#include "motor_ethercat.h"
#include "pdo_pipeline.h"
#include "realtime_port.h"

static volatile dcu_cycle_diagnostics_t cycle_diagnostics;
static pdo_pipeline_t pipeline __attribute__((section(".fast_ram.bss")));

/* These fields alone cross ISR/task ownership; every access is atomic. */
static dcu_cycle_handoff_t handoff;
static dcu_cycle_runtime_t runtime;

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

void dcu_cycle_init(void)
{
    (void)pdo_pipeline_configure(&pipeline, 0, 0, 0);
}

uint16_t dcu_cycle_configure(void)
{
    if (runtime.outputs_running)
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    runtime.configured = false;
    if (!bDcSyncActive || sSyncManOutPar.u16SyncType != SYNCTYPE_DCSYNC0 ||
        sSyncManInPar.u16SyncType != SYNCTYPE_DCSYNC0)
        return ALSTATUSCODE_SYNCHRONNOTSUPPORTED;

    uint32_t period_ns = 0;
    HW_EscReadDWord(period_ns, ESC_DC_SYNC0_CYCLETIME_OFFSET);
    period_ns = SWAPDWORD(period_ns);
    if (!dc_configuration_matches(period_ns))
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    intc_m_disable_irq(IRQn_ESC_SYNC0);
    runtime.configured = pdo_pipeline_configure(&pipeline, period_ns,
                                                DCU_TX_PUBLISH_ADVANCE_US, nPdOutputSize);
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
    if (!dc_configuration_matches(pipeline.period_ns))
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
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
    __atomic_store_n(&handoff.stage_sequence, 0U, __ATOMIC_RELAXED);
    cycle_diagnostics.faults = 0;
    cycle_diagnostics.latched_sequence = 0;
    cycle_diagnostics.published_epoch = 0;
    cycle_diagnostics.worker_max_us = 0;
    runtime.outputs_running = true;
    dcu_tx_allow();
    pdo_pipeline_enable(&pipeline, true);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
    return ALSTATUSCODE_NOERROR;
}

void dcu_cycle_stop_outputs(void)
{
    pdo_pipeline_enable(&pipeline, false);
    runtime.outputs_running = false;
}

void dcu_cycle_stop_inputs(void)
{
    dcu_cycle_stop_outputs();
    runtime.configured = false;
    dcu_tx_allow();
}

void dcu_cycle_stage(const void *data)
{
    uint32_t sequence = __atomic_add_fetch(&handoff.stage_sequence, 1U, __ATOMIC_RELAXED);
    (void)pdo_pipeline_stage(&pipeline, data, nPdOutputSize, sequence);
}

DCU_FAST_CODE void dcu_cycle_sync0_isr(void)
{
    pdo_pipeline_sync(&pipeline, dcu_time_now_us());
    __atomic_fetch_add(&handoff.sync0_pending, 1U, __ATOMIC_RELEASE);
}

void dcu_cycle_sync1_isr(void)
{
    __atomic_fetch_add(&handoff.sync1_pending, 1U, __ATOMIC_RELEASE);
}

void dcu_cycle_timer_isr(void)
{
    __atomic_fetch_add(&handoff.timer_pending, 1U, __ATOMIC_RELEASE);
}

static bool check_faults(uint32_t now_us)
{
    uint32_t faults = pdo_pipeline_check(&pipeline, now_us);
    if (!runtime.outputs_running || !faults)
        return false;
    cycle_diagnostics.faults |= faults;
    dcu_tx_inhibit();
    dcu_cycle_stop_outputs();
    motor_control_cycle_fault();
    ECAT_StateChange(STATE_SAFEOP, ALSTATUSCODE_SYNCERROR);
    return true;
}

static void publish_due(void)
{
    uint32_t now_us = dcu_time_now_us();
    if (check_faults(now_us) || !pdo_pipeline_publish_due(&pipeline, now_us))
        return;
    uint32_t epoch = __atomic_load_n(&pipeline.epoch, __ATOMIC_ACQUIRE);
    motor_control_prepare_input();
    PDO_InputMapping();
    if (pdo_pipeline_mark_published(&pipeline, epoch, dcu_time_now_us()))
        cycle_diagnostics.published_epoch = epoch;
    (void)check_faults(dcu_time_now_us());
}

void dcu_cycle_step(void)
{
    uint32_t start = dcu_time_now_us();
    if (runtime.outputs_running && !dc_configuration_matches(pipeline.period_ns))
        pdo_pipeline_fault(&pipeline, PDO_PIPELINE_CONFIG_CHANGED);
    uint32_t count = take(&handoff.sync0_pending);
    cycle_diagnostics.sync0_count += count;
    if (count) {
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
            motor_control_output(view.data);
            cycle_diagnostics.latched_sequence = view.sequence;
        }
        pdo_pipeline_release(&pipeline, &view);
    }
    publish_due();
    motor_control_process();
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
    if (!runtime.outputs_running && update_idle_input) {
        motor_control_prepare_input();
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
    if (__atomic_load_n(&pipeline.enabled, __ATOMIC_ACQUIRE) &&
        pdo_pipeline_check(&pipeline, dcu_time_now_us())) {
        dcu_tx_inhibit();
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            dcu_can_cancel(c);
    }
}
