#include "cycle_runtime.h"
#include "pdo_pipeline.h"
#include "cycle_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "hpm_gptmr_drv.h"
#include "ecat_def.h"
#include "ecatslv.h"
#include "ecatappl.h"
#include "applInterface.h"
#include "hpm_ecat_hw.h"
#include "ethercat_canfd_gateway.h"
#include "motor_control.h"

#define WORKER_STACK_WORDS 2048U
#define WORKER_TIMER HPM_GPTMR2
#define WORKER_TIMER_CLOCK clock_gptmr2
#define WORKER_TIMER_IRQ IRQn_GPTMR2
#define WORKER_TIMER_CHANNEL 0U
#define TIMEBASE_TIMER HPM_GPTMR1
#define TIMEBASE_CLOCK clock_gptmr1
#define TIMEBASE_CHANNEL 0U

volatile dcu_cycle_diagnostics_t g_dcu_cycle_diagnostics;
static pdo_pipeline_t pipeline __attribute__((section(".fast_ram.bss")));
static StaticTask_t worker_tcb;
static StackType_t worker_stack[WORKER_STACK_WORDS] __attribute__((aligned(16)));
static TaskHandle_t worker_handle;
static uint32_t sync0_pending, sync1_pending, timer_pending;
static uint32_t stage_sequence;
static uint32_t tx_inhibited;
static bool configured, outputs_running;

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
    clock_add_to_group(TIMEBASE_CLOCK, 0);
    hpm_stat_t status = clock_set_source_divider(TIMEBASE_CLOCK, clk_src_osc24m, 24U);
    configASSERT(status == status_success);
    configASSERT(clock_get_frequency(TIMEBASE_CLOCK) == 1000000U);
    gptmr_channel_config_t config;
    gptmr_channel_get_default_config(TIMEBASE_TIMER, &config);
    config.reload = UINT32_MAX;
    config.enable_cmp_output = false;
    gptmr_channel_config(TIMEBASE_TIMER, TIMEBASE_CHANNEL, &config, false);
    gptmr_start_counter(TIMEBASE_TIMER, TIMEBASE_CHANNEL);
}

DCU_FAST_CODE uint32_t dcu_cycle_now_us(void)
{
    /* The SDK inline getter can become an XIP call in a Debug build. */
    return GPTMR_CHANNEL_CNT_COUNTER_GET(TIMEBASE_TIMER->CHANNEL[TIMEBASE_CHANNEL].CNT);
}

bool dcu_cycle_tx_inhibited(void)
{
    return __atomic_load_n(&tx_inhibited, __ATOMIC_ACQUIRE) != 0U;
}

uint16_t dcu_cycle_configure(void)
{
    configured = false;
    if (!bDcSyncActive || sSyncManOutPar.u16SyncType != SYNCTYPE_DCSYNC0 ||
        sSyncManInPar.u16SyncType != SYNCTYPE_DCSYNC0)
        return ALSTATUSCODE_SYNCHRONNOTSUPPORTED;

    uint32_t period_ns = 0;
    HW_EscReadDWord(period_ns, ESC_DC_SYNC0_CYCLETIME_OFFSET);
    period_ns = SWAPDWORD(period_ns);
    if (!dc_configuration_matches(period_ns))
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    intc_m_disable_irq(IRQn_ESC_SYNC0);
    configured = pdo_pipeline_configure(&pipeline, period_ns,
                                       DCU_TX_PUBLISH_ADVANCE_US, nPdOutputSize);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
    g_dcu_cycle_diagnostics.configured_period_ns = configured ? period_ns : 0;
    if (!configured)
        return ALSTATUSCODE_DCSYNC0CYCLETIME;
    motor_control_prepare_input();
    return ALSTATUSCODE_NOERROR;
}

uint16_t dcu_cycle_start_outputs(void)
{
    if (!configured)
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    if (!dc_configuration_matches(pipeline.period_ns))
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    /* A new session must not reuse a command staged before OP. */
    intc_m_disable_irq(IRQn_ESC_SYNC0);
    if (!pdo_pipeline_configure(&pipeline, pipeline.period_ns,
                                 pipeline.advance_us, nPdOutputSize)) {
        configured = false;
        intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
        return ALSTATUSCODE_DCINVALIDSYNCCFG;
    }
    (void)take(&sync0_pending);
    (void)take(&sync1_pending);
    stage_sequence = 0;
    g_dcu_cycle_diagnostics.faults = 0;
    g_dcu_cycle_diagnostics.latched_sequence = 0;
    g_dcu_cycle_diagnostics.published_epoch = 0;
    g_dcu_cycle_diagnostics.worker_max_us = 0;
    outputs_running = true;
    __atomic_store_n(&tx_inhibited, 0U, __ATOMIC_RELEASE);
    pdo_pipeline_enable(&pipeline, true);
    intc_m_enable_irq_with_priority(IRQn_ESC_SYNC0, DCU_IRQ_SYNC0_PRIORITY);
    return ALSTATUSCODE_NOERROR;
}

void dcu_cycle_stop_outputs(void)
{
    pdo_pipeline_enable(&pipeline, false);
    outputs_running = false;
}

void dcu_cycle_stop_inputs(void)
{
    dcu_cycle_stop_outputs();
    configured = false;
    __atomic_store_n(&tx_inhibited, 0U, __ATOMIC_RELEASE);
}

void dcu_cycle_stage(const void *data)
{
    (void)pdo_pipeline_stage(&pipeline, data, nPdOutputSize, ++stage_sequence);
}

DCU_FAST_CODE void dcu_cycle_sync0_isr(void)
{
    pdo_pipeline_sync(&pipeline, dcu_cycle_now_us());
    __atomic_fetch_add(&sync0_pending, 1U, __ATOMIC_RELEASE);
}

void dcu_cycle_sync1_isr(void)
{
    __atomic_fetch_add(&sync1_pending, 1U, __ATOMIC_RELEASE);
}

void dcu_cycle_timer_isr(void)
{
    __atomic_fetch_add(&timer_pending, 1U, __ATOMIC_RELEASE);
}

static bool check_faults(uint32_t now_us)
{
    uint32_t faults = pdo_pipeline_check(&pipeline, now_us);
    if (!outputs_running || !faults)
        return false;
    g_dcu_cycle_diagnostics.faults |= faults;
    __atomic_store_n(&tx_inhibited, 1U, __ATOMIC_RELEASE);
    dcu_cycle_stop_outputs();
    motor_control_cycle_fault();
    ECAT_StateChange(STATE_SAFEOP, ALSTATUSCODE_SYNCERROR);
    return true;
}

static void publish_due(void)
{
    uint32_t now_us = ethercat_canfd_now_us();
    if (check_faults(now_us) || !pdo_pipeline_publish_due(&pipeline, now_us))
        return;
    uint32_t epoch = __atomic_load_n(&pipeline.epoch, __ATOMIC_ACQUIRE);
    motor_control_prepare_input();
    PDO_InputMapping();
    if (pdo_pipeline_mark_published(&pipeline, epoch, ethercat_canfd_now_us()))
        g_dcu_cycle_diagnostics.published_epoch = epoch;
    (void)check_faults(ethercat_canfd_now_us());
}

static void run_once(void)
{
    uint32_t start = ethercat_canfd_now_us();
    if (outputs_running && !dc_configuration_matches(pipeline.period_ns))
        pdo_pipeline_fault(&pipeline, PDO_PIPELINE_CONFIG_CHANGED);
    uint32_t count = take(&sync0_pending);
    g_dcu_cycle_diagnostics.sync0_count += count;
    if (count) {
        if (count > 1 && outputs_running)
            pdo_pipeline_fault(&pipeline, PDO_PIPELINE_RX_OVERRUN);
        /* SSC Sync and watchdog handlers share this task, never preempt each other. */
        Sync0_Isr();
    }
    if (take(&sync1_pending))
        Sync1_Isr();

    (void)check_faults(start);
    pdo_pipeline_view_t view;
    if (pdo_pipeline_acquire(&pipeline, &view)) {
        if (outputs_running && view.epoch == __atomic_load_n(&pipeline.epoch, __ATOMIC_ACQUIRE)) {
            motor_control_output(view.data);
            g_dcu_cycle_diagnostics.latched_sequence = view.sequence;
        }
        pdo_pipeline_release(&pipeline, &view);
    }
    publish_due();
    motor_control_process();
    publish_due();

    uint32_t ticks = take(&timer_pending);
    if (ticks > 10U) {
        if (outputs_running)
            pdo_pipeline_fault(&pipeline, PDO_PIPELINE_SYNC_LOST);
        ticks = 10U;
    }
    bool update_idle_input = ticks != 0U;
    while (ticks--)
        ECAT_CheckTimer();
    MainLoop();
    if (!outputs_running && update_idle_input) {
        motor_control_prepare_input();
#if defined(ESC_EEPROM_EMULATION) && ESC_EEPROM_EMULATION
        /* A Store request deferred in SAFEOP/OP must still reach flash once
         * PREOP/INIT maintenance resumes. The port enforces the state gate. */
        ecat_flash_eeprom_flush();
#endif
    }
    (void)check_faults(ethercat_canfd_now_us());
    uint32_t duration = ethercat_canfd_now_us() - start;
    if (duration > g_dcu_cycle_diagnostics.worker_max_us)
        g_dcu_cycle_diagnostics.worker_max_us = duration;
}

SDK_DECLARE_EXT_ISR_M(WORKER_TIMER_IRQ, dcu_worker_timer_isr)
void dcu_worker_timer_isr(void)
{
    if (gptmr_check_status(WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(WORKER_TIMER_CHANNEL))) {
        gptmr_clear_status(WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(WORKER_TIMER_CHANNEL));
        /* Independent of task progress: inhibit new TX and cancel queued
         * requests on a missed cycle, without mutating motor/SSC objects. */
        if (__atomic_load_n(&pipeline.enabled, __ATOMIC_ACQUIRE) &&
            pdo_pipeline_check(&pipeline, dcu_cycle_now_us())) {
            __atomic_store_n(&tx_inhibited, 1U, __ATOMIC_RELEASE);
            for (unsigned c = 0; c < CANFD_GATEWAY_BUS_COUNT; ++c)
                ethercat_canfd_cancel(c);
        }
        BaseType_t wake = pdFALSE;
        vTaskNotifyGiveFromISR(worker_handle, &wake);
        portYIELD_FROM_ISR(wake);
    }
}

static void worker(void *argument)
{
    (void)argument;
    gptmr_channel_config_t config;
    clock_add_to_group(WORKER_TIMER_CLOCK, 0);
    gptmr_channel_get_default_config(WORKER_TIMER, &config);
    config.enable_cmp_output = false;
    config.reload = clock_get_frequency(WORKER_TIMER_CLOCK) / (1000000U / DCU_WORKER_PERIOD_US);
    gptmr_channel_config(WORKER_TIMER, WORKER_TIMER_CHANNEL, &config, false);
    gptmr_clear_status(WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(WORKER_TIMER_CHANNEL));
    gptmr_enable_irq(WORKER_TIMER, GPTMR_CH_RLD_IRQ_MASK(WORKER_TIMER_CHANNEL));
    intc_m_enable_irq_with_priority(WORKER_TIMER_IRQ, DCU_IRQ_WORKER_PRIORITY);
    gptmr_start_counter(WORKER_TIMER, WORKER_TIMER_CHANNEL);
    for (;;) {
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        run_once();
        static uint32_t last_stack_check_ms;
        uint32_t now_ms = ethercat_canfd_now_ms();
        if ((uint32_t)(now_ms - last_stack_check_ms) >= 1000U) {
            last_stack_check_ms = now_ms;
            g_dcu_cycle_diagnostics.worker_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        }
    }
}

void dcu_rtos_start(void)
{
    worker_handle = xTaskCreateStatic(worker, "ecat-can", WORKER_STACK_WORDS, NULL,
                                      configMAX_PRIORITIES - 1U, worker_stack, &worker_tcb);
    configASSERT(worker_handle != NULL);
    vTaskStartScheduler();
    dcu_rtos_assert(__FILE__, __LINE__);
}

void dcu_rtos_assert(const char *file, unsigned long line)
{
    g_dcu_cycle_diagnostics.fatal_file = file;
    g_dcu_cycle_diagnostics.fatal_line = line;
    /* No allocation, logging or motor-state mutation in a possibly corrupt context. */
    (void)disable_global_irq(CSR_MSTATUS_MIE_MASK);
    for (unsigned c = 0; c < CANFD_GATEWAY_BUS_COUNT; ++c)
        ethercat_canfd_cancel(c);
    for (;;)
        __asm volatile("nop");
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    dcu_rtos_assert(name, __LINE__);
}
