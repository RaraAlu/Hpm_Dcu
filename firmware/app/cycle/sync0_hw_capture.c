#include "sync0_hw_capture.h"
#include "board.h"
#include "cfg/sync0_hw_capture_config.h"
#include "detail/sync0_hw_capture_state.h"
#include "ecat_config.h"
#include "hpm_gptmr_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"

_Static_assert(DCU_SYNC0_CAPTURE_CHANNEL != ECAT_TIMER_GPTMR_CH,
               "Sync0 capture must not use the SSC timer channel");

/* Only the EtherCAT worker updates these; read after leaving OP. */
static volatile dcu_sync0_capture_stats_t sync0_capture_stats;
static uint32_t capture_clock_hz;
static uint64_t lower_bound, upper_bound;

void dcu_sync0_capture_init(void)
{
    GPTMR_Type *timer = DCU_SYNC0_CAPTURE_TIMER;
    const unsigned channel = DCU_SYNC0_CAPTURE_CHANNEL;
    if (timer->CHANNEL[channel].CR != 0U ||
        (timer->IRQEN & (0xFU << (4U * channel))) ||
        HPM_TRGM0->TRGOCFG[DCU_SYNC0_CAPTURE_ROUTE] != 0U)
        return;

    const uint32_t hz = clock_get_frequency(DCU_SYNC0_CAPTURE_CLOCK);
    if (hz < DCU_SYNC0_CAPTURE_MIN_CLOCK_HZ) return;
    gptmr_channel_config_t timer_config;
    gptmr_channel_get_default_config(timer, &timer_config);
    timer_config.mode = gptmr_work_mode_capture_at_rising_edge;
    timer_config.enable_cmp_output = false;
    timer_config.reload = UINT32_MAX;
    if (gptmr_channel_config(timer, channel, &timer_config, false) != status_success)
        return;

    trgm_output_t route = {
        .invert = false, .type = trgm_output_same_as_input, .input = HPM_TRGM0_INPUT_SRC_ESC_SYNC0
    };
    trgm_output_config(HPM_TRGM0, DCU_SYNC0_CAPTURE_ROUTE, &route);
    gptmr_start_counter(timer, channel);
    capture_clock_hz = hz;
}

void dcu_sync0_capture_begin(uint32_t period_ns)
{
    sync0_capture_stats = (dcu_sync0_capture_stats_t){.clock_hz = capture_clock_hz, .period_ns = period_ns};
    if (!capture_clock_hz) return;
    lower_bound = (uint64_t)capture_clock_hz * (period_ns - DCU_SYNC0_CAPTURE_TOLERANCE_NS);
    upper_bound = (uint64_t)capture_clock_hz * (period_ns + DCU_SYNC0_CAPTURE_TOLERANCE_NS);
    gptmr_clear_status(DCU_SYNC0_CAPTURE_TIMER, GPTMR_CH_CAP_STAT_MASK(DCU_SYNC0_CAPTURE_CHANNEL));
}

void dcu_sync0_capture_observe(uint32_t sync_events)
{
    if (!capture_clock_hz) return;
    volatile dcu_sync0_capture_stats_t *stats = &sync0_capture_stats;
    stats->sync_events += sync_events;
    stats->coalesced_events += sync_events - 1U;
    const unsigned channel = DCU_SYNC0_CAPTURE_CHANNEL;
    const uint32_t flag = GPTMR_CH_CAP_STAT_MASK(channel);
    if (!gptmr_check_status(DCU_SYNC0_CAPTURE_TIMER, flag)) {
        ++stats->missing_flags;
        return;
    }
    const uint32_t captured = gptmr_channel_get_counter(DCU_SYNC0_CAPTURE_TIMER, channel,
                                                        gptmr_counter_type_rising_edge);
    gptmr_clear_status(DCU_SYNC0_CAPTURE_TIMER, flag);
    if (stats->edges) {
        const uint32_t previous = stats->last_capture;
        const uint32_t ticks = captured - previous;
        stats->wraps += captured < previous;
        if (!stats->intervals || ticks < stats->min_ticks) stats->min_ticks = ticks;
        if (ticks > stats->max_ticks) stats->max_ticks = ticks;
        ++stats->intervals;
        stats->interval_sum_ticks += ticks;
        const uint64_t scaled = (uint64_t)ticks * 1000000000U;
        if (scaled < lower_bound || scaled > upper_bound) ++stats->outside_tolerance;
    } else {
        stats->first_capture = captured;
    }
    stats->last_capture = captured;
    ++stats->edges;
}
