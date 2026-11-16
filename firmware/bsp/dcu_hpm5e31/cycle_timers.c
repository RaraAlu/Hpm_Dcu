#include "cycle_timers.h"
#include "cfg/cycle_timer_map.h"
#include "cycle_config.h"
#include "realtime_port.h"
#include "hpm_gptmr_drv.h"

bool dcu_timebase_init(void)
{
    clock_add_to_group(DCU_TIMEBASE_CLOCK, 0);
    if (clock_set_source_divider(DCU_TIMEBASE_CLOCK, clk_src_osc24m, 24U) != status_success ||
        clock_get_frequency(DCU_TIMEBASE_CLOCK) != 1000000U) return false;
    gptmr_channel_config_t config;
    gptmr_channel_get_default_config(DCU_TIMEBASE_TIMER, &config);
    config.reload = UINT32_MAX;
    config.enable_cmp_output = false;
    gptmr_channel_config(DCU_TIMEBASE_TIMER, DCU_TIMEBASE_CHANNEL, &config, false);
    gptmr_start_counter(DCU_TIMEBASE_TIMER, DCU_TIMEBASE_CHANNEL);
    return true;
}
DCU_FAST_CODE uint32_t dcu_time_now_us(void)
{
    return GPTMR_CHANNEL_CNT_COUNTER_GET(DCU_TIMEBASE_TIMER->CHANNEL[DCU_TIMEBASE_CHANNEL].CNT);
}
void dcu_worker_timer_start(void)
{
    gptmr_channel_config_t config;
    clock_add_to_group(DCU_WORKER_TIMER_CLOCK, 0);
    gptmr_channel_get_default_config(DCU_WORKER_TIMER, &config);
    config.enable_cmp_output = false;
    config.reload = clock_get_frequency(DCU_WORKER_TIMER_CLOCK) / (1000000U / DCU_WORKER_PERIOD_US);
    gptmr_channel_config(DCU_WORKER_TIMER, DCU_WORKER_TIMER_CHANNEL, &config, false);
    gptmr_clear_status(DCU_WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(DCU_WORKER_TIMER_CHANNEL));
    gptmr_enable_irq(DCU_WORKER_TIMER, GPTMR_CH_RLD_IRQ_MASK(DCU_WORKER_TIMER_CHANNEL));
    intc_m_enable_irq_with_priority(DCU_WORKER_TIMER_IRQ, DCU_IRQ_WORKER_PRIORITY);
    gptmr_start_counter(DCU_WORKER_TIMER, DCU_WORKER_TIMER_CHANNEL);
}
bool dcu_worker_timer_ack(void)
{
    uint32_t mask = GPTMR_CH_RLD_STAT_MASK(DCU_WORKER_TIMER_CHANNEL);
    if (!gptmr_check_status(DCU_WORKER_TIMER, mask)) return false;
    gptmr_clear_status(DCU_WORKER_TIMER, mask);
    return true;
}
