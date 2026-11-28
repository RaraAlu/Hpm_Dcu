#include "cycle_timers.h"
#include "cfg/cycle_timer_map.h"
#include "cycle_config.h"
#include "realtime_port.h"
#include "hpm_gptmr_drv.h"

static uint64_t machine_time_origin;
static uint32_t wake_ticks_per_us, wake_due;
static bool wake_armed;

DCU_FAST_CODE uint32_t dcu_time_ticks(void)
{
    return (uint32_t)DCU_MACHINE_TIMER->MTIME;
}

uint32_t dcu_time_tick_hz(void)
{
    return DCU_MACHINE_TIMER_HZ;
}

static uint64_t machine_time(void)
{
    /* RV32 needs high/low/high reads: a native uint64_t load can tear when
     * the low word rolls over, even with interrupts disabled. */
    volatile const uint32_t *words = (volatile const uint32_t *)&DCU_MACHINE_TIMER->MTIME;
    uint32_t high, low;
    do {
        high = words[1];
        low = words[0];
    } while (high != words[1]);
    return ((uint64_t)high << 32) | low;
}

bool dcu_timebase_init(void)
{
    if (clock_get_frequency(DCU_MACHINE_TIMER_CLOCK) != DCU_MACHINE_TIMER_HZ) return false;
    clock_add_to_group(DCU_TIMEBASE_CLOCK, 0);
    if (clock_set_source_divider(DCU_TIMEBASE_CLOCK, clk_src_osc24m, 24U) != status_success ||
        clock_get_frequency(DCU_TIMEBASE_CLOCK) != 1000000U) return false;
    gptmr_channel_config_t config;
    gptmr_channel_get_default_config(DCU_TIMEBASE_TIMER, &config);
    config.reload = UINT32_MAX;
    config.enable_cmp_output = false;
    gptmr_channel_config(DCU_TIMEBASE_TIMER, DCU_TIMEBASE_CHANNEL, &config, false);
    gptmr_start_counter(DCU_TIMEBASE_TIMER, DCU_TIMEBASE_CHANNEL);
    machine_time_origin = machine_time();
    return true;
}
DCU_FAST_CODE uint32_t dcu_time_now_ms(void)
{
    return (uint32_t)((machine_time() - machine_time_origin) / (DCU_MACHINE_TIMER_HZ / 1000U));
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
    wake_ticks_per_us = clock_get_frequency(DCU_WORKER_TIMER_CLOCK) / 1000000U;
    config.reload = UINT32_MAX;
    gptmr_channel_config(DCU_WORKER_TIMER, DCU_CAN_WAKE_CHANNEL, &config, false);
    gptmr_clear_status(DCU_WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(DCU_CAN_WAKE_CHANNEL));
    gptmr_enable_irq(DCU_WORKER_TIMER, GPTMR_CH_RLD_IRQ_MASK(DCU_CAN_WAKE_CHANNEL));
    gptmr_clear_status(DCU_WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(DCU_WORKER_TIMER_CHANNEL));
    gptmr_enable_irq(DCU_WORKER_TIMER, GPTMR_CH_RLD_IRQ_MASK(DCU_WORKER_TIMER_CHANNEL));
    intc_m_enable_irq_with_priority(DCU_WORKER_TIMER_IRQ, DCU_IRQ_WORKER_PRIORITY);
    gptmr_start_counter(DCU_WORKER_TIMER, DCU_WORKER_TIMER_CHANNEL);
}
DCU_FAST_CODE bool dcu_worker_timer_ack(void)
{
    uint32_t mask = GPTMR_CH_RLD_STAT_MASK(DCU_WORKER_TIMER_CHANNEL);
    if (!gptmr_check_status(DCU_WORKER_TIMER, mask)) return false;
    gptmr_clear_status(DCU_WORKER_TIMER, mask);
    return true;
}

DCU_FAST_CODE void dcu_can_arm_gap_at_us(uint32_t due_us)
{
    // One relative reload serves the earliest bus. The IRQ rechecks the other buses.
    if (!wake_armed || (int32_t)(due_us - wake_due) < 0) {
        gptmr_stop_counter(DCU_WORKER_TIMER, DCU_CAN_WAKE_CHANNEL);
        const int32_t delay = (int32_t)(due_us - dcu_time_now_us());
        gptmr_channel_config_update_reload(DCU_WORKER_TIMER, DCU_CAN_WAKE_CHANNEL,
                                           (delay > 0 ? (uint32_t)delay : 1U) * wake_ticks_per_us);
        gptmr_channel_reset_count(DCU_WORKER_TIMER, DCU_CAN_WAKE_CHANNEL);
        gptmr_clear_status(DCU_WORKER_TIMER, GPTMR_CH_RLD_STAT_MASK(DCU_CAN_WAKE_CHANNEL));
        wake_due = due_us;
        wake_armed = true;
        gptmr_start_counter(DCU_WORKER_TIMER, DCU_CAN_WAKE_CHANNEL);
    }
}

DCU_FAST_CODE bool dcu_can_wake_timer_ack(void)
{
    const uint32_t mask = GPTMR_CH_RLD_STAT_MASK(DCU_CAN_WAKE_CHANNEL);
    if (!gptmr_check_status(DCU_WORKER_TIMER, mask)) return false;
    gptmr_stop_counter(DCU_WORKER_TIMER, DCU_CAN_WAKE_CHANNEL);
    gptmr_clear_status(DCU_WORKER_TIMER, mask);
    wake_armed = false;
    return true;
}
