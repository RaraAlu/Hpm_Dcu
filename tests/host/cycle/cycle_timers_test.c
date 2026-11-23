#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cycle_timers.c"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

test_gptmr_t test_timebase;
test_machine_timer_t test_machine_timer;
static uint32_t machine_hz = 24000000U, micro_hz = 1000000U;
static unsigned starts, clears, enables;
static bool status;
static uint32_t gap_reload;
static int divider_result;

uint32_t clock_get_frequency(uint32_t clock)
{
    return clock == clock_mchtmr0 ? machine_hz : clock == clock_gptmr1 ? micro_hz : 24000000U;
}
void clock_add_to_group(uint32_t clock, uint32_t group) { (void)clock; CHECK(group == 0); }
int clock_set_source_divider(uint32_t clock, uint32_t source, uint32_t divider)
{
    CHECK(clock == clock_gptmr1 && source == clk_src_osc24m && divider == 24);
    return divider_result;
}
void gptmr_channel_get_default_config(void *timer, gptmr_channel_config_t *config)
{ (void)timer; memset(config, 0, sizeof(*config)); }
void gptmr_channel_config(void *timer, uint32_t channel, gptmr_channel_config_t *config, bool start)
{
    CHECK(channel <= 1 && !config->enable_cmp_output && !start);
    CHECK(config->reload == (timer == HPM_GPTMR1 || channel == 1 ? UINT32_MAX : 1200));
}
void gptmr_start_counter(void *timer, uint32_t channel) { (void)timer; CHECK(channel <= 1); ++starts; }
void gptmr_clear_status(void *timer, uint32_t mask) { CHECK(timer == HPM_GPTMR2 && (mask == 1 || mask == 2)); ++clears; }
void gptmr_enable_irq(void *timer, uint32_t mask) { CHECK(timer == HPM_GPTMR2 && (mask == 1 || mask == 2)); ++enables; }
bool gptmr_check_status(void *timer, uint32_t mask) { CHECK(timer == HPM_GPTMR2 && (mask == 1 || mask == 2)); return status; }
void gptmr_stop_counter(void *timer, uint32_t ch) { CHECK(timer == HPM_GPTMR2 && ch == 1); }
void gptmr_channel_reset_count(void *timer, uint32_t ch) { CHECK(timer == HPM_GPTMR2 && ch == 1); }
void gptmr_channel_config_update_reload(void *timer, uint32_t ch, uint32_t value)
{ CHECK(timer == HPM_GPTMR2 && ch == 1); gap_reload = value; }
void intc_m_disable_irq(uint32_t irq) { CHECK(irq == IRQn_GPTMR2); }
void intc_m_enable_irq(uint32_t irq) { CHECK(irq == IRQn_GPTMR2); }
void intc_m_enable_irq_with_priority(uint32_t irq, uint32_t priority)
{ CHECK(irq == IRQn_GPTMR2 && priority == DCU_IRQ_WORKER_PRIORITY); ++enables; }

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "frequency")) {
        machine_hz = 12000000;
        CHECK(!dcu_timebase_init() && starts == 0);
        machine_hz = 24000000;
        micro_hz = 24000000;
        CHECK(!dcu_timebase_init() && starts == 0);
        micro_hz = 1000000;
        divider_result = -1;
        CHECK(!dcu_timebase_init() && starts == 0);
        return 0;
    }
    const uint64_t origin = (UINT64_C(12345) << 32) + UINT32_MAX - 30000;
    test_machine_timer.MTIME = origin;
    CHECK(dcu_timebase_init() && starts == 1);
    CHECK(dcu_time_now_ms() == 0);
    if (!strcmp(argv[1], "elapsed")) {
        test_machine_timer.MTIME = origin + 23999;
        CHECK(dcu_time_tick_hz() == 24000000U);
        CHECK(dcu_time_ticks() == (uint32_t)(origin + 23999));
        CHECK(dcu_time_now_ms() == 0);
        test_machine_timer.MTIME++;
        CHECK(dcu_time_now_ms() == 1);
        test_machine_timer.MTIME += 24000;
        CHECK(dcu_time_now_ms() == 2); /* Crossed the hardware low-word rollover. */
        test_timebase.CHANNEL[0].CNT = UINT32_MAX;
        CHECK(dcu_time_now_us() == UINT32_MAX);
        test_timebase.CHANNEL[0].CNT = 0;
        CHECK(dcu_time_now_us() == 0 && dcu_time_now_ms() == 2);
    } else if (!strcmp(argv[1], "wrap")) {
        test_machine_timer.MTIME = origin + (uint64_t)UINT32_MAX * 24000;
        uint32_t before = dcu_time_now_ms();
        CHECK(before == UINT32_MAX);
        test_machine_timer.MTIME += 10 * 24000;
        CHECK(dcu_time_now_ms() == 9 && (uint32_t)(dcu_time_now_ms() - before) == 10);
    } else if (!strcmp(argv[1], "delay")) {
        /* No callbacks, scheduler ticks or timer ISRs occur during these jumps. */
        const uint64_t milliseconds[] = {11, 1000, 5000000, UINT64_C(500) * 86400000};
        for (unsigned i = 0; i < sizeof(milliseconds) / sizeof(milliseconds[0]); ++i) {
            test_machine_timer.MTIME = origin + milliseconds[i] * 24000;
            CHECK(dcu_time_now_ms() == (uint32_t)milliseconds[i]);
        }
    } else if (!strcmp(argv[1], "worker")) {
        dcu_worker_timer_start();
        CHECK(starts == 2 && clears == 2 && enables == 3);
        CHECK(!dcu_worker_timer_ack() && clears == 2);
        status = true;
        CHECK(dcu_worker_timer_ack() && clears == 3);
        test_timebase.CHANNEL[0].CNT = UINT32_MAX - 19;
        dcu_can_arm_gap_at_us(80);
        CHECK(gap_reload == 2400 && starts == 3);
        dcu_can_arm_gap_at_us(90);
        CHECK(starts == 3);
        dcu_can_arm_gap_at_us(30);
        CHECK(gap_reload == 1200 && starts == 4);
        CHECK(dcu_can_wake_timer_ack() && !wake_armed);
    } else {
        CHECK(false);
    }
    return 0;
}
