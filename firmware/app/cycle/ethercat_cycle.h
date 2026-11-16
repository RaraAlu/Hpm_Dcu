#ifndef DCU_ETHERCAT_CYCLE_H
#define DCU_ETHERCAT_CYCLE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t configured_period_ns;
    uint32_t sync0_count;
    uint32_t latched_sequence;
    uint32_t published_epoch;
    uint32_t faults;
    uint32_t worker_max_us;
    uint32_t worker_stack_free_words;
    uint32_t fatal_line;
    const char *fatal_file;
} dcu_cycle_diagnostics_t;

extern volatile dcu_cycle_diagnostics_t g_dcu_cycle_diagnostics;

void dcu_cycle_init(void);
void dcu_cycle_step(void);
/* SSC/worker task only, except for the explicit PDI/Sync/timer handoffs below. */
uint16_t dcu_cycle_configure(void);
uint16_t dcu_cycle_start_outputs(void);
void dcu_cycle_stop_outputs(void);
void dcu_cycle_stop_inputs(void);
/* The PDI producer stages complete RxPDOs; dedicated ISRs only hand off events. */
void dcu_cycle_stage(const void *data);
void dcu_cycle_sync0_isr(void);
void dcu_cycle_sync1_isr(void);
void dcu_cycle_timer_isr(void);
void dcu_cycle_watchdog_isr(void);

#ifdef __cplusplus
}
#endif
#endif
