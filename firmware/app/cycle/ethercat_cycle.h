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
    uint32_t publish_max_us;
    uint32_t publish_start_delay_max_us;
    uint32_t prepare_max_us;
    uint32_t esc_write_max_us;
    uint32_t fault_al_status, fault_handled_us;
    uint32_t first_rx_miss_epoch, first_rx_miss_sync_us, first_rx_miss_last_stage_us;
    uint32_t first_rx_miss_stage_busy, first_rx_miss_stage_start_us, fault_last_stage_us;
    uint32_t local_sync_error_clears;
    uint32_t missing_can_devices;
} dcu_cycle_diagnostics_t;

/* Copy on the sole SSC/worker task, not from an ISR or a concurrent task.
 * A debugger may inspect the private instance only while the CPU is halted. */
void dcu_cycle_read_diagnostics(dcu_cycle_diagnostics_t *snapshot);

void dcu_cycle_init(void);
void dcu_cycle_step(void);
/* SSC/worker task only, except for the explicit PDI/Sync/timer handoffs below. */
uint16_t dcu_cycle_configure(void);
uint16_t dcu_cycle_start_outputs(void);
void dcu_cycle_stop_outputs(void);
void dcu_cycle_stop_inputs(void);
void dcu_cycle_ack_safeop_error(void);
/* The PDI producer stages complete RxPDOs; dedicated ISRs only hand off events. */
/* Called at the start of the SSC SM2 read, before either data-copy path. */
void dcu_cycle_sm2_event_isr(void);
void dcu_cycle_stage(const void *data);
/* A true result means the full SM2 read was already performed, even if the
 * cycle stopped before commit. read_output returns the image-complete time.
 * A false result leaves the read to SSC. */
bool dcu_cycle_stage_read(uint16_t address, uint16_t size,
                          uint32_t (*read_output)(void *, uint16_t, uint16_t));
void dcu_cycle_sync0_isr(void);
void dcu_cycle_sync1_isr(void);
void dcu_cycle_timer_isr(void);
void dcu_cycle_watchdog_isr(void);

#ifdef __cplusplus
}
#endif
#endif
