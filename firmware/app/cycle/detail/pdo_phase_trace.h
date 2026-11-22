#ifndef DCU_PDO_PHASE_TRACE_H
#define DCU_PDO_PHASE_TRACE_H

#include <stdint.h>

#define DCU_PDO_PHASE_SAMPLES 128U

typedef struct {
    uint32_t sequence, epoch, rx_ready_us, sync0_us;
    uint32_t accepted, published_us, dc_sample_us, dc_low_ns;
    uint32_t worker_us, output_begin_us, output_end_us;
    uint32_t process_calls, process_us, command_after_release_us, command_done_us;
    uint32_t publish_begin_us, pack_end_us, esc_begin_us, esc_end_us;
    /* Publish boundaries are publish_begin_us / esc_end_us in the worker clock.
     * UINT32_MAX means no accepted neutral reply in this epoch. retx_count is
     * observed TX error samples, not the number of on-wire attempts. It is
     * provisional at publish and final at the next consecutive Sync0. */
    uint32_t last_consumed_offset_from_sync0, retx_count;
    uint32_t neutral_wait_checks, neutral_last_wait_us;
} dcu_pdo_phase_sample_t;

typedef struct {
    uint32_t count;
    dcu_pdo_phase_sample_t sample[DCU_PDO_PHASE_SAMPLES];
    uint32_t fault_epoch, fault_us, fault_flags;
} dcu_pdo_phase_trace_t;

typedef struct {
    uint32_t sequence, pdi_sm2_us, esc_start_us, esc_end_us, stage_start_us, stage_commit_us;
    uint32_t direct_to_slot;
} dcu_rx_phase_sample_t;

typedef struct {
    uint32_t count;
    dcu_rx_phase_sample_t sample[DCU_PDO_PHASE_SAMPLES];
} dcu_rx_phase_trace_t;

void dcu_pdo_phase_record_rx(const void *data, uint32_t stage_start_us, uint32_t stage_commit_us,
                            uint32_t direct_to_slot, uint32_t direct_read_end_us);

#endif
