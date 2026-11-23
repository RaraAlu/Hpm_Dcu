#ifndef DCU_ETHERCAT_CYCLE_STATE_H
#define DCU_ETHERCAT_CYCLE_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "cycle_config.h"

/* Every handoff field is accessed atomically across ISR/task ownership. */
typedef struct {
    uint32_t sync0_pending, sync1_pending, timer_pending;
    uint32_t sm2_cycles, sm2_event_us;
    uint32_t stage_sequence;
    uint32_t stage_start_us, stage_busy, stage_done_us;
    uint32_t miss_epoch, miss_sync_us, miss_last_stage_us, miss_stage_busy, miss_stage_start_us;
} dcu_cycle_handoff_t;

typedef struct {
    bool configured, outputs_running;
    bool local_sync_fault, safeop_error_acknowledged;
    bool command_pending;
    uint16_t ssc_sync0_latch;
    uint32_t pending_epoch, processed_epoch, preview_sequence;
} dcu_cycle_runtime_t;

#define DCU_RAW_PDO_QUEUE_DEPTH 4U
typedef struct {
    uint32_t enabled, read, write, fault, worker_us, lease_us;
    uint32_t image[DCU_RAW_PDO_QUEUE_DEPTH][DCU_SINGLE_FD_PDO_BYTES / sizeof(uint32_t)];
    bool configured, published;
} dcu_raw_pdo_t;

#define DCU_CYCLE_TIMING_BINS 2048U

typedef struct {
    uint32_t tick_hz, count, min_ticks, max_ticks, first_epoch, last_epoch, last_sync_us, last_end_us;
} dcu_cyclic_latency_t;

typedef struct {
    uint32_t count, min_us, max_us, overflow;
    uint64_t sum_us;
    uint32_t histogram[DCU_CYCLE_TIMING_BINS];
} dcu_cycle_timing_histogram_t;

typedef struct {
    uint32_t previous_epoch, previous_sync_us;
    dcu_cycle_timing_histogram_t sync_interval, rx_ready_lead;
} dcu_cycle_timing_t;

#endif
