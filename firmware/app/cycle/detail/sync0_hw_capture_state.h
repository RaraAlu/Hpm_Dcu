#ifndef DCU_SYNC0_HW_CAPTURE_STATE_H
#define DCU_SYNC0_HW_CAPTURE_STATE_H

#include <stdint.h>

typedef struct {
    uint32_t clock_hz, period_ns, sync_events, edges, intervals;
    uint32_t missing_flags, coalesced_events, outside_tolerance;
    uint32_t min_ticks, max_ticks, first_capture, last_capture, wraps;
    uint64_t interval_sum_ticks;
} dcu_sync0_capture_stats_t;

#endif
