#ifndef DCU_ETHERCAT_CYCLE_STATE_H
#define DCU_ETHERCAT_CYCLE_STATE_H

#include <stdbool.h>
#include <stdint.h>

/* Every handoff field is accessed atomically across ISR/task ownership. */
typedef struct {
    uint32_t sync0_pending, sync1_pending, timer_pending;
    uint32_t stage_sequence;
} dcu_cycle_handoff_t;

typedef struct {
    bool configured, outputs_running;
} dcu_cycle_runtime_t;

#endif
