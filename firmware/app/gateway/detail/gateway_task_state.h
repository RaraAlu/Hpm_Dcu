#ifndef DCU_GATEWAY_TASK_STATE_H
#define DCU_GATEWAY_TASK_STATE_H

#include "../gateway_task.h"

enum { DCU_GATEWAY_TASK_MAX_PDO = 32 };
typedef enum { GATEWAY_IDLE, GATEWAY_PROBE, GATEWAY_BUSINESS } dcu_gateway_pending_t;

typedef struct {
    dcu_gateway_task_io_t io;
    size_t bytes;
    uint64_t last_probe_us, last_command_us;
    uint32_t probe_interval_ms, command_timeout_ms;
    uint32_t probe_sequence, command_sequence;
    uint8_t expected[DCU_GATEWAY_TASK_MAX_PDO];
    dcu_gateway_pending_t pending;
    bool operational, stopped;
} dcu_gateway_task_state_t;

#endif
