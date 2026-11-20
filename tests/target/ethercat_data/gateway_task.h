#ifndef DCU_GATEWAY_TASK_H
#define DCU_GATEWAY_TASK_H

#include "startup/gateway_session.h"
#include <stddef.h>

typedef struct {
    dcu_gateway_profile_t profile;
    uint32_t probe_interval_ms;
    uint32_t command_timeout_ms;
} dcu_gateway_task_config_t;

typedef struct {
    /* Copy into one nonblocking CAN slot; never send a frame from this call. */
    bool (*stage)(const void *data, size_t size);
    void (*stop)(void);
    void (*publish)(const void *data, size_t size);
} dcu_gateway_task_io_t;

bool dcu_gateway_task_boot(const dcu_gateway_task_config_t *config, const dcu_gateway_task_io_t *io,
                           size_t pdo_bytes, uint64_t boot_id, dcu_gateway_issue_session_fn session,
                           uint64_t now_us);
void dcu_gateway_task_step(uint64_t now_us, bool operational, bool can_healthy);
bool dcu_gateway_task_stage(uint64_t session_id, uint32_t sequence, const void *data,
                            size_t size, uint64_t now_us);
void dcu_gateway_task_can_complete(const void *data, size_t size, uint64_t now_us);

#endif
