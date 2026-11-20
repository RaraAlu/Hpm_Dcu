#include "gateway_task.h"
#include "detail/gateway_task_state.h"

#include <string.h>

static dcu_gateway_task_state_t s_task;

static dcu_gateway_readiness_t status(uint64_t now_us)
{
    dcu_gateway_readiness_t state = dcu_gateway_readiness(now_us);
    if (state.state == DCU_GATEWAY_RUNNING &&
        now_us - s_task.last_command_us >= (uint64_t)s_task.command_timeout_ms * 1000U) {
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_MASTER_TIMEOUT, now_us);
        state = dcu_gateway_readiness(now_us);
    }
    if (state.state == DCU_GATEWAY_HOLD || state.state == DCU_GATEWAY_FAULT) {
        if (!s_task.stopped) s_task.io.stop();
        s_task.stopped = true;
        s_task.pending = GATEWAY_IDLE;
    }
    return state;
}

bool dcu_gateway_task_boot(const dcu_gateway_task_config_t *config, const dcu_gateway_task_io_t *io,
                          size_t pdo_bytes, uint64_t boot_id, dcu_gateway_issue_session_fn session,
                          uint64_t now_us)
{
    if (!config || !io || !io->stage || !io->stop || !io->publish || pdo_bytes < 16U ||
        pdo_bytes > DCU_GATEWAY_TASK_MAX_PDO || pdo_bytes % 8U || !config->probe_interval_ms ||
        config->probe_interval_ms >= config->profile.readiness_ttl_ms || !config->command_timeout_ms)
        return false;
    if (!dcu_gateway_boot_session(&config->profile, boot_id, session, now_us)) return false;
    s_task = (dcu_gateway_task_state_t){
        .io = *io, .bytes = pdo_bytes,
        .probe_interval_ms = config->probe_interval_ms, .command_timeout_ms = config->command_timeout_ms,
    };
    return true;
}

void dcu_gateway_task_step(uint64_t now_us, bool operational, bool can_healthy)
{
    if (!s_task.bytes) return;
    if (!can_healthy)
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_CAN_OFFLINE, now_us);
    else if (s_task.operational && !operational && dcu_gateway_readiness(now_us).session_id)
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_MASTER_TIMEOUT, now_us);
    s_task.operational = operational;
    const dcu_gateway_state_t state = status(now_us).state;
    if ((state != DCU_GATEWAY_PREPARING && state != DCU_GATEWAY_READY) ||
        s_task.pending != GATEWAY_IDLE ||
        (s_task.probe_sequence && now_us - s_task.last_probe_us < (uint64_t)s_task.probe_interval_ms * 1000U))
        return;
    if (s_task.probe_sequence == UINT32_MAX) {
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_PROTOCOL_MISMATCH, now_us);
        (void)status(now_us);
        return;
    }

    /* A changing diagnostic challenge is never published as business PDO feedback. */
    const uint32_t challenge = ++s_task.probe_sequence;
    for (size_t i = 0; i < s_task.bytes; ++i)
        s_task.expected[i] = (uint8_t)((challenge >> (8U * (i % 4U))) ^ (0x5aU + i));
    s_task.last_probe_us = now_us;
    if (s_task.io.stage(s_task.expected, s_task.bytes)) {
        s_task.pending = GATEWAY_PROBE;
    } else {
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_CAN_OFFLINE, now_us);
        (void)status(now_us);
    }
}

bool dcu_gateway_task_stage(uint64_t session_id, uint32_t sequence, const void *data, size_t size,
                            uint64_t now_us)
{
    if (!s_task.bytes) return false;
    const dcu_gateway_readiness_t state = status(now_us);
    if (!s_task.operational || !data || size != s_task.bytes || !session_id ||
        session_id != state.session_id || sequence <= s_task.command_sequence ||
        s_task.pending != GATEWAY_IDLE ||
        (state.state != DCU_GATEWAY_READY && state.state != DCU_GATEWAY_RUNNING))
        return false;

    memcpy(s_task.expected, data, size);
    if (!s_task.io.stage(data, size)) {
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_CAN_OFFLINE, now_us);
        (void)status(now_us);
        return false;
    }
    if (state.state == DCU_GATEWAY_READY && !dcu_gateway_start_run(session_id, now_us)) {
        s_task.io.stop();
        return false;
    }
    s_task.last_command_us = now_us;
    s_task.command_sequence = sequence;
    s_task.pending = GATEWAY_BUSINESS;
    return true;
}

void dcu_gateway_task_can_complete(const void *data, size_t size, uint64_t now_us)
{
    if (!s_task.bytes) return;
    const dcu_gateway_state_t state = status(now_us).state;
    const dcu_gateway_pending_t completed = s_task.pending;
    if (completed == GATEWAY_IDLE) return;
    s_task.pending = GATEWAY_IDLE;
    if (!data || size != s_task.bytes || memcmp(data, s_task.expected, size) != 0) {
        dcu_gateway_link_lost(DCU_GATEWAY_REASON_PROTOCOL_MISMATCH, now_us);
        (void)status(now_us);
        return;
    }
    if (dcu_gateway_probe_verified(now_us, now_us) &&
        completed == GATEWAY_BUSINESS && state == DCU_GATEWAY_RUNNING && s_task.operational)
        s_task.io.publish(data, size);
}
