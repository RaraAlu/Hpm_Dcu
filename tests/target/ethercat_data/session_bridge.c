#include "session_bridge.h"
#include "data_contract.h"
#include "gateway_task.h"
#include "mcan_echo.h"
#include "realtime_port.h"
#include "gateway_identity.h"

#include <string.h>

static uint32_t s_last_tick_us;
static uint64_t s_time_us;
static dcu_gateway_activation_result_t s_result = {.code = DCU_GATEWAY_INVALID};

static const dcu_gateway_task_config_t s_config = {
    .profile = {
        .profile_id = DCU_PROFILE_PROFILE_ID,
        .revision = DCU_PROFILE_REVISION,
        .digest = DCU_PROFILE_DIGEST,
        .readiness_ttl_ms = DCU_PROFILE_READINESS_TTL_MS,
        .activation_wait_s = DCU_PROFILE_ACTIVATION_WAIT_S,
    },
    .probe_interval_ms = DCU_PROFILE_PROBE_INTERVAL_MS,
    .command_timeout_ms = DCU_PROFILE_COMMAND_TIMEOUT_MS,
};

static uint64_t time_us(void)
{
    uint32_t tick = dcu_time_now_us();
    s_time_us += (uint32_t)(tick - s_last_tick_us);
    s_last_tick_us = tick;
    return s_time_us;
}

static uint64_t load_le(const uint8_t *data, size_t size)
{
    uint64_t value = 0;
    for (size_t i = 0; i < size; ++i) value |= (uint64_t)data[i] << (8U * i);
    return value;
}

static void store_le(uint8_t *data, uint64_t value, size_t size)
{
    for (size_t i = 0; i < size; ++i) data[i] = (uint8_t)(value >> (8U * i));
}

static void stop(void)
{
    dcu_data_mcan_disable();
    dcu_data_reset();
}

static void publish(const void *data, size_t size)
{
    (void)dcu_data_capture(data, size);
}

bool dcu_data_session_boot(void)
{
    const dcu_gateway_task_io_t io = {dcu_data_mcan_stage, stop, publish};
    const uint64_t boot_id = dcu_gateway_ids_boot(dcu_gateway_flash_storage());
    return boot_id && dcu_data_mcan_enable() &&
           dcu_gateway_task_boot(&s_config, &io, DCU_DATA_RX_BYTES, boot_id, dcu_gateway_ids_next, time_us());
}

void dcu_data_session_step(bool operational)
{
    dcu_gateway_task_step(time_us(), operational, dcu_data_mcan_healthy());
}

void dcu_data_session_pdo(const void *data, size_t size)
{
    if (!data || size != DCU_DATA_RX_BYTES) return;
    const uint8_t *bytes = data;
    (void)dcu_gateway_task_stage(load_le(bytes, 8), (uint32_t)load_le(bytes + 8, 4), data, size, time_us());
}

void dcu_data_session_can_complete(const void *data, size_t size)
{
    dcu_gateway_task_can_complete(data, size, time_us());
}

size_t dcu_data_session_read(uint16_t index, void *data, size_t capacity)
{
    if (!data) return 0;
    uint8_t *bytes = data;
    if (index == DCU_SESSION_STATUS_INDEX && capacity == DCU_SESSION_STATUS_BYTES) {
        const dcu_gateway_readiness_t s = dcu_gateway_readiness(time_us());
        memset(bytes, 0, capacity);
        store_le(bytes, DCU_DATA_LAYOUT, 2);
        bytes[2] = (uint8_t)s.state;
        bytes[3] = s.state == DCU_GATEWAY_READY || s.state == DCU_GATEWAY_RUNNING;
        store_le(bytes + 4, s.profile_id, 4);
        store_le(bytes + 8, s.revision, 4);
        memcpy(bytes + 12, s.digest, sizeof(s.digest));
        store_le(bytes + 44, s.boot_id, 8);
        store_le(bytes + 52, s.ready_generation, 4);
        store_le(bytes + 56, s.active_request_id, 4);
        store_le(bytes + 60, s.session_id, 8);
        store_le(bytes + 68, s.reason, 4);
        store_le(bytes + 72, s.feedback_age_us, 4);
        store_le(bytes + 76, s.client_tag, 4);
        return capacity;
    }
    if (index == DCU_SESSION_RESULT_INDEX && capacity == DCU_SESSION_RESULT_BYTES) {
        memset(bytes, 0, capacity);
        store_le(bytes, s_result.request_id, 4);
        store_le(bytes + 4, s_result.code, 2);
        bytes[6] = (uint8_t)s_result.state;
        store_le(bytes + 8, s_result.session_id, 8);
        return capacity;
    }
    return 0;
}

bool dcu_data_session_activate(const void *data, size_t size)
{
    if (!data || size != DCU_SESSION_ACTIVATE_BYTES) return false;
    const uint8_t *bytes = data;
    dcu_gateway_activation_t request = {
        .profile_id = (uint32_t)load_le(bytes, 4),
        .revision = (uint32_t)load_le(bytes + 4, 4),
        .boot_id = load_le(bytes + 40, 8),
        .ready_generation = (uint32_t)load_le(bytes + 48, 4),
        .request_id = (uint32_t)load_le(bytes + 52, 4),
        .client_tag = (uint32_t)load_le(bytes + 56, 4),
        .reserved = (uint32_t)load_le(bytes + 60, 4),
    };
    memcpy(request.digest, bytes + 8, sizeof(request.digest));
    s_result = dcu_gateway_activate(&request, time_us());
    return true;
}
