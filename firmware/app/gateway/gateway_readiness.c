#include "gateway_readiness.h"
#include "detail/gateway_readiness_state.h"

#include <string.h>

static dcu_gateway_readiness_state_t s_gateway;

static void leave_ready(dcu_gateway_state_t state, dcu_gateway_reason_t reason)
{
    s_gateway.session_id = 0;
    if (s_gateway.state == DCU_GATEWAY_READY || s_gateway.state == DCU_GATEWAY_RUNNING) {
        if (s_gateway.ready_generation == UINT32_MAX) {
            state = DCU_GATEWAY_FAULT;
            reason = DCU_GATEWAY_REASON_COUNTER_EXHAUSTED;
        } else {
            ++s_gateway.ready_generation;
        }
    }
    s_gateway.state = state;
    s_gateway.reason = reason;
}

static uint64_t ready_limit_us(void)
{
    return (uint64_t)s_gateway.profile.readiness_ttl_ms * 1000U;
}

bool dcu_gateway_boot(const dcu_gateway_profile_t *profile, uint64_t boot_id,
                      dcu_gateway_issue_session_fn issue_session, uint64_t now_us)
{
    if (s_gateway.state != DCU_GATEWAY_BOOT) return false;
    s_gateway.boot_id = boot_id;
    s_gateway.now_us = now_us;
    if (!profile || !profile->profile_id || !profile->revision || !profile->readiness_ttl_ms ||
        !profile->activation_wait_s || !boot_id || !issue_session) {
        leave_ready(DCU_GATEWAY_FAULT, DCU_GATEWAY_REASON_CONFIG_INVALID);
        return false;
    }
    s_gateway.profile = *profile;
    s_gateway.issue_session = issue_session;
    s_gateway.prepared_since_us = now_us;
    s_gateway.state = DCU_GATEWAY_PREPARING;
    return true;
}

void dcu_gateway_tick(uint64_t now_us)
{
    if (s_gateway.state == DCU_GATEWAY_BOOT || s_gateway.state == DCU_GATEWAY_FAULT) return;
    if (now_us < s_gateway.now_us) {
        leave_ready(DCU_GATEWAY_FAULT, DCU_GATEWAY_REASON_PROTOCOL_MISMATCH);
        return;
    }
    s_gateway.now_us = now_us;
    if (s_gateway.state != DCU_GATEWAY_READY && s_gateway.state != DCU_GATEWAY_RUNNING) return;
    if (s_gateway.state == DCU_GATEWAY_READY && s_gateway.session_id &&
        now_us - s_gateway.accepted_us >= (uint64_t)s_gateway.profile.activation_wait_s * 1000000U) {
        leave_ready(DCU_GATEWAY_HOLD, DCU_GATEWAY_REASON_PROTOCOL_MISMATCH);
    } else if (now_us - s_gateway.last_reply_us >= ready_limit_us()) {
        leave_ready(DCU_GATEWAY_HOLD, DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT);
    }
}

bool dcu_gateway_probe_verified(uint64_t reply_us, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_PREPARING && s_gateway.state != DCU_GATEWAY_READY &&
        s_gateway.state != DCU_GATEWAY_RUNNING) return false;
    if (reply_us < s_gateway.prepared_since_us || reply_us > now_us ||
        (s_gateway.have_reply && reply_us <= s_gateway.last_reply_us) ||
        now_us - reply_us >= ready_limit_us()) return false;
    if (s_gateway.state == DCU_GATEWAY_PREPARING) {
        if (s_gateway.ready_generation == UINT32_MAX) {
            leave_ready(DCU_GATEWAY_FAULT, DCU_GATEWAY_REASON_COUNTER_EXHAUSTED);
            return false;
        }
        ++s_gateway.ready_generation;
        s_gateway.state = DCU_GATEWAY_READY;
        s_gateway.reason = DCU_GATEWAY_REASON_NONE;
    }
    s_gateway.have_reply = true;
    s_gateway.last_reply_us = reply_us;
    return true;
}

void dcu_gateway_link_lost(dcu_gateway_reason_t reason, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_PREPARING && s_gateway.state != DCU_GATEWAY_READY &&
        s_gateway.state != DCU_GATEWAY_RUNNING) return;
    if (reason != DCU_GATEWAY_REASON_CAN_OFFLINE && reason != DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT &&
        reason != DCU_GATEWAY_REASON_MASTER_TIMEOUT && reason != DCU_GATEWAY_REASON_PROTOCOL_MISMATCH) return;
    leave_ready(DCU_GATEWAY_HOLD, reason);
}

bool dcu_gateway_retry_preparation(uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_HOLD) return false;
    s_gateway.prepared_since_us = now_us;
    s_gateway.state = DCU_GATEWAY_PREPARING;
    s_gateway.reason = DCU_GATEWAY_REASON_NONE;
    return true;
}

dcu_gateway_activation_result_t dcu_gateway_activate(const dcu_gateway_activation_t *request, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    dcu_gateway_activation_result_t result = {
        .request_id = request ? request->request_id : 0,
        .code = DCU_GATEWAY_INVALID,
        .state = s_gateway.state,
    };
    if (!request || !request->profile_id || !request->revision || !request->boot_id ||
        !request->request_id || !request->client_tag || request->reserved) return result;
    if (s_gateway.state != DCU_GATEWAY_READY && s_gateway.state != DCU_GATEWAY_RUNNING) {
        result.code = DCU_GATEWAY_NOT_READY;
        return result;
    }
    bool same_profile = request->profile_id == s_gateway.profile.profile_id &&
        request->revision == s_gateway.profile.revision &&
        memcmp(request->digest, s_gateway.profile.digest, sizeof(request->digest)) == 0;
    bool same_context = request->boot_id == s_gateway.boot_id &&
        request->ready_generation == s_gateway.ready_generation;
    if (s_gateway.session_id && request->request_id == s_gateway.last_request_id) {
        if (same_profile && same_context && request->client_tag == s_gateway.last_client_tag) {
            result.code = DCU_GATEWAY_ACCEPTED;
            result.session_id = s_gateway.session_id;
        }
        return result;
    }
    if (!same_profile) {
        result.code = DCU_GATEWAY_CONFIG_MISMATCH;
    } else if (!same_context || request->request_id <= s_gateway.last_request_id ||
               request->client_tag <= s_gateway.last_client_tag) {
        result.code = DCU_GATEWAY_STALE_CONTEXT;
    } else if (s_gateway.session_id) {
        result.code = DCU_GATEWAY_BUSY;
    } else {
        uint64_t session_id = s_gateway.issue_session();
        if (session_id <= s_gateway.last_session_id) {
            leave_ready(DCU_GATEWAY_FAULT, DCU_GATEWAY_REASON_COUNTER_EXHAUSTED);
            result.state = s_gateway.state;
            result.code = DCU_GATEWAY_NOT_READY;
            return result;
        }
        s_gateway.last_request_id = request->request_id;
        s_gateway.last_client_tag = request->client_tag;
        s_gateway.session_id = s_gateway.last_session_id = session_id;
        s_gateway.accepted_us = now_us;
        result.code = DCU_GATEWAY_ACCEPTED;
        result.session_id = session_id;
    }
    return result;
}

bool dcu_gateway_resume_static(uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_READY || s_gateway.session_id) return false;
    s_gateway.state = DCU_GATEWAY_RUNNING;
    return true;
}

bool dcu_gateway_apply_policy(uint32_t profile_id, uint32_t base_revision, uint32_t revision,
                             uint32_t ttl_ms, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if ((s_gateway.state != DCU_GATEWAY_PREPARING && s_gateway.state != DCU_GATEWAY_READY &&
         s_gateway.state != DCU_GATEWAY_HOLD) || s_gateway.session_id ||
        s_gateway.profile.profile_id != profile_id || s_gateway.profile.revision != base_revision ||
        revision <= base_revision || !ttl_ms || s_gateway.ready_generation == UINT32_MAX)
        return false;
    if (s_gateway.state == DCU_GATEWAY_READY) ++s_gateway.ready_generation;
    s_gateway.profile.revision = revision;
    s_gateway.profile.readiness_ttl_ms = ttl_ms;
    s_gateway.have_reply = false;
    s_gateway.prepared_since_us = now_us;
    s_gateway.state = DCU_GATEWAY_PREPARING;
    s_gateway.reason = DCU_GATEWAY_REASON_NONE;
    return true;
}

bool dcu_gateway_start_run(uint64_t session_id, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_READY || !session_id || session_id != s_gateway.session_id)
        return false;
    s_gateway.state = DCU_GATEWAY_RUNNING;
    return true;
}

dcu_gateway_readiness_t dcu_gateway_readiness(uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    dcu_gateway_readiness_t status = {
        .state = s_gateway.state,
        .reason = s_gateway.reason,
        .profile_id = s_gateway.profile.profile_id,
        .revision = s_gateway.profile.revision,
        .boot_id = s_gateway.boot_id,
        .ready_generation = s_gateway.ready_generation,
        .active_request_id = s_gateway.session_id ? s_gateway.last_request_id : 0,
        .session_id = s_gateway.session_id,
        .client_tag = s_gateway.session_id ? s_gateway.last_client_tag : 0,
        .feedback_age_us = UINT32_MAX,
    };
    memcpy(status.digest, s_gateway.profile.digest, sizeof(status.digest));
    if (s_gateway.state == DCU_GATEWAY_READY || s_gateway.state == DCU_GATEWAY_RUNNING) {
        uint64_t age_us = now_us - s_gateway.last_reply_us;
        status.feedback_age_us = age_us < UINT32_MAX ? (uint32_t)age_us : UINT32_MAX - 1U;
    }
    return status;
}
