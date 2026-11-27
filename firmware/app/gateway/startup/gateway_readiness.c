#include "gateway_session.h"
#include "detail/gateway_readiness_state.h"
#include "gateway_flow.h"

#include <string.h>

static dcu_gateway_readiness_state_t s_gateway;

static void advance(dcu_gateway_event_t event)
{
    s_gateway.state = dcu_gateway_next(s_gateway.state, event, DCU_GATEWAY_REASON_NONE);
}

static void leave_ready(dcu_gateway_event_t event, dcu_gateway_reason_t reason)
{
    dcu_gateway_state_t next = dcu_gateway_next(s_gateway.state, event, reason);
    if (next == s_gateway.state) return;
    s_gateway.session_id = 0;
    if (s_gateway.state == DCU_GATEWAY_READY || s_gateway.state == DCU_GATEWAY_RUNNING) {
        if (s_gateway.ready_generation == UINT32_MAX) {
            reason = DCU_GATEWAY_REASON_COUNTER_EXHAUSTED;
            next = dcu_gateway_next(s_gateway.state, GW_FATAL, reason);
        } else {
            ++s_gateway.ready_generation;
        }
    }
    s_gateway.state = next;
    s_gateway.reason = reason;
}

static uint64_t ready_limit_us(void)
{
    return (uint64_t)s_gateway.profile.readiness_ttl_ms * 1000U;
}

static bool boot(const dcu_gateway_profile_t *profile, uint64_t boot_id, uint64_t now_us,
                 bool startup_valid)
{
    if (s_gateway.state != DCU_GATEWAY_BOOT) return false;
    s_gateway.boot_id = boot_id;
    s_gateway.now_us = now_us;
    if (!profile || !profile->profile_id || !profile->revision || !profile->readiness_ttl_ms ||
        !boot_id || !startup_valid) {
        leave_ready(GW_FATAL, DCU_GATEWAY_REASON_CONFIG_INVALID);
        return false;
    }
    s_gateway.profile = *profile;
    s_gateway.prepared_since_us = now_us;
    advance(GW_BOOT_VALID);
    return true;
}

bool dcu_gateway_boot_static(const dcu_gateway_profile_t *profile, uint64_t boot_id, uint64_t now_us)
{
    return boot(profile, boot_id, now_us, true);
}

bool dcu_gateway_boot_session(const dcu_gateway_profile_t *profile, uint64_t boot_id,
                              dcu_gateway_issue_session_fn issue_session, uint64_t now_us)
{
    if (!boot(profile, boot_id, now_us, profile && profile->activation_wait_s && issue_session)) return false;
    s_gateway.issue_session = issue_session;
    return true;
}

void dcu_gateway_tick(uint64_t now_us)
{
    if (s_gateway.state == DCU_GATEWAY_BOOT || s_gateway.state == DCU_GATEWAY_FAULT) return;
    if (now_us < s_gateway.now_us) {
        leave_ready(GW_FATAL, DCU_GATEWAY_REASON_PROTOCOL_MISMATCH);
        return;
    }
    s_gateway.now_us = now_us;
    if (s_gateway.state != DCU_GATEWAY_READY && s_gateway.state != DCU_GATEWAY_RUNNING) return;
    if (s_gateway.state == DCU_GATEWAY_READY && s_gateway.session_id &&
        now_us - s_gateway.accepted_us >= (uint64_t)s_gateway.profile.activation_wait_s * 1000000U) {
        leave_ready(GW_AUTHORIZATION_LOST, DCU_GATEWAY_REASON_PROTOCOL_MISMATCH);
    } else if (now_us - s_gateway.last_reply_us >= ready_limit_us()) {
        leave_ready(GW_AUTHORIZATION_LOST, DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT);
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
            leave_ready(GW_FATAL, DCU_GATEWAY_REASON_COUNTER_EXHAUSTED);
            return false;
        }
        ++s_gateway.ready_generation;
        advance(GW_FRESH_FEEDBACK);
        s_gateway.reason = DCU_GATEWAY_REASON_NONE;
    }
    s_gateway.have_reply = true;
    s_gateway.last_reply_us = reply_us;
    return true;
}

void dcu_gateway_link_lost(dcu_gateway_reason_t reason, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    leave_ready(GW_AUTHORIZATION_LOST, reason);
}

bool dcu_gateway_retry_preparation(uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_HOLD) return false;
    s_gateway.prepared_since_us = now_us;
    advance(GW_CAN_HEALTHY);
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
    if (!s_gateway.issue_session) return result;
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
            leave_ready(GW_FATAL, DCU_GATEWAY_REASON_COUNTER_EXHAUSTED);
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
    if (s_gateway.state != DCU_GATEWAY_READY || s_gateway.issue_session) return false;
    advance(GW_OP_READY);
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
    advance(GW_POLICY_APPLIED);
    s_gateway.reason = DCU_GATEWAY_REASON_NONE;
    return true;
}

bool dcu_gateway_start_run(uint64_t session_id, uint64_t now_us)
{
    dcu_gateway_tick(now_us);
    if (s_gateway.state != DCU_GATEWAY_READY || !session_id || session_id != s_gateway.session_id)
        return false;
    advance(GW_EXPLICIT_RUN);
    return true;
}

dcu_gateway_state_t dcu_gateway_current_state(uint64_t now_us, uint64_t *session_id)
{
    dcu_gateway_tick(now_us);
    if (session_id) *session_id = s_gateway.session_id;
    return s_gateway.state;
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
