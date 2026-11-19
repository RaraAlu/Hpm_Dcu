#ifndef DCU_GATEWAY_READINESS_STATE_H
#define DCU_GATEWAY_READINESS_STATE_H

#include "../gateway_readiness.h"

typedef struct {
    dcu_gateway_profile_t profile;
    dcu_gateway_issue_session_fn issue_session;
    dcu_gateway_state_t state;
    dcu_gateway_reason_t reason;
    uint64_t boot_id;
    uint64_t now_us;
    uint64_t prepared_since_us;
    uint64_t last_reply_us;
    uint64_t accepted_us;
    uint64_t session_id;
    uint64_t last_session_id;
    uint32_t ready_generation;
    uint32_t last_request_id;
    uint32_t last_client_tag;
    bool have_reply;
} dcu_gateway_readiness_state_t;

#endif
