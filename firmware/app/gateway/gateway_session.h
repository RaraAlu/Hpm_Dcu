#ifndef DCU_GATEWAY_SESSION_H
#define DCU_GATEWAY_SESSION_H

#include "gateway_readiness.h"

typedef enum {
    DCU_GATEWAY_ACCEPTED = 0,
    DCU_GATEWAY_NOT_READY,
    DCU_GATEWAY_CONFIG_MISMATCH,
    DCU_GATEWAY_STALE_CONTEXT,
    DCU_GATEWAY_BUSY,
    DCU_GATEWAY_INVALID,
} dcu_gateway_activation_code_t;

typedef struct {
    uint32_t profile_id;
    uint32_t revision;
    uint8_t digest[DCU_GATEWAY_DIGEST_BYTES];
    uint64_t boot_id;
    uint32_t ready_generation;
    uint32_t request_id;
    uint32_t client_tag;
    uint32_t reserved;
} dcu_gateway_activation_t;

typedef struct {
    uint32_t request_id;
    dcu_gateway_activation_code_t code;
    dcu_gateway_state_t state;
    uint64_t session_id;
} dcu_gateway_activation_result_t;

/* Bounded, nonblocking provider with no I/O: IDs must be nonzero, increasing
 * and unique across boots. Return 0 on exhaustion. */
typedef uint64_t (*dcu_gateway_issue_session_fn)(void);

#ifdef __cplusplus
extern "C" {
#endif

/* Diagnostic boot only; explicit activation is required after each loss of readiness. */
bool dcu_gateway_boot_session(const dcu_gateway_profile_t *profile, uint64_t boot_id,
                              dcu_gateway_issue_session_fn issue_session, uint64_t now_us);
/* Accepted means waiting for protocol and cycle conditions, never permission to send. */
dcu_gateway_activation_result_t dcu_gateway_activate(const dcu_gateway_activation_t *request, uint64_t now_us);
bool dcu_gateway_start_run(uint64_t session_id, uint64_t now_us);

#ifdef __cplusplus
}
#endif
#endif
