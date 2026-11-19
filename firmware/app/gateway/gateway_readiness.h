#ifndef DCU_GATEWAY_READINESS_H
#define DCU_GATEWAY_READINESS_H

#include <stdbool.h>
#include <stdint.h>

enum { DCU_GATEWAY_DIGEST_BYTES = 32 };

typedef enum {
    DCU_GATEWAY_BOOT = 0,
    DCU_GATEWAY_PREPARING,
    DCU_GATEWAY_READY,
    DCU_GATEWAY_RUNNING,
    DCU_GATEWAY_HOLD,
    DCU_GATEWAY_FAULT,
} dcu_gateway_state_t;

typedef enum {
    DCU_GATEWAY_ACCEPTED = 0,
    DCU_GATEWAY_NOT_READY,
    DCU_GATEWAY_CONFIG_MISMATCH,
    DCU_GATEWAY_STALE_CONTEXT,
    DCU_GATEWAY_BUSY,
    DCU_GATEWAY_INVALID,
} dcu_gateway_activation_code_t;

typedef enum {
    DCU_GATEWAY_REASON_NONE = 0,
    DCU_GATEWAY_REASON_CONFIG_INVALID,
    DCU_GATEWAY_REASON_CAN_OFFLINE,
    DCU_GATEWAY_REASON_FEEDBACK_TIMEOUT,
    DCU_GATEWAY_REASON_MASTER_TIMEOUT,
    DCU_GATEWAY_REASON_PROTOCOL_MISMATCH,
    DCU_GATEWAY_REASON_INVALID_COMMAND,
    DCU_GATEWAY_REASON_BUSY,
    DCU_GATEWAY_REASON_COUNTER_EXHAUSTED,
} dcu_gateway_reason_t;

typedef struct {
    uint32_t profile_id;
    uint32_t revision;
    uint8_t digest[DCU_GATEWAY_DIGEST_BYTES];
    uint32_t readiness_ttl_ms;
    uint32_t activation_wait_s;
} dcu_gateway_profile_t;

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

typedef struct {
    dcu_gateway_state_t state;
    dcu_gateway_reason_t reason;
    uint32_t profile_id;
    uint32_t revision;
    uint8_t digest[DCU_GATEWAY_DIGEST_BYTES];
    uint64_t boot_id;
    uint32_t ready_generation;
    uint32_t active_request_id;
    uint64_t session_id;
    uint32_t client_tag;
    uint32_t feedback_age_us;
} dcu_gateway_readiness_t;

/* Bounded, nonblocking provider with no I/O: IDs must be nonzero, increasing
 * and unique across boots. Return 0 on exhaustion. */
typedef uint64_t (*dcu_gateway_issue_session_fn)(void);

#ifdef __cplusplus
extern "C" {
#endif

/* Single task owner, not ISR-safe. Native values only; none are wire structs.
 * Call boot once per reset. boot_id must identify this boot uniquely. */
bool dcu_gateway_boot(const dcu_gateway_profile_t *profile, uint64_t boot_id,
                      dcu_gateway_issue_session_fn issue_session, uint64_t now_us);
void dcu_gateway_tick(uint64_t now_us);

/* Only the adapter that matched the expected probe may submit evidence.
 * Both timestamps are from the same monotonic clock. */
bool dcu_gateway_probe_verified(uint64_t reply_us, uint64_t now_us);
void dcu_gateway_link_lost(dcu_gateway_reason_t reason, uint64_t now_us);
bool dcu_gateway_retry_preparation(uint64_t now_us);

/* Accepted means waiting for protocol and cycle conditions, never permission to send. */
dcu_gateway_activation_result_t dcu_gateway_activate(const dcu_gateway_activation_t *request, uint64_t now_us);
/* For the fixed Profile, OP permits PDO reception; the application gates motion. */
bool dcu_gateway_resume_static(uint64_t now_us);
bool dcu_gateway_apply_policy(uint32_t profile_id, uint32_t base_revision, uint32_t revision,
                              uint32_t ttl_ms, uint64_t now_us);
/* The activation-based diagnostic adapter separately verifies its startup conditions. */
bool dcu_gateway_start_run(uint64_t session_id, uint64_t now_us);
dcu_gateway_readiness_t dcu_gateway_readiness(uint64_t now_us);

#ifdef __cplusplus
}
#endif

#endif
