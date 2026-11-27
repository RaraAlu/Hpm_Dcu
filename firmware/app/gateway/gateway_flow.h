#ifndef DCU_GATEWAY_FLOW_H
#define DCU_GATEWAY_FLOW_H

#include "startup/gateway_readiness.h"

typedef enum {
    DCU_PDO_MOTOR = 0, DCU_PDO_MOTOR_WITH_RAW = 1, DCU_PDO_RAW = 2, DCU_PDO_SINGLE_FD = 3
} dcu_pdo_mode_t;

typedef enum {
    GW_BOOT_VALID, GW_FRESH_FEEDBACK, GW_OP_READY, GW_EXPLICIT_RUN,
    GW_AUTHORIZATION_LOST, GW_CAN_HEALTHY, GW_POLICY_APPLIED, GW_FATAL
} dcu_gateway_event_t;

/* Values 0..5 are also the legacy states reported at 0x2101:03. */
typedef enum {
    PDO_DISABLED = 0, PDO_WAIT_CONFIRM = 1, PDO_WAIT_OP = 2, PDO_ACTIVE = 3,
    PDO_DRAINING = 4, PDO_FAULTED = 5,
    PDO_WAIT_COMMIT, PDO_RECOVERY_PERMITTED, PDO_DRAINING_CONFIRMED, PDO_REFUSED
} dcu_layout_phase_t;

typedef enum {
    PDO_SDO_2101_01_REQUEST, PDO_SDO_2103_01_SELECT, PDO_CAN_DRAINED, PDO_SDO_2101_02_CONFIRM,
    PDO_PREOP_MAPPING, PDO_SDO_2103_06_COMMIT, PDO_ENTER_OP, PDO_LEAVE_OP,
    PDO_DRAIN_TIMEOUT, PDO_TOPOLOGY_TIMEOUT, PDO_DROP_LIMIT, PDO_ABORT, PDO_FIXED_MAILBOX_STOP
} dcu_layout_event_t;

typedef enum { PDO_WAIT_BASELINE, PDO_CYCLIC, PDO_FRAME_REFUSED } dcu_session_phase_t;
typedef enum {
    PDO_REVOKE_SESSION, PDO_ZERO_ENABLE_BASELINE, PDO_EMPTY_FD_BASELINE,
    PDO_STOP_WITH_TARGETS, PDO_OTHER_FRAME
} dcu_session_event_t;

#ifdef __cplusplus
extern "C" {
#endif
/* Prospective transitions: callers only store an accepted phase after the operation succeeds.
 * PDO_REFUSED and PDO_FRAME_REFUSED are results, never stored phases. */
dcu_gateway_state_t dcu_gateway_next(dcu_gateway_state_t state, dcu_gateway_event_t event,
                                     dcu_gateway_reason_t reason);
dcu_layout_phase_t dcu_layout_next(dcu_layout_phase_t phase, dcu_layout_event_t event,
                                   dcu_pdo_mode_t mode, bool topology_pending, bool operational);
dcu_session_phase_t dcu_session_next(dcu_session_phase_t phase, dcu_session_event_t event,
                                     bool operational_and_gateway_running);
#ifdef __cplusplus
}
#endif
#endif
