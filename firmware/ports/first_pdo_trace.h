#ifndef DCU_FIRST_PDO_TRACE_H
#define DCU_FIRST_PDO_TRACE_H

#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
#include <stdbool.h>
#include <stdint.h>

enum dcu_first_pdo_event {
    DCU_FIRST_SM2 = 1, DCU_FIRST_LATCH, DCU_FIRST_AXIS_ACCEPT,
    DCU_FIRST_SCHEDULE, DCU_FIRST_TX_REQUEST, DCU_FIRST_TX_EOF, DCU_FIRST_RX_EOF,
    DCU_FIRST_RX_CONSUMED, DCU_FIRST_SM3_BEGIN, DCU_FIRST_SM3_PACKED,
    DCU_FIRST_SM3_WRITTEN, DCU_FIRST_SM3_MARK, DCU_FIRST_FAULT_BIT,
    DCU_FIRST_WATCHDOG, DCU_FIRST_WORKER_FAULT, DCU_FIRST_DC_ANCHOR, DCU_FIRST_NOT_RUNNING,
    DCU_FIRST_ENCODE_FAILED, DCU_FIRST_SUBMIT_FAILED,
};

enum dcu_first_can_gate {
    DCU_FIRST_CAN_READY, DCU_FIRST_CAN_FIFO_FULL, DCU_FIRST_CAN_PENDING,
    DCU_FIRST_CAN_EOF_GAP, DCU_FIRST_CAN_OBSERVED_GAP,
};

#ifdef __cplusplus
extern "C" {
#endif
bool dcu_first_pdo_trace_active(void);
void dcu_first_pdo_trace_at(uint32_t time_us, uint32_t event, uint32_t a, uint32_t b, uint32_t c, uint32_t d);
uint32_t dcu_first_pdo_can_gate(void);
#ifdef __cplusplus
}
#endif

#define DCU_FIRST_TRACE_AT(us, event, a, b, c, d) dcu_first_pdo_trace_at(us, event, a, b, c, d)
#define DCU_FIRST_TRACE(event, a, b, c, d) DCU_FIRST_TRACE_AT(dcu_time_now_us(), event, a, b, c, d)
#else
#define DCU_FIRST_TRACE_AT(...) ((void)0)
#define DCU_FIRST_TRACE(...) ((void)0)
#endif

#endif
