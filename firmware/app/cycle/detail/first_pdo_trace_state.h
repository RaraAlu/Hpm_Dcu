#ifndef DCU_FIRST_PDO_TRACE_STATE_H
#define DCU_FIRST_PDO_TRACE_STATE_H

#include <stdint.h>

#define DCU_FIRST_PDO_EVENT_LIMIT 256U
#define DCU_FIRST_PDO_CAPTURE_US 8000U

typedef struct {
    uint32_t time_us, event, a, b, c, d;
} dcu_first_pdo_trace_entry_t;

typedef struct {
    uint32_t armed_us, sequence, count;
    dcu_first_pdo_trace_entry_t entry[DCU_FIRST_PDO_EVENT_LIMIT];
} dcu_first_pdo_trace_t;

#endif
