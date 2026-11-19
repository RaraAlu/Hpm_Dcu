#ifndef DCU_SYNC0_HW_CAPTURE_H
#define DCU_SYNC0_HW_CAPTURE_H

#include <stdint.h>

void dcu_sync0_capture_init(void);
void dcu_sync0_capture_begin(uint32_t period_ns);
void dcu_sync0_capture_observe(uint32_t sync_events);

#endif
