#ifndef DCU_CYCLE_TIMERS_H
#define DCU_CYCLE_TIMERS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool dcu_timebase_init(void);
void dcu_worker_timer_start(void);
bool dcu_worker_timer_ack(void);

#ifdef __cplusplus
}
#endif

#endif
