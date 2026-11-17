#ifndef DCU_REALTIME_PORT_H
#define DCU_REALTIME_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t dcu_time_now_us(void);
/* Hardware elapsed time since BSP timebase init, modulo 2^32 milliseconds.
 * Independent of LED, RTOS ticks and IRQ delivery. Valid in task and ISR. */
uint32_t dcu_time_now_ms(void);
bool dcu_tx_inhibited(void);
void dcu_tx_inhibit(void);
void dcu_tx_allow(void);

#ifdef __cplusplus
}
#endif
#endif
