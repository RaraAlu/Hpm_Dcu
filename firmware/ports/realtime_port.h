#ifndef DCU_REALTIME_PORT_H
#define DCU_REALTIME_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t dcu_time_now_us(void);
#if defined(CONFIG_CAN_BATCH_SAMPLE)
uint32_t dcu_worker_wake_us(void);
#endif
/* Low machine timer word for sub-microsecond intervals shorter than one wrap. */
uint32_t dcu_time_ticks(void);
uint32_t dcu_time_tick_hz(void);
/* Hardware elapsed time since BSP timebase init, modulo 2^32 milliseconds.
 * Independent of LED, RTOS ticks and IRQ delivery. Valid in task and ISR. */
uint32_t dcu_time_now_ms(void);
#if defined(CONFIG_PDO_PHASE_TRACE)
/* Worker timing hook: a verified neutral reply has just been consumed. */
void dcu_trace_neutral_consumed(uint32_t consumed_us);
#endif
bool dcu_tx_inhibited(void);
bool dcu_cyclic_tx_window_current(uint32_t cutoff_us);
void dcu_notify_ecat_can_from_isr(void);
/* Caller serializes the one-shot with the CAN/gap IRQ threshold. */
void dcu_can_arm_gap_at_us(uint32_t due_us);
void dcu_tx_inhibit(void);
void dcu_tx_allow(void);

#ifdef __cplusplus
}
#endif
#endif
