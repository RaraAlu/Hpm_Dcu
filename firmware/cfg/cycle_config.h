#ifndef DCU_CYCLE_CONFIG_H
#define DCU_CYCLE_CONFIG_H

#define DCU_SYNC_MIN_PERIOD_NS 1000000U
#define DCU_SYNC_MAX_PERIOD_NS 5000000U
#define DCU_WORKER_PERIOD_US 50U
/* SM2-synchronous operation expects a 1 kHz master, with no Sync0 pulses. */
#define DCU_SM2_CYCLE_NS 1000000U
#define DCU_EVENT_PDO_MAX_BYTES (24U + 4U * 64U)

/* Must precede the master's next input read, not merely the next Sync0. */
#ifndef DCU_TX_PUBLISH_ADVANCE_US
#define DCU_TX_PUBLISH_ADVANCE_US 250U
#endif
/* Include worker dispatch and preemption before the unchanged publish deadline. */
#define DCU_TX_BUILD_BUDGET_US 600U
/* The DC motor master schedules LRW ahead of Sync0 (375 us phase contract).
 * End the causal publish window before that wire read, not at Sync0. */
#define DCU_CYCLIC_SM3_ADVANCE_US 400U

#define DCU_IRQ_SYNC0_PRIORITY 5U
#define DCU_IRQ_PDI_PRIORITY 4U
#define DCU_IRQ_TIMER_PRIORITY 3U
#define DCU_IRQ_CAN_PRIORITY 2U
#define DCU_IRQ_WORKER_PRIORITY 1U
#define DCU_IRQ_KERNEL_PRIORITY 3U

#if DCU_IRQ_SYNC0_PRIORITY <= DCU_IRQ_PDI_PRIORITY || \
    DCU_IRQ_PDI_PRIORITY <= DCU_IRQ_KERNEL_PRIORITY || \
    DCU_IRQ_CAN_PRIORITY > DCU_IRQ_KERNEL_PRIORITY || \
    DCU_IRQ_WORKER_PRIORITY > DCU_IRQ_KERNEL_PRIORITY
#error "Cycle IRQ priorities violate the fast-ISR/kernel handoff contract"
#endif

#if DCU_WORKER_PERIOD_US == 0U || DCU_SYNC_MIN_PERIOD_NS < 1000U || \
    DCU_SYNC_MAX_PERIOD_NS < DCU_SYNC_MIN_PERIOD_NS || \
    DCU_TX_PUBLISH_ADVANCE_US < DCU_WORKER_PERIOD_US || \
    DCU_TX_PUBLISH_ADVANCE_US + DCU_TX_BUILD_BUDGET_US >= DCU_SYNC_MIN_PERIOD_NS / 1000U
#error "Invalid cycle range or feedback publication window"
#endif

#if defined(__riscv)
#define DCU_FAST_CODE __attribute__((section(".fast"), optimize("O2")))
#else
#define DCU_FAST_CODE
#endif

#endif
