#ifndef DCU_CYCLE_CONFIG_H
#define DCU_CYCLE_CONFIG_H

#define DCU_SYNC_MIN_PERIOD_NS 1000000U
#define DCU_SYNC_MAX_PERIOD_NS 5000000U
#define DCU_WORKER_PERIOD_US 50U

/* Must precede the master's next input read, not merely the next Sync0. */
#ifndef DCU_TX_PUBLISH_ADVANCE_US
#define DCU_TX_PUBLISH_ADVANCE_US 250U
#endif
#define DCU_TX_BUILD_BUDGET_US 100U

#define DCU_IRQ_SYNC0_PRIORITY 5U
#define DCU_IRQ_PDI_PRIORITY 4U
#define DCU_IRQ_TIMER_PRIORITY 3U
#define DCU_IRQ_CAN_PRIORITY 2U
#define DCU_IRQ_WORKER_PRIORITY 1U

#if defined(__riscv)
#define DCU_FAST_CODE __attribute__((section(".fast"), optimize("O2")))
#else
#define DCU_FAST_CODE
#endif

#endif
