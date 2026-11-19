#ifndef DCU_CAN_INTERNAL_LOOPBACK_H
#define DCU_CAN_INTERNAL_LOOPBACK_H

#include <stdbool.h>
#include <stdint.h>
#include "cfg/can_loopback_config.h"

typedef enum {
    DCU_LOOPBACK_NOT_RUN = 0,
    DCU_LOOPBACK_RUNNING,
    DCU_LOOPBACK_PASS,
    DCU_LOOPBACK_TIMEBASE_FAILED,
    DCU_LOOPBACK_SETUP_FAILED,
    DCU_LOOPBACK_MODE_FAILED,
    DCU_LOOPBACK_TX_FAILED,
    DCU_LOOPBACK_RX_MISMATCH,
    DCU_LOOPBACK_TIMEOUT,
    DCU_LOOPBACK_IRQ_FAILED,
    DCU_LOOPBACK_QUEUE_OVERFLOW,
    DCU_LOOPBACK_EXTRA_RX,
    DCU_LOOPBACK_STOP_FAILED,
} dcu_can_loopback_result_t;

typedef struct {
    uint32_t elapsed_us;
    uint32_t received_id;
    uint8_t received_dlc;
    uint8_t received_flags;
    uint8_t received_data[8];
    bool matched;
} dcu_can_loopback_frame_result_t;

typedef struct {
    dcu_can_loopback_result_t result;
    uint32_t sdk_status;
    uint32_t frames_checked;
    uint32_t irq_count;
    uint32_t tx_completed;
    uint32_t rx_received;
    uint32_t queue_drops;
    uint32_t error_flags;
    uint32_t mode_cccr;
    uint32_t mode_test;
    bool controller_stopped;
    dcu_can_loopback_frame_result_t frames[DCU_LOOPBACK_FRAME_COUNT];
} dcu_can_loopback_report_t;

/* One-shot diagnostic after board_init(), never a production transport entry. */
void dcu_can_loopback_run(void);
const volatile dcu_can_loopback_report_t *dcu_can_loopback_report(void);
void dcu_can_loopback_finished(void);

#endif
