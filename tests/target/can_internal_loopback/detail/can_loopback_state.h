#ifndef DCU_CAN_LOOPBACK_STATE_H
#define DCU_CAN_LOOPBACK_STATE_H

#include "can_internal_loopback.h"
#include "hpm_mcan_drv.h"

#define DCU_LOOPBACK_ERROR_MASK (MCAN_EVENT_ERROR | MCAN_INT_RXFIFO0_MSG_LOST | \
    MCAN_INT_MSG_RAM_ACCESS_FAILURE | MCAN_INT_WATCHDOG_INT | MCAN_INT_ACCESS_TO_RESERVED_ADDR)
#define DCU_LOOPBACK_IRQ_MASK (MCAN_INT_RXFIFO0_NEW_MSG | MCAN_EVENT_TRANSMIT | DCU_LOOPBACK_ERROR_MASK)

typedef struct {
    mcan_rx_message_t frame;
    uint32_t arrival_us;
} dcu_loopback_rx_item_t;

typedef struct {
    dcu_loopback_rx_item_t items[DCU_LOOPBACK_QUEUE_DEPTH];
    volatile uint32_t read;
    volatile uint32_t write;
    volatile uint32_t irq_count;
    volatile uint32_t tx_completed;
    volatile uint32_t rx_received;
    volatile uint32_t queue_drops;
    volatile uint32_t error_flags;
    volatile uint32_t pending_mask;
    volatile bool read_failed;
} dcu_loopback_irq_state_t;

#endif
