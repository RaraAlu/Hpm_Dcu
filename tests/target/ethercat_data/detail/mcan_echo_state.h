#ifndef DCU_DATA_MCAN_ECHO_STATE_H
#define DCU_DATA_MCAN_ECHO_STATE_H

#include "data_contract.h"
#include "hpm_mcan_drv.h"

#define DCU_DATA_MCAN_ERROR_MASK (MCAN_EVENT_ERROR | MCAN_INT_RXFIFO0_MSG_LOST | \
    MCAN_INT_MSG_RAM_ACCESS_FAILURE | MCAN_INT_WATCHDOG_INT | MCAN_INT_ACCESS_TO_RESERVED_ADDR)

typedef enum {
    MCAN_ECHO_UNINITIALIZED, MCAN_ECHO_READY, MCAN_ECHO_INIT_FAILED, MCAN_ECHO_WRONG_MODE,
    MCAN_ECHO_CONTROLLER_ERROR, MCAN_ECHO_TX_FAILED, MCAN_ECHO_RX_MISMATCH, MCAN_ECHO_TIMEOUT
} dcu_data_mcan_status_t;

typedef struct {
    bool enabled, staged, discard, tx_done, rx_done;
    unsigned chunk, slot;
    uint32_t pending, started_us;
    uint8_t request[DCU_DATA_RX_BYTES], response[DCU_DATA_RX_BYTES];
} dcu_data_mcan_state_t;

#endif
