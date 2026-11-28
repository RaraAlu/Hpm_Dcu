#ifndef DCU_CAN_GATEWAY_STATE_H
#define DCU_CAN_GATEWAY_STATE_H

#include <stdint.h>
#include "can_transport.h"
#include "cfg/can_bus_config.h"
#include "hpm_gpio_drv.h"

typedef struct {
    dcu_can_frame_t frame;
    dcu_can_rx_stamp_t stamp;
#if defined(CONFIG_PDO_PHASE_TRACE)
    uint32_t observed_us;
#endif
} rx_item_t;

typedef struct {
    rx_item_t item[32];
    volatile uint8_t read, write;
} rx_queue_t;

typedef struct {
    dcu_can_tx_id_t id;
    unsigned in_flight_device;
    bool revoked, feedback, launched;
    uint32_t wire_id;
    dcu_can_eof_t eof;
} tx_slot_t;

typedef struct {
    unsigned next, count, unretired;
    uint32_t cutoff_us, eof_tick;
    bool armed;
} paced_tx_t;

typedef struct {
    uint32_t hz, samples, min_gap_ticks, completed_without_eof;
    uint32_t completed[CANFD_TX_FIFO_DEPTH];
} feedback_stats_t;

typedef struct {
    dcu_can_eof_t eof;
    uint32_t observed_us;
    bool seen;
} tx_retirement_t;

typedef enum {
    CANFD_FAULT_FROM_IRQ = 1,
    CANFD_FAULT_FROM_STATE_READ,
    CANFD_FAULT_BEFORE_TX_EVENT,
    CANFD_FAULT_BEFORE_CANCEL,
} canfd_fault_source_t;

typedef struct {
    uint32_t valid, observed_us, source;
    uint32_t ir, psr, ecr;
    uint32_t txbrp, txbto, txbcf, txefs, rxf0s;
    uint32_t nbtp, dbtp, tdcr, cccr;
    dcu_can_tx_id_t tx_id[CANFD_TX_SLOTS];
} canfd_first_fault_t;

typedef struct {
    GPIO_Type *ctrl;
    uint32_t index;
    uint32_t pin;
    uint8_t on_level;
    uint8_t off_level;
} canfd_led_t;

enum {
    CANFD_LED_TX = 0,
    CANFD_LED_RX,
    CANFD_LED_ERR,
    CANFD_LED_KIND_COUNT,
};

#endif
