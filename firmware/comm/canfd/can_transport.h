#ifndef DCU_CAN_TRANSPORT_H
#define DCU_CAN_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include "can_frame.h"

enum {
    DCU_CAN_BUS_COUNT = 4,
    DCU_CAN_BUS_ERROR_ACTIVE = 0,
    DCU_CAN_BUS_ERROR_WARNING = 1,
    DCU_CAN_BUS_ERROR_PASSIVE = 2,
    DCU_CAN_BUS_OFF = 3,
    DCU_CAN_BUS_DISABLED = 4,
};

typedef struct {
    unsigned bus;
    uint32_t nominal_bitrate_bps;
    uint32_t data_bitrate_bps; /* Zero for classic CAN. */
    bool fd;
} dcu_can_channel_config_t;

typedef uint64_t dcu_can_tx_id_t;

typedef enum {
    DCU_CAN_TX_COMPLETED,
    DCU_CAN_TX_CANCELLED,
} dcu_can_tx_status_t;

typedef struct {
    dcu_can_tx_id_t id;
    dcu_can_tx_status_t status;
} dcu_can_tx_event_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Single worker owns submission, RX and TX event consumption. IDs are scoped
 * to the physical bus and this boot. Zero means the frame was not submitted.
 * A TX event says nothing about the validity of any received reply. */
uint32_t dcu_can_rx_drops(unsigned bus);
const dcu_can_channel_config_t *dcu_can_channel_config(unsigned bus);
uint16_t dcu_can_bus_state(uint32_t bus);
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, uint32_t *stamp_ms);
/* Includes terminal results not yet consumed by the worker. */
bool dcu_can_pending(unsigned bus);
unsigned dcu_can_tx_free(unsigned bus);
uint32_t dcu_can_tx_progress(unsigned bus);
void dcu_can_cancel(unsigned bus);
/* Bus-wide revocation from the watchdog IRQ or with global IRQs masked. */
void dcu_can_cancel_from_isr(unsigned bus);
bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id);
dcu_can_tx_id_t dcu_can_submit(unsigned bus, const dcu_can_frame_t *frame);
/* Consume one terminal event. CANCELLED means revoked, not that the frame
 * could not have appeared on the bus during a cancellation race. */
bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event);

#ifdef __cplusplus
}
#endif
#endif
