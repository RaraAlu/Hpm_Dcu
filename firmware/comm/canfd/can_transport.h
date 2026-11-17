#ifndef DCU_CAN_TRANSPORT_H
#define DCU_CAN_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>
#include "can_frame.h"

enum {
    DCU_CAN_BUS_ERROR_ACTIVE = 0,
    DCU_CAN_BUS_ERROR_WARNING = 1,
    DCU_CAN_BUS_ERROR_PASSIVE = 2,
    DCU_CAN_BUS_OFF = 3,
};

#ifdef __cplusplus
extern "C" {
#endif

/* Single worker owns RX consumption and TX submission. IRQs produce RX and
 * completion counts; cancel is also allowed from the independent watchdog.
 * Bus and axis arguments use the product topology, not an EtherCAT layout. */
uint32_t dcu_can_rx_drops(unsigned bus);
uint32_t dcu_can_completions(unsigned axis);
uint16_t dcu_can_bus_state(uint32_t bus);
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, uint32_t *stamp_ms);
bool dcu_can_pending(unsigned bus);
bool dcu_can_axis_pending(unsigned axis);
unsigned dcu_can_tx_free(unsigned bus);
uint32_t dcu_can_tx_progress(unsigned bus);
void dcu_can_cancel(unsigned bus);
void dcu_can_cancel_axis(unsigned axis);
void dcu_can_cancel_service(unsigned bus);
bool dcu_can_send(unsigned bus, const dcu_can_frame_t *frame, unsigned axis);

#ifdef __cplusplus
}
#endif
#endif
