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
    DCU_CAN_TX_EOF_GAP_US = 50,
};

typedef struct {
    unsigned bus;
    uint32_t nominal_bitrate_bps;
    uint32_t data_bitrate_bps; /* Zero for classic CAN. */
    bool fd;
    bool timestamp_eof; /* Report hardware TX and RX frame-end timestamps. */
} dcu_can_channel_config_t;

typedef uint64_t dcu_can_tx_id_t;
typedef struct {
    unsigned owner;
    const dcu_can_frame_t *frame;
} dcu_can_batch_item_t;

typedef struct {
    const dcu_can_batch_item_t *items;
    unsigned count;
    dcu_can_tx_id_t first_id; /* Output: zero if this bus did not start. */
    uint32_t submit_us;       /* First request time when batch sampling is enabled. */
} dcu_can_bus_batch_t;

/* Hardware frame-end ticks are only comparable on the same controller. */
typedef struct {
    uint32_t tick;
    bool valid;
    uint32_t us; /* Captured TSU tick mapped to the worker clock, not IRQ/service time. */
} dcu_can_eof_t;

typedef struct {
    uint32_t observed_ms;
    dcu_can_eof_t eof;
#if defined(CONFIG_CAN_BATCH_SAMPLE)
    uint32_t irq_us, enqueued_us, dequeue_us, dequeued_us;
    uint32_t can_irq_us; /* Time inside CAN IRQ handlers between enqueue and successful dequeue. */
#endif
} dcu_can_rx_stamp_t;

typedef enum {
    DCU_CAN_TX_COMPLETED,
    DCU_CAN_TX_CANCELLED,
} dcu_can_tx_status_t;

typedef struct {
    dcu_can_tx_id_t id;
    dcu_can_tx_status_t status;
    dcu_can_eof_t eof;
    unsigned batch_owner_plus_one; /* Zero for ordinary FIFO submissions. */
} dcu_can_tx_event_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Single worker owns admission, RX and TX event consumption. IDs are scoped
 * to the physical bus and this boot. Zero means the frame was not submitted.
 * A TX event says nothing about the validity of any received reply. */
uint32_t dcu_can_rx_drops(unsigned bus);
const dcu_can_channel_config_t *dcu_can_channel_config(unsigned bus);
uint16_t dcu_can_bus_state(uint32_t bus);
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *frame, dcu_can_rx_stamp_t *stamp);
/* Includes terminal results not yet consumed by the worker. */
/* Raw frames have no motor owner and exclude all devices until retired.
 * Passing ANY queries the entire bus (e.g. for stall detection). */
#define DCU_CAN_DEVICE_ANY UINT32_MAX
#define DCU_CAN_DEVICE_RAW (UINT32_MAX - 1U)
/* 只撤销透传；终态尚未被任务收割的槽仍算在途。 */
bool dcu_can_drain_raw(unsigned bus);
/* 查询 RAW 时只看透传槽（含待收割终态），不撤销任何帧。 */
bool dcu_can_pending(unsigned bus, unsigned device);
#if defined(CONFIG_PDO_PHASE_TRACE)
uint32_t dcu_can_observed_retx_count(void);
#endif
unsigned dcu_can_tx_free(unsigned bus);
/* Production admission: no outstanding TX and at least the configured gap
 * after the last TX EOF. Without an EOF, wait from observed retirement. */
bool dcu_can_tx_serial_ready(unsigned bus);
uint32_t dcu_can_tx_progress(unsigned bus);
/* Raw requests retired as cancelled by the controller. */
uint32_t dcu_can_raw_tx_cancelled(unsigned bus);
void dcu_can_cancel(unsigned bus);
/* Bus-wide revocation from the watchdog IRQ or with global IRQs masked. */
void dcu_can_cancel_from_isr(unsigned bus);
bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id);
dcu_can_tx_id_t dcu_can_submit(unsigned bus, unsigned device, const dcu_can_frame_t *frame);
/* All or none. Contiguous IDs identify individual results; only the first TXBAR is issued here. */
dcu_can_tx_id_t dcu_can_submit_batch(unsigned bus, const dcu_can_batch_item_t *items, unsigned count,
                                     uint32_t cutoff_us);
/* One entry per bus; zero count skips it. Start ready buses before any successor can launch. */
void dcu_can_submit_batches(dcu_can_bus_batch_t batches[DCU_CAN_BUS_COUNT], uint32_t cutoff_us);
void dcu_can_gap_elapsed_isr(void);
/* Consume one terminal event. CANCELLED means revoked, not that the frame
 * could not have appeared on the bus during a cancellation race. */
bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event);

#ifdef __cplusplus
}
#endif
#endif
