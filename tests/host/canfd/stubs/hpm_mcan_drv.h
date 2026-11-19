#ifndef TEST_CANFD_MCAN_H
#define TEST_CANFD_MCAN_H
#include <stdbool.h>
#include <stdint.h>
#include "hpm_common.h"
typedef struct { uint32_t TXBTO, TXBCF, TXBRP, TXBCR, TXFQS; } MCAN_Type;
typedef struct {
    bool use_ext_id, rtr, canfd_frame, bitrate_switch;
    uint32_t ext_id, std_id;
    uint8_t dlc, data_8[64];
} mcan_tx_frame_t;
typedef mcan_tx_frame_t mcan_rx_message_t;
typedef struct { bool in_bus_off_state, in_error_passive_state, in_warning_state; } mcan_protocol_status_t;
typedef struct {
    bool enable_std_filter, enable_ext_filter, enable_rxbuf, enable_tx_evt_fifo;
    struct { unsigned enable, elem_count, data_field_size; } rxfifos[2];
    unsigned txbuf_data_field_size, txbuf_dedicated_txbuf_elem_count;
    unsigned txbuf_fifo_or_queue_elem_count, txfifo_or_txqueue_mode;
} test_ram_config_t;
typedef struct {
    unsigned baudrate, baudrate_fd, txbuf_trans_interrupt_mask, interrupt_mask;
    bool enable_canfd, enable_tdc;
    test_ram_config_t ram_config;
    struct {
        struct { unsigned mcan_filter_elem_count; } std_id_filter_list, ext_id_filter_list;
    } all_filters_config;
} mcan_config_t;
#define MCAN_EVENT_TRANSMIT 1U
#define MCAN_EVENT_ERROR 2U
#define MCAN_INT_RXFIFO0_NEW_MSG 4U
#define MCAN_DATA_FIELD_SIZE_8BYTES 8U
#define MCAN_DATA_FIELD_SIZE_64BYTES 7U
#define MCAN_TXFQS_TFFL_GET(value) (value)
uint32_t mcan_get_interrupt_flags(MCAN_Type *base);
void mcan_clear_interrupt_flags(MCAN_Type *base, uint32_t flags);
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *base, unsigned fifo);
int mcan_read_rxfifo(MCAN_Type *base, unsigned fifo, mcan_rx_message_t *frame);
int mcan_get_protocol_status(MCAN_Type *base, mcan_protocol_status_t *status);
void mcan_get_default_config(MCAN_Type *base, mcan_config_t *config);
void mcan_get_default_ram_config(MCAN_Type *base, test_ram_config_t *config, bool fd);
int mcan_init(MCAN_Type *base, mcan_config_t *config, uint32_t clock);
void mcan_interrupt_line_select(MCAN_Type *base, uint32_t mask, unsigned line);
void mcan_cancel_tx_buf_send_request(MCAN_Type *base, unsigned index);
int mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index);
int mcan_transmit_via_txfifo_nonblocking(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index);
void mcan_send_add_request(MCAN_Type *base, uint32_t index);
#endif
