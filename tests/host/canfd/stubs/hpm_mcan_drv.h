#ifndef TEST_CANFD_MCAN_H
#define TEST_CANFD_MCAN_H
#include <stdbool.h>
#include <stdint.h>
#include "hpm_common.h"
typedef struct {
    uint32_t TXBTO, TXBCF, TXBRP, TXBCR, TXFQS, CCCR, TSCFG, TSS1;
    uint32_t IR, PSR, ECR, TXEFS, RXF0S, NBTP, DBTP, TDCR;
} MCAN_Type;
typedef struct {
    bool use_ext_id, rtr, canfd_frame, bitrate_switch;
    uint32_t ext_id, std_id;
    uint8_t dlc, data_8[64];
    bool timestamp_capture_enable, event_fifo_control, rx_timestamp_captured;
    uint8_t message_marker_l, rx_timestamp_pointer;
} mcan_tx_frame_t;
typedef mcan_tx_frame_t mcan_rx_message_t;
typedef struct {
    uint8_t event_type, message_marker, tx_timestamp_pointer;
    bool tx_timestamp_captured;
} mcan_tx_event_fifo_elem_t;
typedef struct {
    uint8_t filter_type, filter_config, can_id_type, sync_message;
    uint32_t filter_id, filter_mask;
} mcan_filter_elem_t;
typedef struct {
    bool enable_std_filter, enable_ext_filter, enable_rxbuf, enable_tx_evt_fifo;
    struct { unsigned enable, elem_count, data_field_size; } rxfifos[2];
    unsigned txbuf_data_field_size, txbuf_dedicated_txbuf_elem_count;
    unsigned txbuf_fifo_or_queue_elem_count, txfifo_or_txqueue_mode;
    unsigned tx_evt_fifo_elem_count, tx_evt_fifo_watermark;
    unsigned std_filter_elem_count;
} test_ram_config_t;
typedef struct {
    unsigned baudrate, baudrate_fd, txbuf_trans_interrupt_mask, interrupt_mask;
    bool enable_canfd, enable_tdc, disable_auto_retransmission;
    bool use_timestamping_unit;
    struct { unsigned ssp_offset, filter_window_length; } tdc_config;
    struct { unsigned timestamp_selection; } timestamp_cfg;
    struct { bool enable_tsu, capture_on_sof, use_ext_timebase, enable_64bit_timestamp; } tsu_config;
    test_ram_config_t ram_config;
    struct {
        struct {
            unsigned mcan_filter_elem_count;
            const mcan_filter_elem_t *filter_elem_list;
        } std_id_filter_list, ext_id_filter_list;
    } all_filters_config;
} mcan_config_t;
#define MCAN_EVENT_TRANSMIT 1U
#define MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE 0x10000000U
#define MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE 0x08000000U
#define MCAN_EVENT_ERROR (2U | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE | MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE)
#define MCAN_INT_RXFIFO0_NEW_MSG 4U
#define MCAN_DATA_FIELD_SIZE_8BYTES 8U
#define MCAN_DATA_FIELD_SIZE_64BYTES 7U
#define MCAN_TXFQS_TFFL_GET(value) (value)
#define MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED 2U
#define MCAN_FILTER_TYPE_CLASSIC_FILTER 2U
#define MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH 1U
#define MCAN_CAN_ID_TYPE_STANDARD 0U
#define MCAN_CCCR_DAR_MASK 0x40U
#define MCAN_CCCR_UTSU_MASK 0x400U
#define MCAN_TSCFG_TSUE_MASK 0x1U
#define MCAN_TSCFG_TBCS_MASK 0x2U
#define MCAN_TSCFG_SCP_MASK 0x4U
#define MCAN_TSCFG_EN64_MASK 0x8U
#define MCAN_PSR_BO_MASK 0x80U
#define MCAN_PSR_EW_MASK 0x40U
#define MCAN_PSR_EP_MASK 0x20U
#define MCAN_PSR_LEC_GET(psr) ((psr) & 7U)
#define MCAN_PSR_DLEC_GET(psr) (((psr) >> 8) & 7U)
uint32_t mcan_get_interrupt_flags(MCAN_Type *base);
void mcan_clear_interrupt_flags(MCAN_Type *base, uint32_t flags);
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *base, unsigned fifo);
int mcan_read_rxfifo(MCAN_Type *base, unsigned fifo, mcan_rx_message_t *frame);
int mcan_read_tx_evt_fifo(MCAN_Type *base, mcan_tx_event_fifo_elem_t *event);
bool mcan_is_tsu_timestamp_available(MCAN_Type *base, unsigned slot);
bool mcan_is_tsu_used(MCAN_Type *base);
uint32_t mcan_read_32bit_tsu_timestamp(MCAN_Type *base, uint32_t slot);
void mcan_get_default_config(MCAN_Type *base, mcan_config_t *config);
void mcan_get_default_ram_config(MCAN_Type *base, test_ram_config_t *config, bool fd);
int mcan_init(MCAN_Type *base, mcan_config_t *config, uint32_t clock);
void mcan_interrupt_line_select(MCAN_Type *base, uint32_t mask, unsigned line);
void mcan_cancel_tx_buf_send_request(MCAN_Type *base, unsigned index);
int mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index);
int mcan_transmit_via_txfifo_nonblocking(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index);
void mcan_send_add_request(MCAN_Type *base, uint32_t index);
#endif
