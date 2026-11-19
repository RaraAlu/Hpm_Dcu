#ifndef TEST_LOOPBACK_MCAN_H
#define TEST_LOOPBACK_MCAN_H

#include <stdbool.h>
#include <stdint.h>

typedef int hpm_stat_t;
#define status_success 0
#define MCAN_SOC_MSG_BUF_IN_AHB_RAM 1
#define MCAN_MSG_BUF_SIZE_IN_WORDS 640U
#define ATTR_PLACE_AT(name) __attribute__((section(name)))
#define MCAN_CCCR_INIT_MASK 1U
#define MCAN_CCCR_MON_MASK 2U
#define MCAN_CCCR_TEST_MASK 4U
#define MCAN_CCCR_FDOE_MASK 8U
#define MCAN_CCCR_BRSE_MASK 16U
#define MCAN_TEST_LBCK_MASK 16U
#define MCAN_INT_RXFIFO0_NEW_MSG 1U
#define MCAN_EVENT_TRANSMIT 2U
#define MCAN_EVENT_ERROR 4U
#define MCAN_INT_RXFIFO0_MSG_LOST 8U
#define MCAN_INT_MSG_RAM_ACCESS_FAILURE 16U
#define MCAN_INT_WATCHDOG_INT 32U
#define MCAN_INT_ACCESS_TO_RESERVED_ADDR 64U
#define MCAN_DATA_FIELD_SIZE_8BYTES 0U
#define MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0 0U
#define MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT 2U

typedef struct { volatile uint32_t CCCR, TEST, TXBTO; } MCAN_Type;
typedef enum { mcan_mode_normal, mcan_mode_loopback_internal } mcan_node_mode_t;
typedef struct { uint32_t ram_base, ram_size; } mcan_msg_buf_attr_t;
typedef struct {
    bool use_ext_id, rtr, canfd_frame, bitrate_switch;
    uint32_t ext_id, std_id;
    uint8_t dlc, data_8[64];
} mcan_tx_frame_t;
typedef mcan_tx_frame_t mcan_rx_message_t;
typedef struct {
    bool enable_std_filter, enable_ext_filter, enable_rxbuf, enable_tx_evt_fifo;
    struct { unsigned enable, elem_count, data_field_size; } rxfifos[2];
    unsigned txbuf_data_field_size, txbuf_dedicated_txbuf_elem_count;
    unsigned txbuf_fifo_or_queue_elem_count, txfifo_or_txqueue_mode;
} mcan_ram_config_t;
typedef struct {
    mcan_node_mode_t mode;
    unsigned baudrate, txbuf_trans_interrupt_mask, interrupt_mask;
    bool enable_canfd, enable_tdc;
    mcan_ram_config_t ram_config;
    struct {
        struct { unsigned mcan_filter_elem_count; } std_id_filter_list, ext_id_filter_list;
        struct {
            bool reject_remote_std_frame, reject_remote_ext_frame;
            unsigned accept_non_matching_std_frame_option, accept_non_matching_ext_frame_option;
        } global_filter_config;
    } all_filters_config;
} mcan_config_t;

uint32_t mcan_get_interrupt_flags(MCAN_Type *base);
void mcan_clear_interrupt_flags(MCAN_Type *base, uint32_t flags);
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *base, unsigned fifo);
hpm_stat_t mcan_read_rxfifo(MCAN_Type *base, unsigned fifo, mcan_rx_message_t *frame);
hpm_stat_t mcan_set_msg_buf_attr(MCAN_Type *base, const mcan_msg_buf_attr_t *attr);
void mcan_get_default_config(MCAN_Type *base, mcan_config_t *config);
void mcan_get_default_ram_config(MCAN_Type *base, mcan_ram_config_t *config, bool fd);
hpm_stat_t mcan_init(MCAN_Type *base, const mcan_config_t *config, uint32_t clock_hz);
void mcan_interrupt_line_select(MCAN_Type *base, uint32_t mask, unsigned line);
hpm_stat_t mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *slot);
void mcan_send_add_request(MCAN_Type *base, uint32_t slot);
void mcan_disable_interrupts(MCAN_Type *base, uint32_t mask);
void mcan_cancel_tx_buf_send_request(MCAN_Type *base, unsigned index);
void mcan_enter_init_mode(MCAN_Type *base);

#endif
