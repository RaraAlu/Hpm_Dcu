#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../target/can_internal_loopback/can_internal_loopback.c"

static const char *scenario;
static MCAN_Type controller;
static bool irq_enabled, irq_active, peripheral_irq, late_sent;
static bool deferred_tx, deferred_rx;
static uint32_t deferred_at_us;
static uint32_t now_us, flags, send_count, cancel_count, clock_count, init_count, next_slot;
static unsigned fifo_count, fifo_read;
static mcan_rx_message_t fifo[8];
static mcan_tx_frame_t staged;

static bool is(const char *name) { return strcmp(scenario, name) == 0; }
MCAN_Type *test_loopback_controller(void) { return &controller; }

static void dispatch(void)
{
    if (irq_enabled && peripheral_irq && flags && !irq_active && !is("no_irq")) {
        irq_active = true;
        registered_handler();
        irq_active = false;
    }
}

static void enqueue(unsigned count)
{
    fifo_read = 0;
    fifo_count = count;
    for (unsigned i = 0; i < count; ++i) fifo[i] = staged;
    if (is("bad_data")) fifo[0].data_8[3] ^= 1U;
    if (is("bad_id")) fifo[0].std_id = 0x601U;
    if (is("bad_dlc")) fifo[0].dlc = 9U;
    if (is("short_dlc")) fifo[0].dlc = 7U;
    if (is("fd")) fifo[0].canfd_frame = true;
    if (is("brs")) fifo[0].bitrate_switch = true;
    if (is("rtr")) fifo[0].rtr = true;
    if (is("extended")) fifo[0].use_ext_id = true;
    if (count) flags |= MCAN_INT_RXFIFO0_NEW_MSG;
}

bool dcu_timebase_init(void) { return !is("timebase_failure"); }
uint32_t dcu_time_now_us(void)
{
    if (!is("frozen_clock") && !(is("quiet_clock_failure") &&
        s_report.frames_checked == DCU_LOOPBACK_FRAME_COUNT)) ++now_us;
    if (!irq_active && (deferred_tx || deferred_rx) && now_us >= deferred_at_us) {
        if (deferred_tx) flags |= MCAN_EVENT_TRANSMIT;
        if (deferred_rx) enqueue(1U);
        deferred_tx = deferred_rx = false;
        dispatch();
    }
    if (is("late_rx") && !irq_active && !late_sent && s_report.frames_checked == DCU_LOOPBACK_FRAME_COUNT) {
        late_sent = true;
        enqueue(1U);
        dispatch();
    }
    return now_us;
}

uint32_t board_init_can_clock(MCAN_Type *base)
{
    assert(base == &controller);
    ++clock_count;
    return is("clock_failure") ? 0U : 80000000U;
}

void intc_m_disable_irq(unsigned irq)
{
    assert(irq == IRQn_MCAN0);
    irq_enabled = false;
    if (s_report.frames_checked == DCU_LOOPBACK_FRAME_COUNT && !late_sent) {
        if (is("pending_rx_at_stop")) { late_sent = true; enqueue(1U); }
        if (is("pending_error_at_stop")) { late_sent = true; flags |= MCAN_EVENT_ERROR; }
    }
}
void intc_m_enable_irq(unsigned irq) { assert(irq == IRQn_MCAN0); irq_enabled = true; dispatch(); }
void intc_m_enable_irq_with_priority(unsigned irq, unsigned priority)
{
    assert(priority == 1U);
    intc_m_enable_irq(irq);
}
uint32_t mcan_get_interrupt_flags(MCAN_Type *base) { assert(base == &controller); return flags; }
void mcan_clear_interrupt_flags(MCAN_Type *base, uint32_t mask)
{
    assert(base == &controller);
    flags &= ~mask;
}
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *base, unsigned index)
{
    assert(base == &controller && index == 0U);
    return fifo_count - fifo_read;
}
hpm_stat_t mcan_read_rxfifo(MCAN_Type *base, unsigned index, mcan_rx_message_t *frame)
{
    assert(base == &controller && index == 0U && fifo_read < fifo_count);
    if (is("read_failure")) return -1;
    *frame = fifo[fifo_read++];
    return status_success;
}
hpm_stat_t mcan_set_msg_buf_attr(MCAN_Type *base, const mcan_msg_buf_attr_t *attr)
{
    assert(base == &controller && attr->ram_size == 2560U);
    return is("ram_failure") ? -1 : status_success;
}
void mcan_get_default_config(MCAN_Type *base, mcan_config_t *config)
{
    assert(base == &controller);
    memset(config, 0, sizeof(*config));
}
void mcan_get_default_ram_config(MCAN_Type *base, mcan_ram_config_t *config, bool fd)
{
    assert(base == &controller && !fd);
    memset(config, 0, sizeof(*config));
}
hpm_stat_t mcan_init(MCAN_Type *base, const mcan_config_t *config, uint32_t clock_hz)
{
    assert(base == &controller && clock_hz == 80000000U && !irq_enabled);
    assert(config->mode == mcan_mode_loopback_internal && !config->enable_canfd && !config->enable_tdc);
    assert(config->baudrate == 1000000U);
    assert(config->ram_config.rxfifos[0].enable && config->ram_config.rxfifos[0].elem_count == 4U);
    assert(config->ram_config.rxfifos[0].data_field_size == MCAN_DATA_FIELD_SIZE_8BYTES);
    assert(!config->ram_config.rxfifos[1].enable && !config->ram_config.enable_rxbuf);
    assert(!config->ram_config.enable_std_filter && !config->ram_config.enable_ext_filter);
    assert(!config->all_filters_config.std_id_filter_list.mcan_filter_elem_count);
    assert(!config->all_filters_config.ext_id_filter_list.mcan_filter_elem_count);
    assert(config->all_filters_config.global_filter_config.reject_remote_std_frame);
    assert(config->all_filters_config.global_filter_config.reject_remote_ext_frame);
    assert(config->all_filters_config.global_filter_config.accept_non_matching_std_frame_option == 0U);
    assert(config->all_filters_config.global_filter_config.accept_non_matching_ext_frame_option == 2U);
    assert(config->ram_config.txbuf_fifo_or_queue_elem_count == 4U);
    assert(!config->ram_config.txbuf_dedicated_txbuf_elem_count && !config->ram_config.txfifo_or_txqueue_mode);
    assert(!config->ram_config.enable_tx_evt_fifo && config->txbuf_trans_interrupt_mask == 0xfU);
    assert(config->interrupt_mask == DCU_LOOPBACK_IRQ_MASK);
    ++init_count;
    controller.CCCR = MCAN_CCCR_TEST_MASK | MCAN_CCCR_MON_MASK;
    controller.TEST = MCAN_TEST_LBCK_MASK;
    if (is("wrong_mode")) controller.CCCR &= ~MCAN_CCCR_MON_MASK;
    if (is("wrong_test_bit")) controller.TEST = 0U;
    peripheral_irq = true;
    return is("init_failure") ? -1 : status_success;
}
void mcan_interrupt_line_select(MCAN_Type *base, uint32_t mask, unsigned line)
{
    assert(base == &controller && mask == DCU_LOOPBACK_IRQ_MASK && line == 0U);
}
hpm_stat_t mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *slot)
{
    assert(base == &controller && !irq_enabled && internal_mode());
    assert(frame->std_id == 0x600U && frame->dlc == 8U);
    assert(!frame->use_ext_id && !frame->rtr && !frame->canfd_frame && !frame->bitrate_switch);
    staged = *frame;
    *slot = is("bad_slot") ? 4U : next_slot++ % 4U;
    controller.TXBTO &= ~(1U << *slot);
    return is("tx_failure") ? -1 : status_success;
}
void mcan_send_add_request(MCAN_Type *base, uint32_t slot)
{
    assert(base == &controller && !irq_enabled && s_irq.pending_mask == (1U << slot));
    assert(s_report.result == DCU_LOOPBACK_RUNNING);
    ++send_count;
    if (!is("no_tx")) {
        controller.TXBTO |= 1U << slot;
        flags |= MCAN_EVENT_TRANSMIT;
    }
    if (!is("no_rx") && !is("frozen_clock")) enqueue(is("duplicate") ? 2U : is("overflow") ? 4U : 1U);
    if (is("rx_then_tx")) {
        flags &= ~MCAN_EVENT_TRANSMIT;
        deferred_tx = true;
        deferred_at_us = now_us + 8U;
    }
    if (is("tx_then_rx")) {
        flags &= ~MCAN_INT_RXFIFO0_NEW_MSG;
        fifo_read = fifo_count = 0U;
        deferred_rx = true;
        deferred_at_us = now_us + 8U;
    }
    if (is("irq_error")) flags |= MCAN_EVENT_ERROR;
    if (is("fifo_lost")) flags |= MCAN_INT_RXFIFO0_MSG_LOST;
    if (is("ram_access_failure")) flags |= MCAN_INT_MSG_RAM_ACCESS_FAILURE;
    if (is("late")) now_us += DCU_LOOPBACK_TIMEOUT_MS * 1000U;
    if (is("mode_lost")) controller.CCCR &= ~MCAN_CCCR_MON_MASK;
}
void mcan_disable_interrupts(MCAN_Type *base, uint32_t mask)
{
    assert(base == &controller && mask == UINT32_MAX);
    peripheral_irq = false;
}
void mcan_cancel_tx_buf_send_request(MCAN_Type *base, unsigned slot)
{
    assert(base == &controller && slot < 4U && !irq_enabled);
    ++cancel_count;
}
void mcan_enter_init_mode(MCAN_Type *base)
{
    assert(base == &controller && !irq_enabled && !peripheral_irq);
    if (!is("stop_failure")) controller.CCCR |= MCAN_CCCR_INIT_MASK;
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    scenario = argv[1];
    if (is("payloads")) {
        for (unsigned i = 0; i < 3U; ++i) {
            for (unsigned j = 0; j < 8U; ++j) printf("%02x", s_payloads[i][j]);
            printf("\n");
        }
        return 0;
    }
    if (is("wrap")) now_us = UINT32_MAX - 50U;
    dcu_can_loopback_run();
    const volatile dcu_can_loopback_report_t *report = dcu_can_loopback_report();
    dcu_can_loopback_result_t expected = DCU_LOOPBACK_PASS;
    if (is("timebase_failure")) expected = DCU_LOOPBACK_TIMEBASE_FAILED;
    if (is("clock_failure") || is("ram_failure") || is("init_failure")) expected = DCU_LOOPBACK_SETUP_FAILED;
    if (is("wrong_mode") || is("wrong_test_bit") || is("mode_lost")) expected = DCU_LOOPBACK_MODE_FAILED;
    if (is("tx_failure") || is("bad_slot")) expected = DCU_LOOPBACK_TX_FAILED;
    if (is("bad_data") || is("bad_id") || is("bad_dlc") || is("short_dlc") ||
        is("fd") || is("brs") || is("rtr") || is("extended")) expected = DCU_LOOPBACK_RX_MISMATCH;
    if (is("no_tx") || is("no_rx") || is("no_irq") || is("frozen_clock") ||
        is("quiet_clock_failure") || is("late")) expected = DCU_LOOPBACK_TIMEOUT;
    if (is("irq_error") || is("fifo_lost") || is("read_failure") ||
        is("ram_access_failure") || is("pending_error_at_stop")) expected = DCU_LOOPBACK_IRQ_FAILED;
    if (is("overflow")) expected = DCU_LOOPBACK_QUEUE_OVERFLOW;
    if (is("duplicate") || is("late_rx") || is("pending_rx_at_stop")) expected = DCU_LOOPBACK_EXTRA_RX;
    if (is("stop_failure")) expected = DCU_LOOPBACK_STOP_FAILED;
    if (report->result != expected) fprintf(stderr, "result=%u expected=%u\n", report->result, expected);
    assert(report->result == expected && !irq_enabled);
    if (is("timebase_failure") || is("clock_failure")) {
        assert(send_count == 0U && init_count == 0U && cancel_count == 0U);
        assert(!report->controller_stopped);
        if (is("timebase_failure")) assert(clock_count == 0U);
    } else {
        assert(cancel_count == 4U && !peripheral_irq);
        assert(report->controller_stopped == !is("stop_failure"));
    }
    if (expected == DCU_LOOPBACK_PASS) {
        assert(send_count == DCU_LOOPBACK_FRAME_COUNT && report->frames_checked == send_count);
        assert(report->tx_completed == send_count && report->rx_received == send_count);
        assert(report->irq_count == send_count * ((is("rx_then_tx") || is("tx_then_rx")) ? 2U : 1U));
        assert(report->queue_drops == 0U && report->error_flags == 0U);
        for (unsigned i = 0; i < send_count; ++i) {
            assert(report->frames[i].matched && report->frames[i].received_dlc == 8U);
            assert(report->frames[i].received_id == 0x600U && report->frames[i].received_flags == 0U);
            assert(report->frames[i].elapsed_us < DCU_LOOPBACK_TIMEOUT_MS * 1000U);
            for (unsigned j = 0; j < 8U; ++j) assert(report->frames[i].received_data[j] == s_payloads[i % 3U][j]);
        }
    }
    if (expected == DCU_LOOPBACK_SETUP_FAILED || is("wrong_mode") || is("wrong_test_bit") ||
        expected == DCU_LOOPBACK_TX_FAILED) assert(send_count == 0U);
    if (is("mode_lost")) assert(send_count == 1U);
    uint32_t previous_sends = send_count;
    dcu_can_loopback_run();
    assert(send_count == previous_sends && report->result == expected);
    return 0;
}
