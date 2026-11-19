#include <stdio.h>
#include <stdlib.h>
#include "hpm_mcan_transport.c"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

MCAN_Type test_mcan[4];
GPIO_Type test_gpio;
static bool inhibited, during_stage, during_commit, deferred_fault, stage_failure;
static unsigned critical_depth, stage_calls, commits;
static uint8_t expected_dlc = 8;
static mcan_tx_frame_t staged;
static mcan_rx_message_t pending_rx;
static unsigned rx_fill, configured, fail_at, enabled_irqs, enabled_phys;
static uint32_t irq_flags = MCAN_INT_RXFIFO0_NEW_MSG;
static const dcu_can_channel_config_t fd_bus0 = {0, 1000000, 5000000, true};
static const dcu_can_channel_config_t *expected = &fd_bus0;

static void fault_timer(void)
{
    if (critical_depth) {
        deferred_fault = true;
        return;
    }
    inhibited = true;
    dcu_can_cancel_from_isr(0);
}

bool dcu_tx_inhibited(void) { return inhibited; }
uint32_t dcu_time_now_us(void) { return 0; }
uint32_t dcu_time_now_ms(void) { return 42; }
void test_enter_critical(void) { ++critical_depth; }
void test_exit_critical(void)
{
    CHECK(critical_depth == 1);
    --critical_depth;
    if (deferred_fault) {
        deferred_fault = false;
        fault_timer();
    }
}
UBaseType_t test_set_interrupt_mask(void) { return critical_depth++; }
void test_clear_interrupt_mask(UBaseType_t saved) { CHECK(critical_depth == saved + 1); critical_depth = saved; }
void intc_m_disable_irq(unsigned irq) { CHECK(irq >= IRQn_MCAN0 && irq <= IRQn_MCAN3); }
void intc_m_enable_irq(unsigned irq) { CHECK(irq >= IRQn_MCAN0 && irq <= IRQn_MCAN3); }
void intc_m_enable_irq_with_priority(unsigned irq, unsigned priority)
{ CHECK(priority == DCU_IRQ_CAN_PRIORITY); enabled_irqs |= 1U << (irq - IRQn_MCAN0); }
void gpio_write_pin(GPIO_Type *base, unsigned index, unsigned pin, unsigned level)
{ (void)base; (void)index; (void)pin; (void)level; }
void board_init_can(MCAN_Type *base) { CHECK(base == &test_mcan[expected[configured].bus]); }
uint32_t board_init_can_clock(MCAN_Type *base)
{ CHECK(base == &test_mcan[expected[configured].bus]); return 80000000U; }
void board_can_transceiver_phy_set(MCAN_Type *base, bool enabled)
{ CHECK(enabled); enabled_phys |= 1U << (base - test_mcan); }
void mcan_get_default_config(MCAN_Type *base, mcan_config_t *config)
{ (void)base; memset(config, 0, sizeof(*config)); }
void mcan_get_default_ram_config(MCAN_Type *base, test_ram_config_t *config, bool fd)
{ (void)base; CHECK(fd == expected[configured].fd); memset(config, 0, sizeof(*config)); }
int mcan_init(MCAN_Type *base, mcan_config_t *config, uint32_t clock)
{
    const dcu_can_channel_config_t *channel = &expected[configured++];
    CHECK(base == &test_mcan[channel->bus] && clock == 80000000U);
    CHECK(config->enable_canfd == channel->fd && config->enable_tdc == channel->fd);
    CHECK(config->baudrate == channel->nominal_bitrate_bps && config->baudrate_fd == channel->data_bitrate_bps);
    CHECK(config->ram_config.rxfifos[0].enable &&
          config->ram_config.rxfifos[0].elem_count == CANFD_RX_FIFO_DEPTH &&
          config->ram_config.rxfifos[0].data_field_size == MCAN_DATA_FIELD_SIZE_64BYTES);
    CHECK(config->ram_config.txbuf_fifo_or_queue_elem_count == CANFD_TX_FIFO_DEPTH &&
          config->ram_config.txbuf_data_field_size == MCAN_DATA_FIELD_SIZE_64BYTES);
    return configured == fail_at ? 1 : status_success;
}
void mcan_interrupt_line_select(MCAN_Type *base, uint32_t mask, unsigned line)
{ (void)base; CHECK(mask & MCAN_INT_RXFIFO0_NEW_MSG); CHECK(line == 0); }
uint32_t mcan_get_interrupt_flags(MCAN_Type *base)
{ CHECK(base == HPM_MCAN0); return irq_flags; }
void mcan_clear_interrupt_flags(MCAN_Type *base, uint32_t flags)
{ CHECK(base == HPM_MCAN0 && flags == irq_flags); }
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *base, unsigned fifo)
{ CHECK(base == HPM_MCAN0 && fifo == 0); return rx_fill; }
int mcan_read_rxfifo(MCAN_Type *base, unsigned fifo, mcan_rx_message_t *frame)
{
    CHECK(base == HPM_MCAN0 && fifo == 0 && rx_fill == 1);
    *frame = pending_rx;
    --rx_fill;
    return status_success;
}
int mcan_get_protocol_status(MCAN_Type *base, mcan_protocol_status_t *status)
{ CHECK(dcu_can_channel_config((unsigned)(base - test_mcan))); memset(status, 0, sizeof(*status)); return 0; }
int mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index)
{
    (void)base;
    CHECK(critical_depth == 0 && frame->dlc == expected_dlc);
    staged = *frame;
    ++stage_calls;
    *index = 2;
    if (during_stage)
        fault_timer();
    return stage_failure ? 1 : 0;
}
void mcan_send_add_request(MCAN_Type *base, uint32_t index)
{
    if (during_commit)
        fault_timer();
    CHECK(!inhibited && critical_depth == 1);
    CHECK(s_tx[base - test_mcan][index].id);
    ++commits;
    base->TXBTO &= ~(1U << index);
    base->TXBCF &= ~(1U << index);
    base->TXBRP |= 1U << index;
}

static void finish_hardware(unsigned bus, bool sent, bool cancelled)
{
    const uint32_t slot = 1U << 2;
    test_mcan[bus].TXBRP &= ~slot;
    if (sent) test_mcan[bus].TXBTO |= slot;
    if (cancelled) test_mcan[bus].TXBCF |= slot;
}

static void test_roundtrip(void)
{
    const uint8_t sizes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    for (unsigned i = 0; i < sizeof(sizes); ++i) {
        const bool fd = i > 8;
        const bool brs = i > 9;
        dcu_can_frame_t frame = {.id = 0x123, .dlc = i,
            .flags = (fd ? DCU_CAN_FLAG_FD : 0) | (brs ? DCU_CAN_FLAG_BRS : 0)};
        for (unsigned b = 0; b < sizeof(frame.data); ++b) frame.data[b] = 0x80U + b + i;
        expected_dlc = i;
        const dcu_can_tx_id_t id = dcu_can_submit(0, &frame);
        CHECK(id != 0 && dcu_can_pending(0));
        CHECK(staged.std_id == frame.id && !staged.use_ext_id && staged.canfd_frame == fd);
        CHECK(staged.bitrate_switch == brs);
        CHECK(!memcmp(staged.data_8, frame.data, sizes[i]));
        for (unsigned b = sizes[i]; b < sizeof(staged.data_8); ++b) CHECK(staged.data_8[b] == 0);
        pending_rx = staged;
        memset(pending_rx.data_8 + sizes[i], 0x99, sizeof(pending_rx.data_8) - sizes[i]);
        rx_fill = 1;
        mcan0_isr();
        uint32_t stamp = 0;
        dcu_can_frame_t received;
        CHECK(dcu_can_receive(0, &received, &stamp));
        CHECK(stamp == 42 && received.id == frame.id && received.dlc == frame.dlc && received.flags == frame.flags);
        CHECK(!memcmp(received.data, frame.data, sizes[i]));
        for (unsigned b = sizes[i]; b < sizeof(received.data); ++b) CHECK(received.data[b] == 0);
        CHECK(!dcu_can_receive(0, &received, &stamp));
        finish_hardware(0, true, false);
        dcu_can_tx_event_t event;
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == id && event.status == DCU_CAN_TX_COMPLETED);
    }
    CHECK(commits == sizeof(sizes) && stage_calls == sizeof(sizes) && dcu_can_rx_drops(0) == 0);
}

static void test_invalid_lengths(void)
{
    for (unsigned dlc = 9; dlc <= 16; ++dlc) {
        for (unsigned fd = 0; fd <= 1; ++fd) {
            if (fd && dlc <= 15) continue;
            dcu_can_frame_t frame = {.id = 1, .dlc = dlc, .flags = fd ? DCU_CAN_FLAG_FD : 0};
            CHECK(!dcu_can_submit(0, &frame));
            pending_rx = (mcan_rx_message_t){.std_id = 1, .dlc = dlc, .canfd_frame = fd};
            rx_fill = 1;
            mcan0_isr();
            uint32_t stamp;
            CHECK(!dcu_can_receive(0, &frame, &stamp));
        }
    }
    CHECK(commits == 0 && stage_calls == 0 && dcu_can_rx_drops(0) == 9);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    dcu_can_frame_t frame = {.id = 1, .dlc = 8};
    if (!strcmp(argv[1], "single_profile")) {
        const dcu_can_channel_config_t channel = {2, 500000, 0, false};
        expected = &channel;
        CHECK(dcu_mcan_init(&channel, 1) == status_success);
        CHECK(configured == 1 && enabled_irqs == 4 && enabled_phys == 4);
        CHECK(dcu_can_channel_config(2)->nominal_bitrate_bps == 500000);
        CHECK(dcu_can_bus_state(2) == DCU_CAN_BUS_ERROR_ACTIVE);
        for (unsigned c = 0; c <= 4; ++c) {
            if (c == 2) continue;
            uint32_t stamp;
            CHECK(!dcu_can_channel_config(c) && dcu_can_bus_state(c) == DCU_CAN_BUS_DISABLED);
            CHECK(!dcu_can_submit(c, &frame) && !dcu_can_receive(c, &frame, &stamp));
            CHECK(!dcu_can_pending(c) && !dcu_can_tx_free(c) && !dcu_can_tx_progress(c) && !dcu_can_rx_drops(c));
            dcu_can_cancel(c);
            CHECK(!dcu_can_cancel_tx(c, 1));
        }
        frame.flags = DCU_CAN_FLAG_FD;
        CHECK(!dcu_can_submit(2, &frame));
        CHECK(!stage_calls && !commits);
        return 0;
    }
    if (!strcmp(argv[1], "invalid_profile")) {
        dcu_can_channel_config_t channels[] = {fd_bus0, fd_bus0};
        CHECK(dcu_mcan_init(NULL, 1) == status_invalid_argument);
        CHECK(dcu_mcan_init(channels, 0) == status_invalid_argument);
        CHECK(dcu_mcan_init(channels, 2) == status_invalid_argument);
        channels[1].bus = 4;
        CHECK(dcu_mcan_init(channels, 2) == status_invalid_argument);
        channels[1].bus = 2;
        channels[1].data_bitrate_bps = 0;
        CHECK(dcu_mcan_init(channels, 2) == status_invalid_argument);
        CHECK(!configured && !enabled_irqs && !enabled_phys);
        return 0;
    }
    if (!strcmp(argv[1], "partial_profile")) {
        const dcu_can_channel_config_t channels[] = {{0, 1000000, 5000000, true}, {2, 1000000, 0, false}};
        expected = channels;
        fail_at = 2;
        CHECK(dcu_mcan_init(channels, 2) != status_success);
        CHECK(configured == 2 && !enabled_irqs && !enabled_phys);
        CHECK(!dcu_can_channel_config(0) && !dcu_can_channel_config(2));
        CHECK(!dcu_can_submit(0, &frame) && !stage_calls);
        return 0;
    }
    if (!strcmp(argv[1], "transaction_ids")) {
        const dcu_can_channel_config_t channels[] = {fd_bus0, {2, 1000000, 5000000, true}};
        expected = channels;
        CHECK(dcu_mcan_init(channels, 2) == status_success);
        const dcu_can_tx_id_t id0 = dcu_can_submit(0, &frame);
        const dcu_can_tx_id_t id2 = dcu_can_submit(2, &frame);
        CHECK(id0 && id0 == id2);
        dcu_can_tx_event_t event;
        CHECK(!dcu_can_take_tx_event(0, &event) && !dcu_can_take_tx_event(2, &event));
        CHECK(dcu_can_cancel_tx(0, id0));
        CHECK(test_mcan[0].TXBCR == 4 && test_mcan[2].TXBCR == 0);
        CHECK(!dcu_can_take_tx_event(0, &event));
        finish_hardware(2, true, false);
        CHECK(dcu_can_take_tx_event(2, &event) && event.id == id2 && event.status == DCU_CAN_TX_COMPLETED);
        CHECK(!dcu_can_take_tx_event(0, &event) && dcu_can_tx_progress(0) == 0);
        finish_hardware(0, true, true); /* Both bits can be raised in a cancel-vs-send race. */
        CHECK(!dcu_can_submit(0, &frame)); /* The old result still owns the slot. */
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == id0 && event.status == DCU_CAN_TX_CANCELLED);
        CHECK(!dcu_can_take_tx_event(0, &event) && dcu_can_tx_progress(0) == 0);
        const dcu_can_tx_id_t next = dcu_can_submit(0, &frame);
        CHECK(next > id0 && !dcu_can_cancel_tx(0, id0));
        irq_flags = MCAN_EVENT_TRANSMIT;
        mcan0_isr(); /* A late IRQ carries no identity and must not settle the new request. */
        CHECK(!dcu_can_take_tx_event(0, &event));
        finish_hardware(0, true, false);
        CHECK(!dcu_can_cancel_tx(0, next));
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == next && event.status == DCU_CAN_TX_COMPLETED);
        CHECK(!dcu_can_take_tx_event(0, &event) && dcu_can_tx_progress(0) == 1 && dcu_can_tx_progress(2) == 1);
        return 0;
    }
    CHECK(dcu_mcan_init(&fd_bus0, 1) == status_success && configured == 1);
    if (!strcmp(argv[1], "staging_fault")) {
        during_stage = true;
        CHECK(!dcu_can_submit(0, &frame));
        CHECK(stage_calls == 1 && commits == 0 && !dcu_can_pending(0));
    } else if (!strcmp(argv[1], "commit_fault")) {
        during_commit = true;
        CHECK(dcu_can_submit(0, &frame));
        CHECK(commits == 1 && inhibited && test_mcan[0].TXBCR == 4);
        finish_hardware(0, false, true);
        dcu_can_tx_event_t event;
        CHECK(dcu_can_take_tx_event(0, &event) && event.status == DCU_CAN_TX_CANCELLED);
    } else if (!strcmp(argv[1], "already_inhibited")) {
        inhibited = true;
        CHECK(!dcu_can_submit(0, &frame));
        CHECK(stage_calls == 0 && commits == 0);
    } else if (!strcmp(argv[1], "stage_failure")) {
        stage_failure = true;
        CHECK(!dcu_can_submit(0, &frame));
        CHECK(commits == 0 && !dcu_can_pending(0));
    } else if (!strcmp(argv[1], "invalid")) {
        CHECK(!dcu_can_submit(4, &frame));
        CHECK(!dcu_can_submit(0, NULL));
        CHECK(stage_calls == 0 && commits == 0);
    } else if (!strcmp(argv[1], "fd_roundtrip")) {
        test_roundtrip();
    } else if (!strcmp(argv[1], "invalid_lengths")) {
        test_invalid_lengths();
    } else {
        CHECK(!strcmp(argv[1], "normal"));
        CHECK(dcu_can_submit(0, &frame));
        CHECK(commits == 1 && stage_calls == 1 && test_mcan[0].TXBRP == 4);
    }
    CHECK(critical_depth == 0);
    return 0;
}
