#include <stdio.h>
#include <stdlib.h>
#include "hpm_mcan_transport.c"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); \
} } while (0)

MCAN_Type test_mcan[4];
GPIO_Type test_gpio;
static bool inhibited, during_stage, during_commit, deferred_fault, stage_failure;
static unsigned fifo_index = 2;
static unsigned critical_depth, stage_calls, commits;
static uint8_t expected_dlc = 8;
static mcan_tx_frame_t staged;
static mcan_rx_message_t pending_rx;
static mcan_tx_event_fifo_elem_t pending_event;
static bool event_ready, force_sof, force_dar;
static uint32_t tsu_ticks[4][16];
static unsigned rx_fill, configured, fail_at, enabled_irqs, enabled_phys;
static uint32_t irq_flags = MCAN_INT_RXFIFO0_NEW_MSG;
static uint32_t observed_us;
static unsigned ir_reads_to_inject;
static uint32_t injected_psr;
static bool clear_first_error_evidence_on_ack;
static unsigned expected_can3_tdco;
static const dcu_can_channel_config_t fd_bus0 = {0, 1000000, 5000000, true, false};
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
uint32_t dcu_time_now_us(void) { return observed_us; }
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
    const unsigned tdco = channel->bus == 3U ? expected_can3_tdco : 0U;
    CHECK(config->tdc_config.ssp_offset == tdco);
    CHECK(config->tdc_config.filter_window_length == (tdco ? 12U : 0U));
    CHECK(!config->disable_auto_retransmission);
    CHECK(config->use_timestamping_unit == channel->timestamp_eof);
    CHECK(config->ram_config.enable_tx_evt_fifo == channel->timestamp_eof);
    base->CCCR = force_dar ? MCAN_CCCR_DAR_MASK : 0U;
    if (channel->timestamp_eof) {
        CHECK(config->timestamp_cfg.timestamp_selection == MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED);
        CHECK(config->tsu_config.enable_tsu && !config->tsu_config.capture_on_sof &&
              !config->tsu_config.use_ext_timebase && !config->tsu_config.enable_64bit_timestamp);
        CHECK(config->ram_config.tx_evt_fifo_elem_count == CANFD_TX_FIFO_DEPTH);
        CHECK(config->ram_config.enable_std_filter && config->ram_config.std_filter_elem_count == 1U);
        CHECK(config->all_filters_config.std_id_filter_list.mcan_filter_elem_count == 1U);
        const mcan_filter_elem_t *filter = config->all_filters_config.std_id_filter_list.filter_elem_list;
        CHECK(filter && filter->sync_message == 1U);
        CHECK(filter->filter_type == MCAN_FILTER_TYPE_CLASSIC_FILTER &&
              filter->filter_config == MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH &&
              filter->can_id_type == MCAN_CAN_ID_TYPE_STANDARD &&
              filter->filter_id == 0U && filter->filter_mask == 0U);
        base->CCCR |= MCAN_CCCR_UTSU_MASK;
        base->TSCFG = MCAN_TSCFG_TSUE_MASK | (force_sof ? MCAN_TSCFG_SCP_MASK : 0);
    } else {
        CHECK(!config->ram_config.enable_std_filter &&
              config->all_filters_config.std_id_filter_list.mcan_filter_elem_count == 0U);
    }
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
{
    CHECK(dcu_can_channel_config((unsigned)(base - test_mcan)));
    if (ir_reads_to_inject && --ir_reads_to_inject == 0) {
        base->IR = MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE;
        base->PSR = injected_psr;
    }
    return irq_flags | base->IR;
}
void mcan_clear_interrupt_flags(MCAN_Type *base, uint32_t flags)
{
    CHECK(dcu_can_channel_config((unsigned)(base - test_mcan)) && flags == (irq_flags | base->IR));
    base->IR &= ~flags;
    if (clear_first_error_evidence_on_ack) {
        base->PSR = 0x707U;
        base->ECR = 0;
    }
}
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *base, unsigned fifo)
{ CHECK((base == HPM_MCAN0 || base == HPM_MCAN1) && fifo == 0); return rx_fill; }
int mcan_read_rxfifo(MCAN_Type *base, unsigned fifo, mcan_rx_message_t *frame)
{
    CHECK((base == HPM_MCAN0 || base == HPM_MCAN1) && fifo == 0 && rx_fill == 1);
    *frame = pending_rx;
    --rx_fill;
    return status_success;
}
int mcan_read_tx_evt_fifo(MCAN_Type *base, mcan_tx_event_fifo_elem_t *event)
{
    CHECK(base == HPM_MCAN1 && critical_depth == 1);
    if (!event_ready) return status_fail;
    *event = pending_event;
    event_ready = false;
    return status_success;
}
bool mcan_is_tsu_used(MCAN_Type *base) { return (base->CCCR & MCAN_CCCR_UTSU_MASK) != 0; }
bool mcan_is_tsu_timestamp_available(MCAN_Type *base, unsigned slot)
{ return slot < 16 && (base->TSS1 & (1U << slot)) != 0; }
uint32_t mcan_read_32bit_tsu_timestamp(MCAN_Type *base, uint32_t slot)
{
    CHECK(mcan_is_tsu_timestamp_available(base, slot));
    base->TSS1 &= ~(1U << slot);
    return tsu_ticks[base - test_mcan][slot];
}
int mcan_request_and_fill_txfifo(MCAN_Type *base, mcan_tx_frame_t *frame, uint32_t *index)
{
    (void)base;
    CHECK(critical_depth == 0 && frame->dlc == expected_dlc);
    staged = *frame;
    ++stage_calls;
    *index = fifo_index;
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

static void test_first_fault(const char *scenario)
{
    const dcu_can_channel_config_t channels[] = {fd_bus0, {1, 1000000, 5000000, true, true}};
    expected = channels;
    CHECK(dcu_mcan_init(channels, 2) == status_success);
    const dcu_can_frame_t frame = {.id = 2, .dlc = 8, .flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS};
    const dcu_can_tx_id_t id = dcu_can_submit(1, 0, &frame);
    CHECK(id && !s_first_fault[1].valid);
    MCAN_Type *base = HPM_MCAN1;
    observed_us = 12345678;
    base->IR = MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE;
    base->PSR = 0x001c340fU;
    base->ECR = 0x00010008U;
    base->DBTP = 0x00800a33U;
    base->NBTP = 0x26003a13U;
    base->TDCR = 0x00000c0cU;
    base->TXEFS = 0x123U;
    base->RXF0S = 0x456U;
    canfd_fault_source_t source = CANFD_FAULT_FROM_STATE_READ;
    bool terminal = true, error_flag = true;
    if (!strcmp(scenario, "first_fault_irq")) {
        source = CANFD_FAULT_FROM_IRQ;
        finish_hardware(1, false, true);
        clear_first_error_evidence_on_ack = true;
        mcan1_isr();
        CHECK(base->ECR == 0 && !base->IR);
    } else if (!strcmp(scenario, "first_fault_poll")) {
        finish_hardware(1, false, true);
        CHECK(dcu_can_bus_state(1) == DCU_CAN_BUS_ERROR_ACTIVE);
    } else if (!strcmp(scenario, "first_fault_poll_edge")) {
        finish_hardware(1, false, true);
        base->IR = 0;
        base->PSR = 0x707U;
        injected_psr = 0x001c340fU;
        ir_reads_to_inject = 2;
        CHECK(dcu_can_bus_state(1) == DCU_CAN_BUS_ERROR_ACTIVE && !ir_reads_to_inject);
    } else if (!strcmp(scenario, "first_fault_poll_read_clear")) {
        finish_hardware(1, false, true);
        base->IR = 0;
        injected_psr = 0x001c370fU; // The first PSR read consumed DLEC before PED was seen.
        ir_reads_to_inject = 2;
        CHECK(dcu_can_bus_state(1) == DCU_CAN_BUS_ERROR_ACTIVE && !ir_reads_to_inject);
    } else if (!strcmp(scenario, "first_fault_psr_only")) {
        finish_hardware(1, false, true);
        base->IR = 0;
        error_flag = false;
        CHECK(dcu_can_bus_state(1) == DCU_CAN_BUS_ERROR_ACTIVE);
    } else if (!strcmp(scenario, "first_fault_tx_event")) {
        source = CANFD_FAULT_BEFORE_TX_EVENT;
        finish_hardware(1, false, true);
        dcu_can_tx_event_t event;
        CHECK(dcu_can_take_tx_event(1, &event) && event.id == id && event.status == DCU_CAN_TX_CANCELLED);
        CHECK(!dcu_can_pending(1, 0));
    } else if (!strcmp(scenario, "first_fault_cancel")) {
        source = CANFD_FAULT_BEFORE_CANCEL;
        terminal = false;
        dcu_can_cancel_from_isr(1);
        CHECK(base->TXBCR == 4);
    } else {
        CHECK(false);
    }

    const canfd_first_fault_t first = s_first_fault[1];
    CHECK(first.valid == 1 && first.source == source && first.observed_us == 12345678);
    CHECK(first.psr == 0x001c340fU && first.ecr == 0x00010008U);
    CHECK((first.ir & s_protocol_errors) == (error_flag ? MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE : 0));
    CHECK(first.txbrp == (terminal ? 0U : 4U) && !first.txbto && first.txbcf == (terminal ? 4U : 0U));
    CHECK(first.nbtp == base->NBTP && first.dbtp == base->DBTP && first.tdcr == base->TDCR &&
          first.cccr == base->CCCR && first.txefs == base->TXEFS && first.rxf0s == base->RXF0S);
    for (unsigned i = 0; i < CANFD_TX_FIFO_DEPTH; ++i)
        CHECK(first.tx_id[i] == (i == 2 ? id : 0U));
    CHECK(!s_first_fault[0].valid);
    observed_us += 50;
    base->IR = MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE;
    base->PSR = MCAN_PSR_EW_MASK | MCAN_PSR_EP_MASK | 5U;
    CHECK(dcu_can_bus_state(1) == DCU_CAN_BUS_ERROR_PASSIVE);
    mcan1_isr();
    CHECK(s_first_fault[1].source == first.source && s_first_fault[1].psr == first.psr &&
          s_first_fault[1].observed_us == first.observed_us);
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
        const dcu_can_tx_id_t id = dcu_can_submit(0, 0, &frame);
        CHECK(id != 0 && dcu_can_pending(0, 0));
        CHECK(staged.std_id == frame.id && !staged.use_ext_id && staged.canfd_frame == fd);
        CHECK(staged.bitrate_switch == brs);
        CHECK(!memcmp(staged.data_8, frame.data, sizes[i]));
        for (unsigned b = sizes[i]; b < sizeof(staged.data_8); ++b) CHECK(staged.data_8[b] == 0);
        pending_rx = staged;
        memset(pending_rx.data_8 + sizes[i], 0x99, sizeof(pending_rx.data_8) - sizes[i]);
        rx_fill = 1;
        mcan0_isr();
        dcu_can_rx_stamp_t stamp = {0};
        dcu_can_frame_t received;
        CHECK(dcu_can_receive(0, &received, &stamp));
        CHECK(stamp.observed_ms == 42 && !stamp.eof.valid && received.id == frame.id &&
              received.dlc == frame.dlc && received.flags == frame.flags);
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
            CHECK(!dcu_can_submit(0, 0, &frame));
            pending_rx = (mcan_rx_message_t){.std_id = 1, .dlc = dlc, .canfd_frame = fd};
            rx_fill = 1;
            mcan0_isr();
            dcu_can_rx_stamp_t stamp;
            CHECK(!dcu_can_receive(0, &frame, &stamp));
        }
    }
    CHECK(commits == 0 && stage_calls == 0 && dcu_can_rx_drops(0) == 9);
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    if (!strcmp(argv[1], "tdc_baseline") || !strcmp(argv[1], "tdc_comparison")) {
        const dcu_can_channel_config_t channels[] = {
            {1, 1000000, 5000000, true, true},
            {3, 1000000, 5000000, true, true},
        };
        expected = channels;
        expected_can3_tdco = !strcmp(argv[1], "tdc_comparison") ? 13U : 0U;
        CHECK(dcu_mcan_init(channels, 2) == status_success);
        CHECK(configured == 2 && enabled_irqs == 0xAU && enabled_phys == 0xAU);
        return 0;
    }
    dcu_can_frame_t frame = {.id = 1, .dlc = 8};
    if (!strncmp(argv[1], "first_fault_", sizeof("first_fault_") - 1)) {
        test_first_fault(argv[1]);
        return 0;
    }
    if (!strcmp(argv[1], "single_profile")) {
        const dcu_can_channel_config_t channel = {2, 500000, 0, false, false};
        expected = &channel;
        CHECK(dcu_mcan_init(&channel, 1) == status_success);
        CHECK(configured == 1 && enabled_irqs == 4 && enabled_phys == 4);
        CHECK(dcu_can_channel_config(2)->nominal_bitrate_bps == 500000);
        CHECK(dcu_can_bus_state(2) == DCU_CAN_BUS_ERROR_ACTIVE);
        for (unsigned c = 0; c <= 4; ++c) {
            if (c == 2) continue;
            dcu_can_rx_stamp_t stamp;
            CHECK(!dcu_can_channel_config(c) && dcu_can_bus_state(c) == DCU_CAN_BUS_DISABLED);
            CHECK(!dcu_can_submit(c, 0, &frame) && !dcu_can_receive(c, &frame, &stamp));
            CHECK(!dcu_can_pending(c, 0) && !dcu_can_tx_free(c) && !dcu_can_tx_progress(c) && !dcu_can_rx_drops(c));
            dcu_can_cancel(c);
            CHECK(!dcu_can_cancel_tx(c, 1));
        }
        frame.flags = DCU_CAN_FLAG_FD;
        CHECK(!dcu_can_submit(2, 0, &frame));
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
    if (!strcmp(argv[1], "retry_completed") || !strcmp(argv[1], "retry_cancelled")) {
        const dcu_can_channel_config_t channel = {1, 1000000, 5000000, true, true};
        const bool cancel = !strcmp(argv[1], "retry_cancelled");
        expected = &channel;
        CHECK(dcu_mcan_init(&channel, 1) == status_success);
        frame.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        const dcu_can_tx_id_t id = dcu_can_submit(1, 0, &frame);
        CHECK(id && commits == 1);
        MCAN_Type *base = HPM_MCAN1;
        base->IR = MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE;
        base->PSR = 0x001c340fU;
        base->ECR = 0x00010008U;
        mcan1_isr();
        CHECK(s_first_fault[1].valid && s_first_fault[1].txbrp == (1U << 2));
#if defined(CONFIG_PDO_PHASE_TRACE)
        CHECK(dcu_can_observed_retx_count() == 1U);
        base->PSR = 0x707U; // The real PSR resets LEC/DLEC on read.
#endif
        dcu_can_tx_event_t event;
        CHECK(dcu_can_pending(1, 0) && !dcu_can_take_tx_event(1, &event));
        if (cancel) {
            CHECK(dcu_can_cancel_tx(1, id) && base->TXBCR == (1U << 2));
            CHECK(dcu_can_pending(1, 0) && !dcu_can_take_tx_event(1, &event));
        }
        pending_event = (mcan_tx_event_fifo_elem_t){cancel ? 2U : 1U, staged.message_marker_l, 0, true};
        event_ready = true;
        base->TSS1 = 1U;
        tsu_ticks[1][0] = 100U;
        finish_hardware(1, true, cancel); // A retry already on the wire can finish while being cancelled.
        CHECK(dcu_can_take_tx_event(1, &event) && event.id == id);
        CHECK(event.status == (cancel ? DCU_CAN_TX_CANCELLED : DCU_CAN_TX_COMPLETED));
        CHECK(cancel ? !event.eof.valid : (event.eof.valid && event.eof.tick == 100U));
        CHECK(!dcu_can_pending(1, 0) && dcu_can_tx_progress(1) == (cancel ? 0U : 1U) && commits == 1);
        return 0;
    }
    if (!strcmp(argv[1], "frame_eof") || !strcmp(argv[1], "bad_eof_mode") || !strcmp(argv[1], "bad_dar_mode")) {
        const dcu_can_channel_config_t channel = {1, 1000000, 5000000, true, true};
        expected = &channel;
        force_sof = !strcmp(argv[1], "bad_eof_mode");
        force_dar = !strcmp(argv[1], "bad_dar_mode");
        if (force_sof || force_dar) {
            CHECK(dcu_mcan_init(&channel, 1) == status_fail && !enabled_irqs && !enabled_phys);
            return 0;
        }
        CHECK(dcu_mcan_init(&channel, 1) == status_success);
        frame.flags = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
        dcu_can_tx_id_t id = dcu_can_submit(1, 0, &frame);
        CHECK(id && staged.timestamp_capture_enable && staged.event_fifo_control);
        CHECK(staged.message_marker_l == (uint8_t)id);
        pending_event = (mcan_tx_event_fifo_elem_t){1, staged.message_marker_l, 0, true};
        event_ready = true;
        HPM_MCAN1->TSS1 = 1U;
        HPM_MCAN1->ATB = 800;
        observed_us = 500;
        finish_hardware(1, true, false);
        dcu_can_tx_event_t event;
        CHECK(dcu_can_take_tx_event(1, &event) && event.id == id && event.status == DCU_CAN_TX_COMPLETED);
        CHECK(event.eof.valid && event.eof.tick == 0 && !HPM_MCAN1->TSS1);

#if defined(CONFIG_PDO_PHASE_TRACE)
        CHECK(event.eof.us == 490U); // 800 ticks at 80 MHz before observation.
#endif
        pending_rx = staged;
        pending_rx.rx_timestamp_captured = true;
        pending_rx.rx_timestamp_pointer = 1;
        HPM_MCAN1->TSS1 = 1U << 1;
        tsu_ticks[1][1] = UINT32_MAX - 79U;
        HPM_MCAN1->ATB = 80;
        rx_fill = 1;
        mcan1_isr();
        dcu_can_frame_t received;
        dcu_can_rx_stamp_t stamp;
        CHECK(dcu_can_receive(1, &received, &stamp));
        CHECK(stamp.observed_ms == 42 && stamp.eof.valid && stamp.eof.tick == UINT32_MAX - 79U && !HPM_MCAN1->TSS1);
#if defined(CONFIG_PDO_PHASE_TRACE)
        CHECK(stamp.eof.us == 498U); // Tick wrap before the RX ISR.
#endif
        rx_fill = 1; // Capture bit with no valid TSU slot is not evidence.
        mcan1_isr();
        CHECK(dcu_can_receive(1, &received, &stamp) && !stamp.eof.valid);

        id = dcu_can_submit(1, 0, &frame);
        CHECK(id && pending_event.message_marker != staged.message_marker_l);
        event_ready = true; // Marker from the stale FIFO entry must not tag the new TX.
        HPM_MCAN1->TSS1 = 1U;
        finish_hardware(1, true, false);
        CHECK(dcu_can_take_tx_event(1, &event) && event.id == id && !event.eof.valid && !HPM_MCAN1->TSS1);

        id = dcu_can_submit(1, 0, &frame);
        CHECK(id && dcu_can_cancel_tx(1, id));
        pending_event.message_marker = staged.message_marker_l;
        event_ready = true;
        HPM_MCAN1->TSS1 = 1U;
        finish_hardware(1, true, true);
        CHECK(dcu_can_take_tx_event(1, &event) && event.id == id &&
              event.status == DCU_CAN_TX_CANCELLED && !event.eof.valid && !HPM_MCAN1->TSS1);
        return 0;
    }
    if (!strcmp(argv[1], "partial_profile")) {
        const dcu_can_channel_config_t channels[] = {{0, 1000000, 5000000, true, false},
                                                    {2, 1000000, 0, false, false}};
        expected = channels;
        fail_at = 2;
        CHECK(dcu_mcan_init(channels, 2) != status_success);
        CHECK(configured == 2 && !enabled_irqs && !enabled_phys);
        CHECK(!dcu_can_channel_config(0) && !dcu_can_channel_config(2));
        CHECK(!dcu_can_submit(0, 0, &frame) && !stage_calls);
        return 0;
    }
    if (!strcmp(argv[1], "device_pending")) {
        CHECK(dcu_mcan_init(&fd_bus0, 1) == status_success);
        const dcu_can_tx_id_t first = dcu_can_submit(0, 0, &frame);
        CHECK(first && dcu_can_pending(0, 0) && !dcu_can_pending(0, 6));
        fifo_index = 3;
        const dcu_can_tx_id_t second = dcu_can_submit(0, 6, &frame);
        CHECK(second > first && dcu_can_pending(0, 0) && dcu_can_pending(0, 6));
        test_mcan[0].TXBRP &= ~(1U << fifo_index);
        test_mcan[0].TXBTO |= 1U << fifo_index;
        CHECK(dcu_can_pending(0, 6)); // Terminal but still owned.
        dcu_can_tx_event_t event;
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == second);
        CHECK(!dcu_can_pending(0, 6) && dcu_can_pending(0, 0));
        finish_hardware(0, true, false);
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == first);
        CHECK(!dcu_can_pending(0, DCU_CAN_DEVICE_ANY));
        CHECK(dcu_can_submit(0, DCU_CAN_DEVICE_ANY, &frame));
        CHECK(dcu_can_pending(0, 0) && dcu_can_pending(0, 6));
        // 透传撤销不能触碰电机或总线独占的参数事务。
        CHECK(!dcu_can_pending(0, DCU_CAN_DEVICE_RAW) && dcu_can_drain_raw(0));
        fifo_index = 1;
        CHECK(dcu_can_submit(0, 0, &frame));
        fifo_index = 4;
        const dcu_can_tx_id_t raw = dcu_can_submit(0, DCU_CAN_DEVICE_RAW, &frame);
        CHECK(raw && !dcu_can_drain_raw(0) && test_mcan[0].TXBCR == (1U << 4));
        test_mcan[0].TXBRP &= ~(1U << 4);
        test_mcan[0].TXBCF |= 1U << 4;
        CHECK(dcu_can_pending(0, DCU_CAN_DEVICE_RAW) && !dcu_can_drain_raw(0));
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == raw && event.status == DCU_CAN_TX_CANCELLED);
        CHECK(!dcu_can_pending(0, DCU_CAN_DEVICE_RAW) && dcu_can_drain_raw(0) && dcu_can_pending(0, 0));
        return 0;
    }
    if (!strcmp(argv[1], "transaction_ids")) {
        const dcu_can_channel_config_t channels[] = {fd_bus0, {2, 1000000, 5000000, true, false}};
        expected = channels;
        CHECK(dcu_mcan_init(channels, 2) == status_success);
        const dcu_can_tx_id_t id0 = dcu_can_submit(0, 0, &frame);
        const dcu_can_tx_id_t id2 = dcu_can_submit(2, 0, &frame);
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
        CHECK(!dcu_can_submit(0, 0, &frame)); /* The old result still owns the slot. */
        CHECK(dcu_can_take_tx_event(0, &event) && event.id == id0 && event.status == DCU_CAN_TX_CANCELLED);
        CHECK(!dcu_can_take_tx_event(0, &event) && dcu_can_tx_progress(0) == 0);
        const dcu_can_tx_id_t next = dcu_can_submit(0, 0, &frame);
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
        CHECK(!dcu_can_submit(0, 0, &frame));
        CHECK(stage_calls == 1 && commits == 0 && !dcu_can_pending(0, 0));
    } else if (!strcmp(argv[1], "commit_fault")) {
        during_commit = true;
        CHECK(dcu_can_submit(0, 0, &frame));
        CHECK(commits == 1 && inhibited && test_mcan[0].TXBCR == 4);
        finish_hardware(0, false, true);
        dcu_can_tx_event_t event;
        CHECK(dcu_can_take_tx_event(0, &event) && event.status == DCU_CAN_TX_CANCELLED);
    } else if (!strcmp(argv[1], "already_inhibited")) {
        inhibited = true;
        CHECK(!dcu_can_submit(0, 0, &frame));
        CHECK(stage_calls == 0 && commits == 0);
    } else if (!strcmp(argv[1], "stage_failure")) {
        stage_failure = true;
        CHECK(!dcu_can_submit(0, 0, &frame));
        CHECK(commits == 0 && !dcu_can_pending(0, 0));
    } else if (!strcmp(argv[1], "invalid")) {
        CHECK(!dcu_can_submit(4, 0, &frame));
        CHECK(!dcu_can_submit(0, 0, NULL));
        CHECK(stage_calls == 0 && commits == 0);
    } else if (!strcmp(argv[1], "fd_roundtrip")) {
        test_roundtrip();
    } else if (!strcmp(argv[1], "invalid_lengths")) {
        test_invalid_lengths();
    } else {
        CHECK(!strcmp(argv[1], "normal"));
        CHECK(dcu_can_submit(0, 0, &frame));
        CHECK(commits == 1 && stage_calls == 1 && test_mcan[0].TXBRP == 4);
    }
    CHECK(critical_depth == 0);
    return 0;
}
