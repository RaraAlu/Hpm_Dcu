#include "board.h"
#include "cfg/mcan_echo_config.h"
#include "data_contract.h"
#include "mcan_echo.h"
#include "realtime_port.h"

#include <assert.h>
#include <string.h>

#ifdef TEST_ALTERNATE_PROFILE
#include "static_profile.h"
static const unsigned expected_bus = 2U, expected_bytes = 16U, expected_can_id = 0x650U;
static const unsigned expected_timeout_ms = 7U, expected_product = 0x7fff0004U;
#else
static const unsigned expected_bus = 0U;
#ifndef DCU_DATA_SESSION_GATE
static const unsigned expected_bytes = 32U, expected_can_id = 0x600U;
static const unsigned expected_timeout_ms = 5U, expected_product = 0x7fff0003U;
#endif
#endif
static MCAN_Type controllers[4];
static unsigned init_count, irq_count;
static mcan_tx_frame_t staged;
static mcan_rx_message_t received;
static unsigned tx_count, fifo_fill;
static uint32_t flags, now_us = UINT32_MAX - 50U;

MCAN_Type *test_loopback_controller_at(unsigned bus) { return &controllers[bus]; }
MCAN_Type *test_loopback_controller(void) { return test_loopback_controller_at(0U); }
void intc_m_disable_irq(unsigned irq)
{
    assert(irq == IRQn_MCAN0 + expected_bus);
    ++irq_count;
}
uint32_t dcu_time_now_us(void) { return now_us; }
uint32_t board_init_can_clock(MCAN_Type *base)
{
    assert(base == &controllers[expected_bus]);
    return 80000000U;
}

hpm_stat_t mcan_set_msg_buf_attr(MCAN_Type *ptr, const mcan_msg_buf_attr_t *attr)
{
    assert(ptr == &controllers[expected_bus] && attr->ram_size >= 256U);
    return status_success;
}
void mcan_get_default_config(MCAN_Type *ptr, mcan_config_t *config)
{
    assert(ptr == &controllers[expected_bus]);
    memset(config, 0, sizeof(*config));
}
void mcan_get_default_ram_config(MCAN_Type *ptr, mcan_ram_config_t *config, bool fd)
{
    assert(ptr == &controllers[expected_bus] && !fd);
    memset(config, 0, sizeof(*config));
}
hpm_stat_t mcan_init(MCAN_Type *ptr, const mcan_config_t *config, uint32_t clock)
{
    assert(ptr == &controllers[expected_bus] && clock == 80000000U);
    ++init_count;
    assert(config->mode == mcan_mode_loopback_internal && !config->enable_canfd);
    assert(config->baudrate == 1000000U && !config->interrupt_mask);
    ptr->CCCR = MCAN_CCCR_TEST_MASK | MCAN_CCCR_MON_MASK;
    ptr->TEST = MCAN_TEST_LBCK_MASK;
    return status_success;
}
uint32_t mcan_get_interrupt_flags(MCAN_Type *ptr) { assert(ptr == &controllers[expected_bus]); return flags; }
void mcan_clear_interrupt_flags(MCAN_Type *ptr, uint32_t mask)
{
    assert(ptr == &controllers[expected_bus]);
    flags &= ~mask;
}
unsigned mcan_get_rxfifo_fill_level(MCAN_Type *ptr, unsigned fifo)
{
    assert(ptr == &controllers[expected_bus] && fifo == 0U);
    return fifo_fill;
}
hpm_stat_t mcan_read_rxfifo(MCAN_Type *ptr, unsigned fifo, mcan_rx_message_t *frame)
{
    assert(ptr == &controllers[expected_bus] && fifo == 0U && fifo_fill == 1U);
    *frame = received;
    --fifo_fill;
    return status_success;
}
hpm_stat_t mcan_request_and_fill_txfifo(MCAN_Type *ptr, mcan_tx_frame_t *frame, uint32_t *slot)
{
    assert(ptr == &controllers[expected_bus] && !fifo_fill && !flags);
    assert(frame->dlc == 8U && !frame->use_ext_id && !frame->rtr && !frame->canfd_frame && !frame->bitrate_switch);
    staged = *frame;
    *slot = 0U;
    return status_success;
}
void mcan_send_add_request(MCAN_Type *ptr, uint32_t slot)
{
    assert(ptr == &controllers[expected_bus] && slot == 0U);
    assert((ptr->CCCR & (MCAN_CCCR_TEST_MASK | MCAN_CCCR_MON_MASK)) ==
                        (MCAN_CCCR_TEST_MASK | MCAN_CCCR_MON_MASK));
    assert(ptr->TEST & MCAN_TEST_LBCK_MASK);
    ++tx_count;
    ptr->TXBTO = 0U;
}
void mcan_cancel_tx_buf_send_request(MCAN_Type *ptr, unsigned slot)
{
    assert(ptr == &controllers[expected_bus] && slot == 0U);
}
void mcan_enter_init_mode(MCAN_Type *ptr)
{
    assert(ptr == &controllers[expected_bus]);
    ptr->CCCR |= MCAN_CCCR_INIT_MASK;
}

#ifdef DCU_DATA_SESSION_GATE
unsigned test_mcan_sent(void) { return tx_count; }
void test_mcan_advance(unsigned us) { now_us += us; }
void test_mcan_finish_frame(bool matches)
{
    assert(tx_count > 0U && !fifo_fill);
    now_us += 100U;
    received = staged;
    if (!matches) received.std_id++;
    fifo_fill = 1U;
    flags = MCAN_EVENT_TRANSMIT;
    controllers[expected_bus].TXBTO = 1U;
    dcu_data_mcan_poll();
}
#else
static void expect_pdo(unsigned counter, const uint8_t *expected)
{
    uint8_t tx[DCU_DATA_TX_BYTES];
    assert(dcu_data_copy(tx, sizeof(tx)));
    assert(tx[expected_bytes] == counter && tx[expected_bytes + 8U] == 1U);
    assert(tx[expected_bytes + 4U] == (expected ? expected_bytes : 0U));
    assert(tx[expected_bytes + 12U] == (expected ? 1U : 0U));
    if (expected) assert(memcmp(tx, expected, expected_bytes) == 0);
}

static void tx_done(void)
{
    now_us += 100U;
    flags |= MCAN_EVENT_TRANSMIT;
    controllers[expected_bus].TXBTO = 1U;
    dcu_data_mcan_poll();
}

static void rx_done(void)
{
    now_us += 100U;
    fifo_fill = 1U;
    dcu_data_mcan_poll();
}

static void exchange(const uint8_t *payload, unsigned count_before)
{
    uint8_t overwrite[DCU_DATA_RX_BYTES] = {0};
    const unsigned frames = expected_bytes / 8U;
    dcu_data_mcan_stage(payload, expected_bytes);
    for (unsigned i = 0; i < frames; ++i) {
        dcu_data_mcan_poll();
        assert(staged.std_id == expected_can_id + i);
        assert(memcmp(staged.data_8, payload + 8U * i, 8U) == 0);
        received = staged;
        dcu_data_mcan_stage(overwrite, sizeof(overwrite));
        if (i % 2U == 0U) {
            tx_done();
            expect_pdo(0, NULL);
            rx_done();
        } else {
            rx_done();
            expect_pdo(0, NULL);
            tx_done();
        }
        if (i + 1U < frames) expect_pdo(0, NULL);
    }
    assert(tx_count == count_before + frames);
    expect_pdo(1U, payload);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    uint64_t sii = 0U;
    assert(dcu_data_sii_read(10U, 4U, &sii) && sii == expected_product);
#ifdef TEST_ALTERNATE_PROFILE
    uint16_t id = 0x7ffU;
    assert(!dcu_static_profile_can_id(0U, 0U, &id) && id == 0x7ffU);
    assert(!dcu_static_profile_can_id(2U, 2U, &id));
    assert(!dcu_static_profile_can_id(2U, 0U, NULL));
    assert(dcu_static_profile_can_id(2U, 1U, &id) && id == 0x651U);
    assert(strcmp(dcu_static_profile()->device_id, "probe_alt") == 0);
#endif
    dcu_data_reset();
    assert(dcu_data_mcan_init());
    assert(init_count == 1U && irq_count == 1U);
    assert(DCU_DATA_RX_BYTES == expected_bytes && DCU_DATA_TX_BYTES == expected_bytes + 16U);
    uint8_t payload[DCU_DATA_RX_BYTES];
    for (unsigned i = 0; i < sizeof(payload); ++i) payload[i] = i;
    dcu_data_mcan_stage(payload, sizeof(payload));
    dcu_data_mcan_poll();
    assert(tx_count == 0U);
    assert(dcu_data_mcan_enable());
    dcu_data_mcan_stage(payload, sizeof(payload) - 1U);
    dcu_data_mcan_poll();
    assert(tx_count == 0U);
    if (!strcmp(argv[1], "exchange")) {
        exchange(payload, 0U);
        return 0;
    }

    dcu_data_mcan_stage(payload, sizeof(payload));
    dcu_data_mcan_poll();
    assert(tx_count == 1U);
    received = staged;
    if (!strcmp(argv[1], "leave_op")) {
        dcu_data_mcan_disable();
        dcu_data_reset();
        rx_done();
        tx_done();
        dcu_data_mcan_poll();
        assert(tx_count == 1U);
        expect_pdo(0, NULL);
        assert(dcu_data_mcan_enable());
        exchange(payload, 1U);
        return 0;
    }
    if (!strcmp(argv[1], "bad_id")) {
        received.std_id++;
        rx_done();
    } else if (!strcmp(argv[1], "bad_data")) {
        received.data_8[7] ^= 1U;
        rx_done();
    } else if (!strcmp(argv[1], "fd")) {
        received.canfd_frame = true;
        rx_done();
    } else if (!strcmp(argv[1], "duplicate")) {
        fifo_fill = 2U;
        dcu_data_mcan_poll();
    } else if (!strcmp(argv[1], "no_tx")) {
        rx_done();
        now_us += expected_timeout_ms * 1000U - 101U;
        dcu_data_mcan_poll();
        assert(!(controllers[expected_bus].CCCR & MCAN_CCCR_INIT_MASK));
        ++now_us;
        dcu_data_mcan_poll();
    } else if (!strcmp(argv[1], "error")) {
        flags |= MCAN_INT_MSG_RAM_ACCESS_FAILURE;
        dcu_data_mcan_poll();
    } else if (!strcmp(argv[1], "mode_lost")) {
        controllers[expected_bus].CCCR &= ~MCAN_CCCR_MON_MASK;
        dcu_data_mcan_poll();
    } else {
        assert(false);
    }
    assert(controllers[expected_bus].CCCR & MCAN_CCCR_INIT_MASK);
    assert(!dcu_data_mcan_enable());
    dcu_data_mcan_stage(payload, sizeof(payload));
    dcu_data_mcan_poll();
    assert(tx_count == 1U);
    expect_pdo(0, NULL);
}
#endif
