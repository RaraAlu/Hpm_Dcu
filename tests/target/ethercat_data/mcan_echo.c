#include "mcan_echo.h"
#include "cfg/mcan_echo_config.h"
#include "detail/mcan_echo_state.h"
#include "board.h"
#include "hpm_interrupt.h"
#include "realtime_port.h"
#ifdef DCU_DATA_STATIC_PROFILE
#include "static_profile.h"
#endif
#ifdef DCU_DATA_SESSION_GATE
#include "session_bridge.h"
#endif

#include <string.h>

#if !defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) || !MCAN_SOC_MSG_BUF_IN_AHB_RAM
#error "The diagnostic requires the HPM5E31 AHB message RAM layout"
#endif
#if defined(CAN_SOC_TX_RX_BUFFER_ACCESS_WORKAROUND) && CAN_SOC_TX_RX_BUFFER_ACCESS_WORKAROUND
#error "This diagnostic does not support the buffer access workaround"
#endif
_Static_assert(DCU_DATA_RX_BYTES % DCU_DATA_MCAN_FRAME_BYTES == 0, "Incomplete CAN frame");
#ifdef DCU_DATA_STATIC_PROFILE
_Static_assert(DCU_PROFILE_CAN_FRAMES * DCU_DATA_MCAN_FRAME_BYTES == DCU_DATA_RX_BYTES, "Profile/PDO size mismatch");
_Static_assert(DCU_PROFILE_CAN_ID_BASE + DCU_PROFILE_CAN_FRAMES <= 0x800U, "CAN IDs must be standard");
#else
_Static_assert(DCU_DATA_MCAN_CAN_ID_BASE + DCU_DATA_RX_BYTES / DCU_DATA_MCAN_FRAME_BYTES <= 0x800U,
               "CAN IDs must be standard");
#endif
_Static_assert(DCU_DATA_MCAN_FIFO_DEPTH > 0 && DCU_DATA_MCAN_FIFO_DEPTH < 32U, "Invalid FIFO depth");

ATTR_PLACE_AT(".ahb_sram") static uint32_t s_mcan_ram[MCAN_MSG_BUF_SIZE_IN_WORDS];
static dcu_data_mcan_state_t s_mcan;
static volatile dcu_data_mcan_status_t s_mcan_status;
static MCAN_Type *s_controller;
static uint32_t s_frame_timeout_ms;

static bool frame_id(unsigned chunk, uint16_t *id)
{
#ifdef DCU_DATA_STATIC_PROFILE
    return dcu_static_profile_can_id(dcu_static_profile()->bus, chunk, id);
#else
    *id = DCU_DATA_MCAN_CAN_ID_BASE + chunk;
    return true;
#endif
}

static bool internal_mode(void)
{
    uint32_t required = MCAN_CCCR_TEST_MASK | MCAN_CCCR_MON_MASK;
    uint32_t forbidden = MCAN_CCCR_INIT_MASK | MCAN_CCCR_FDOE_MASK | MCAN_CCCR_BRSE_MASK;
    return (s_controller->CCCR & (required | forbidden)) == required &&
           (s_controller->TEST & MCAN_TEST_LBCK_MASK) != 0U;
}

static void fail(dcu_data_mcan_status_t status)
{
    if (s_mcan.pending) mcan_cancel_tx_buf_send_request(s_controller, s_mcan.slot);
    mcan_enter_init_mode(s_controller);
    s_mcan_status = status;
    s_mcan = (dcu_data_mcan_state_t){0};
    dcu_data_reset();
}

bool dcu_data_mcan_init(void)
{
#ifdef DCU_DATA_STATIC_PROFILE
    const dcu_static_profile_t *profile = dcu_static_profile();
    MCAN_Type *const controllers[] = {HPM_MCAN0, HPM_MCAN1, HPM_MCAN2, HPM_MCAN3};
    const unsigned irqs[] = {IRQn_MCAN0, IRQn_MCAN1, IRQn_MCAN2, IRQn_MCAN3};
    s_mcan_status = MCAN_ECHO_INIT_FAILED;
    if (profile->bus >= sizeof(controllers) / sizeof(controllers[0])) return false;
    s_controller = controllers[profile->bus];
    s_frame_timeout_ms = profile->frame_timeout_ms;
    const uint32_t bitrate = profile->nominal_bitrate_bps;
    intc_m_disable_irq(irqs[profile->bus]);
#else
    s_controller = HPM_MCAN0;
    s_frame_timeout_ms = DCU_DATA_MCAN_FRAME_TIMEOUT_MS;
    const uint32_t bitrate = DCU_DATA_MCAN_NOMINAL_BITRATE_KBPS * 1000U;
    intc_m_disable_irq(IRQn_MCAN0);
#endif
    /* Deliberately do not configure CAN pinmux or a transceiver. */
    uint32_t clock_hz = board_init_can_clock(s_controller);
    mcan_msg_buf_attr_t attr = {
        .ram_base = (uint32_t)(uintptr_t)s_mcan_ram,
        .ram_size = sizeof(s_mcan_ram),
    };
    s_mcan_status = MCAN_ECHO_INIT_FAILED;
    if (!clock_hz || mcan_set_msg_buf_attr(s_controller, &attr) != status_success) return false;

    mcan_config_t config;
    mcan_get_default_config(s_controller, &config);
    config.mode = mcan_mode_loopback_internal;
    config.baudrate = bitrate;
    config.enable_canfd = false;
    config.enable_tdc = false;
    mcan_get_default_ram_config(s_controller, &config.ram_config, false);
    config.ram_config.enable_std_filter = false;
    config.ram_config.enable_ext_filter = false;
    config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 0U;
    config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0U;
    config.all_filters_config.global_filter_config.reject_remote_std_frame = true;
    config.all_filters_config.global_filter_config.reject_remote_ext_frame = true;
    config.all_filters_config.global_filter_config.accept_non_matching_std_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_IN_RXFIFO0;
    config.all_filters_config.global_filter_config.accept_non_matching_ext_frame_option =
        MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    config.ram_config.rxfifos[0].enable = 1U;
    config.ram_config.rxfifos[0].elem_count = DCU_DATA_MCAN_FIFO_DEPTH;
    config.ram_config.rxfifos[0].data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
    config.ram_config.rxfifos[1].enable = 0U;
    config.ram_config.enable_rxbuf = false;
    config.ram_config.txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
    config.ram_config.txbuf_dedicated_txbuf_elem_count = 0U;
    config.ram_config.txbuf_fifo_or_queue_elem_count = DCU_DATA_MCAN_FIFO_DEPTH;
    config.ram_config.txfifo_or_txqueue_mode = 0U;
    config.ram_config.enable_tx_evt_fifo = false;
    config.txbuf_trans_interrupt_mask = (1U << DCU_DATA_MCAN_FIFO_DEPTH) - 1U;
    config.interrupt_mask = 0U;
    if (mcan_init(s_controller, &config, clock_hz) != status_success) {
        fail(MCAN_ECHO_INIT_FAILED);
        return false;
    }
    if (!internal_mode()) {
        fail(MCAN_ECHO_WRONG_MODE);
        return false;
    }
    mcan_clear_interrupt_flags(s_controller, UINT32_MAX);
    s_mcan_status = MCAN_ECHO_READY;
    return true;
}

bool dcu_data_mcan_enable(void)
{
    if (s_mcan_status != MCAN_ECHO_READY) return false;
    if (!internal_mode()) {
        fail(MCAN_ECHO_WRONG_MODE);
        return false;
    }
    s_mcan.enabled = true;
    return true;
}

void dcu_data_mcan_disable(void)
{
    s_mcan.enabled = false;
    s_mcan.staged = false;
    s_mcan.discard = s_mcan.pending != 0U;
}

bool dcu_data_mcan_healthy(void)
{
    return s_mcan_status == MCAN_ECHO_READY;
}

bool dcu_data_mcan_stage(const void *data, size_t size)
{
    if (!s_mcan.enabled || s_mcan.staged || s_mcan.pending || !data ||
        size != sizeof(s_mcan.request)) return false;
    memcpy(s_mcan.request, data, size);
    s_mcan.chunk = 0U;
    s_mcan.staged = true;
    return true;
}

static void submit(uint32_t now_us, unsigned offset)
{
    mcan_tx_frame_t tx = {0};
    uint16_t id;
    if (!frame_id(s_mcan.chunk, &id)) {
        fail(MCAN_ECHO_TX_FAILED);
        return;
    }
    tx.std_id = id;
    tx.dlc = DCU_DATA_MCAN_FRAME_BYTES;
    memcpy(tx.data_8, s_mcan.request + offset, DCU_DATA_MCAN_FRAME_BYTES);
    uint32_t slot;
    if (mcan_request_and_fill_txfifo(s_controller, &tx, &slot) != status_success ||
        slot >= DCU_DATA_MCAN_FIFO_DEPTH) {
        fail(MCAN_ECHO_TX_FAILED);
        return;
    }
    if (!internal_mode()) {
        fail(MCAN_ECHO_WRONG_MODE);
        return;
    }
    s_mcan.slot = slot;
    s_mcan.pending = 1U << slot;
    s_mcan.started_us = now_us;
    s_mcan.rx_done = s_mcan.tx_done = false;
    mcan_send_add_request(s_controller, slot);
}

void dcu_data_mcan_poll(void)
{
    if (s_mcan_status != MCAN_ECHO_READY) return;
    if (!internal_mode()) {
        fail(MCAN_ECHO_WRONG_MODE);
        return;
    }
    uint32_t flags = mcan_get_interrupt_flags(s_controller);
    mcan_clear_interrupt_flags(s_controller, flags);
    if (flags & DCU_DATA_MCAN_ERROR_MASK) {
        fail(MCAN_ECHO_CONTROLLER_ERROR);
        return;
    }

    unsigned count = mcan_get_rxfifo_fill_level(s_controller, 0U);
    uint32_t now_us = dcu_time_now_us();
    unsigned offset = s_mcan.chunk * DCU_DATA_MCAN_FRAME_BYTES;
    if (!s_mcan.pending) {
        if (count != 0U) fail(MCAN_ECHO_RX_MISMATCH);
        else if (s_mcan.staged) submit(now_us, offset);
        return;
    }
    if (now_us - s_mcan.started_us >= s_frame_timeout_ms * 1000U) {
        fail(MCAN_ECHO_TIMEOUT);
        return;
    }
    if ((flags & MCAN_EVENT_TRANSMIT) && (s_controller->TXBTO & s_mcan.pending)) s_mcan.tx_done = true;
    if (count != 0U) {
        mcan_rx_message_t rx;
        uint16_t id;
        if (!frame_id(s_mcan.chunk, &id) || count != 1U || s_mcan.rx_done ||
            mcan_read_rxfifo(s_controller, 0U, &rx) != status_success ||
            rx.use_ext_id || rx.rtr || rx.canfd_frame || rx.bitrate_switch ||
            rx.std_id != id ||
            rx.dlc != DCU_DATA_MCAN_FRAME_BYTES ||
            memcmp(rx.data_8, s_mcan.request + offset, DCU_DATA_MCAN_FRAME_BYTES) != 0) {
            fail(MCAN_ECHO_RX_MISMATCH);
            return;
        }
        memcpy(s_mcan.response + offset, rx.data_8, DCU_DATA_MCAN_FRAME_BYTES);
        s_mcan.rx_done = true;
    }
    if (!s_mcan.tx_done || !s_mcan.rx_done) return;

    s_mcan.pending = 0U;
    /* An in-flight frame may finish after OP stops, but its PDO is discarded. */
    if (s_mcan.discard) {
        s_mcan.discard = false;
    } else if (++s_mcan.chunk == DCU_DATA_RX_BYTES / DCU_DATA_MCAN_FRAME_BYTES) {
        s_mcan.staged = false;
#ifdef DCU_DATA_SESSION_GATE
        dcu_data_session_can_complete(s_mcan.response, sizeof(s_mcan.response));
#else
        (void)dcu_data_capture(s_mcan.response, sizeof(s_mcan.response));
#endif
    }
}
