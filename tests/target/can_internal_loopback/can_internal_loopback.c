#include "can_internal_loopback.h"
#include "detail/can_loopback_state.h"
#include "board.h"
#include "cycle_timers.h"
#include "realtime_port.h"
#include "hpm_interrupt.h"
#include <string.h>

#if !defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) || !MCAN_SOC_MSG_BUF_IN_AHB_RAM
#error "This diagnostic is for the HPM5E31 AHB message RAM layout"
#endif
#if defined(CAN_SOC_TX_RX_BUFFER_ACCESS_WORKAROUND) && CAN_SOC_TX_RX_BUFFER_ACCESS_WORKAROUND
#error "Split TX commit needs the SoC buffer access workaround"
#endif

_Static_assert(DCU_LOOPBACK_FRAME_COUNT > 0U, "At least one frame is required");
_Static_assert(DCU_LOOPBACK_FIFO_DEPTH > 0U && DCU_LOOPBACK_FIFO_DEPTH < 32U, "Invalid FIFO depth");
_Static_assert(DCU_LOOPBACK_QUEUE_DEPTH > 1U, "Queue needs a spare slot");
_Static_assert(DCU_LOOPBACK_TIMEOUT_MS > 0U && DCU_LOOPBACK_TIMEOUT_MS < UINT32_MAX / 1000U,
               "Invalid timeout");

ATTR_PLACE_AT(".ahb_sram") static uint32_t s_message_ram[MCAN_MSG_BUF_SIZE_IN_WORDS];
static dcu_loopback_irq_state_t s_irq;
static volatile dcu_can_loopback_report_t s_report;
static const uint8_t s_payloads[][8] = {
    {0x44, 0x33, 0x22, 0x11, 0x04, 0x03, 0x02, 0x01},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
};

static bool internal_mode(void)
{
    uint32_t required = MCAN_CCCR_TEST_MASK | MCAN_CCCR_MON_MASK;
    uint32_t forbidden = MCAN_CCCR_INIT_MASK | MCAN_CCCR_FDOE_MASK | MCAN_CCCR_BRSE_MASK;
    return (HPM_MCAN0->CCCR & (required | forbidden)) == required &&
           (HPM_MCAN0->TEST & MCAN_TEST_LBCK_MASK) != 0U;
}

static void loopback_isr(void)
{
    uint32_t flags = mcan_get_interrupt_flags(HPM_MCAN0);
    mcan_clear_interrupt_flags(HPM_MCAN0, flags);
    ++s_irq.irq_count;
    s_irq.error_flags |= flags & DCU_LOOPBACK_ERROR_MASK;
    if ((flags & MCAN_EVENT_TRANSMIT) && (HPM_MCAN0->TXBTO & s_irq.pending_mask)) {
        ++s_irq.tx_completed;
        s_irq.pending_mask = 0U;
    }
    for (unsigned count = 0; count < DCU_LOOPBACK_FIFO_DEPTH; ++count) {
        if (mcan_get_rxfifo_fill_level(HPM_MCAN0, 0U) == 0U) break;
        dcu_loopback_rx_item_t item = {0};
        if (mcan_read_rxfifo(HPM_MCAN0, 0U, &item.frame) != status_success) {
            s_irq.read_failed = true;
            break;
        }
        item.arrival_us = dcu_time_now_us();
        ++s_irq.rx_received;
        uint32_t next = (s_irq.write + 1U) % DCU_LOOPBACK_QUEUE_DEPTH;
        if (next == s_irq.read) {
            ++s_irq.queue_drops;
        } else {
            s_irq.items[s_irq.write] = item;
            __asm__ volatile("" ::: "memory");
            s_irq.write = next;
        }
    }
    if (s_irq.read_failed || mcan_get_rxfifo_fill_level(HPM_MCAN0, 0U) != 0U) {
        s_irq.read_failed = true;
    }
    if (s_irq.read_failed || s_irq.error_flags || s_irq.queue_drops) {
        mcan_disable_interrupts(HPM_MCAN0, UINT32_MAX);
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, loopback_isr);

static bool receive(dcu_loopback_rx_item_t *item)
{
    intc_m_disable_irq(IRQn_MCAN0);
    bool available = s_irq.read != s_irq.write;
    if (available) {
        *item = s_irq.items[s_irq.read];
        s_irq.read = (s_irq.read + 1U) % DCU_LOOPBACK_QUEUE_DEPTH;
    }
    intc_m_enable_irq(IRQn_MCAN0);
    return available;
}

static dcu_can_loopback_result_t irq_result(void)
{
    if (s_irq.queue_drops) return DCU_LOOPBACK_QUEUE_OVERFLOW;
    if (s_irq.error_flags || s_irq.read_failed) return DCU_LOOPBACK_IRQ_FAILED;
    return DCU_LOOPBACK_PASS;
}

static hpm_stat_t configure(uint32_t clock_hz)
{
    mcan_msg_buf_attr_t attr = {
        .ram_base = (uint32_t)(uintptr_t)s_message_ram,
        .ram_size = sizeof(s_message_ram),
    };
    hpm_stat_t status = mcan_set_msg_buf_attr(HPM_MCAN0, &attr);
    if (status != status_success) return status;
    mcan_config_t config;
    mcan_get_default_config(HPM_MCAN0, &config);
    config.mode = mcan_mode_loopback_internal;
    config.baudrate = DCU_LOOPBACK_NOMINAL_BITRATE_KBPS * 1000U;
    config.enable_canfd = false;
    config.enable_tdc = false;
    mcan_get_default_ram_config(HPM_MCAN0, &config.ram_config, false);
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
    config.ram_config.rxfifos[0].elem_count = DCU_LOOPBACK_FIFO_DEPTH;
    config.ram_config.rxfifos[0].data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
    config.ram_config.rxfifos[1].enable = 0U;
    config.ram_config.enable_rxbuf = false;
    config.ram_config.txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
    config.ram_config.txbuf_dedicated_txbuf_elem_count = 0U;
    config.ram_config.txbuf_fifo_or_queue_elem_count = DCU_LOOPBACK_FIFO_DEPTH;
    config.ram_config.txfifo_or_txqueue_mode = 0U;
    config.ram_config.enable_tx_evt_fifo = false;
    config.txbuf_trans_interrupt_mask = (1U << DCU_LOOPBACK_FIFO_DEPTH) - 1U;
    config.interrupt_mask = DCU_LOOPBACK_IRQ_MASK;
    status = mcan_init(HPM_MCAN0, &config, clock_hz);
    if (status == status_success) {
        mcan_interrupt_line_select(HPM_MCAN0, config.interrupt_mask, 0U);
    }
    return status;
}

static dcu_can_loopback_result_t exchange(unsigned frame_index)
{
    mcan_tx_frame_t tx = {0};
    tx.std_id = DCU_LOOPBACK_CAN_ID;
    tx.dlc = 8U;
    memcpy(tx.data_8, s_payloads[frame_index % 3U], sizeof(s_payloads[0]));
    uint32_t start_us = dcu_time_now_us();
    uint32_t slot = 0U;
    intc_m_disable_irq(IRQn_MCAN0);
    dcu_can_loopback_result_t irq_status = irq_result();
    if (irq_status != DCU_LOOPBACK_PASS) {
        intc_m_enable_irq(IRQn_MCAN0);
        return irq_status;
    }
    if (!internal_mode()) {
        intc_m_enable_irq(IRQn_MCAN0);
        return DCU_LOOPBACK_MODE_FAILED;
    }
    hpm_stat_t status = mcan_request_and_fill_txfifo(HPM_MCAN0, &tx, &slot);
    if (status != status_success || slot >= DCU_LOOPBACK_FIFO_DEPTH) {
        s_report.sdk_status = status;
        intc_m_enable_irq(IRQn_MCAN0);
        return DCU_LOOPBACK_TX_FAILED;
    }
    s_irq.pending_mask = 1U << slot;
    mcan_send_add_request(HPM_MCAN0, slot);
    intc_m_enable_irq(IRQn_MCAN0);

    bool received = false;
    for (unsigned poll = 0; poll < DCU_LOOPBACK_POLL_LIMIT; ++poll) {
        dcu_can_loopback_result_t result = irq_result();
        if (result != DCU_LOOPBACK_PASS) return result;
        uint32_t elapsed_us = dcu_time_now_us() - start_us;
        if (elapsed_us >= DCU_LOOPBACK_TIMEOUT_MS * 1000U) return DCU_LOOPBACK_TIMEOUT;
        dcu_loopback_rx_item_t item;
        if (receive(&item)) {
            if (received) return DCU_LOOPBACK_EXTRA_RX;
            received = true;
            volatile dcu_can_loopback_frame_result_t *trace = &s_report.frames[frame_index];
            trace->elapsed_us = item.arrival_us - start_us;
            trace->received_id = item.frame.use_ext_id ? item.frame.ext_id : item.frame.std_id;
            trace->received_dlc = item.frame.dlc;
            trace->received_flags = (item.frame.use_ext_id ? 2U : 0U) | (item.frame.rtr ? 4U : 0U) |
                (item.frame.canfd_frame ? 8U : 0U) | (item.frame.bitrate_switch ? 16U : 0U);
            for (unsigned i = 0; i < 8U; ++i) trace->received_data[i] = item.frame.data_8[i];
            if (trace->elapsed_us >= DCU_LOOPBACK_TIMEOUT_MS * 1000U) return DCU_LOOPBACK_TIMEOUT;
            if (trace->received_id != tx.std_id || trace->received_dlc != 8U ||
                trace->received_flags != 0U || memcmp(item.frame.data_8, tx.data_8, 8U) != 0) {
                return DCU_LOOPBACK_RX_MISMATCH;
            }
        }
        if (s_irq.rx_received > frame_index + 1U) return DCU_LOOPBACK_EXTRA_RX;
        if (received && s_irq.tx_completed == frame_index + 1U) {
            s_report.frames[frame_index].matched = true;
            return DCU_LOOPBACK_PASS;
        }
    }
    return DCU_LOOPBACK_TIMEOUT;
}

static dcu_can_loopback_result_t quiet_check(void)
{
    uint32_t start_us = dcu_time_now_us();
    for (unsigned poll = 0; poll < DCU_LOOPBACK_POLL_LIMIT; ++poll) {
        dcu_can_loopback_result_t result = irq_result();
        if (result != DCU_LOOPBACK_PASS) return result;
        if (s_irq.read != s_irq.write || s_irq.rx_received != DCU_LOOPBACK_FRAME_COUNT) {
            return DCU_LOOPBACK_EXTRA_RX;
        }
        if (dcu_time_now_us() - start_us >= DCU_LOOPBACK_TIMEOUT_MS * 1000U) return DCU_LOOPBACK_PASS;
    }
    return DCU_LOOPBACK_TIMEOUT;
}

static bool stop_controller(void)
{
    intc_m_disable_irq(IRQn_MCAN0);
    mcan_disable_interrupts(HPM_MCAN0, UINT32_MAX);
    s_irq.error_flags |= mcan_get_interrupt_flags(HPM_MCAN0) & DCU_LOOPBACK_ERROR_MASK;
    for (unsigned slot = 0; slot < DCU_LOOPBACK_FIFO_DEPTH; ++slot) {
        mcan_cancel_tx_buf_send_request(HPM_MCAN0, slot);
    }
    /* Keep internal loopback selected even if stopping times out. */
    mcan_enter_init_mode(HPM_MCAN0);
    for (unsigned poll = 0; poll < DCU_LOOPBACK_POLL_LIMIT; ++poll) {
        if (HPM_MCAN0->CCCR & MCAN_CCCR_INIT_MASK) {
            s_report.controller_stopped = true;
            break;
        }
    }
    return s_report.controller_stopped;
}

void dcu_can_loopback_run(void)
{
    if (s_report.result != DCU_LOOPBACK_NOT_RUN) return;
    s_report.result = DCU_LOOPBACK_RUNNING;
    intc_m_disable_irq(IRQn_MCAN0);
    if (!dcu_timebase_init()) {
        s_report.result = DCU_LOOPBACK_TIMEBASE_FAILED;
        return;
    }
    /* No CAN pinmux or transceiver enable: only the controller's internal path. */
    uint32_t clock_hz = board_init_can_clock(HPM_MCAN0);
    if (clock_hz == 0U) {
        s_report.result = DCU_LOOPBACK_SETUP_FAILED;
        return;
    }
    s_report.sdk_status = configure(clock_hz);
    s_report.mode_cccr = HPM_MCAN0->CCCR;
    s_report.mode_test = HPM_MCAN0->TEST;
    dcu_can_loopback_result_t result;
    if (s_report.sdk_status != status_success) {
        result = DCU_LOOPBACK_SETUP_FAILED;
    } else if (!internal_mode()) {
        result = DCU_LOOPBACK_MODE_FAILED;
    } else {
        intc_m_enable_irq_with_priority(IRQn_MCAN0, 1U);
        for (unsigned i = 0; i < DCU_LOOPBACK_FRAME_COUNT; ++i) {
            result = exchange(i);
            if (result != DCU_LOOPBACK_PASS) break;
            ++s_report.frames_checked;
        }
        if (result == DCU_LOOPBACK_PASS) result = quiet_check();
    }
    bool stopped = stop_controller();
    s_report.irq_count = s_irq.irq_count;
    s_report.tx_completed = s_irq.tx_completed;
    s_report.rx_received = s_irq.rx_received;
    s_report.queue_drops = s_irq.queue_drops;
    s_report.error_flags = s_irq.error_flags;
    /* Recheck after masking IRQs so a final interrupt cannot leave a false PASS. */
    if (result == DCU_LOOPBACK_PASS) {
        result = irq_result();
        if (result == DCU_LOOPBACK_PASS && (s_irq.rx_received != DCU_LOOPBACK_FRAME_COUNT ||
            s_irq.tx_completed != DCU_LOOPBACK_FRAME_COUNT || s_irq.read != s_irq.write ||
            mcan_get_rxfifo_fill_level(HPM_MCAN0, 0U) != 0U)) {
            result = DCU_LOOPBACK_EXTRA_RX;
        }
    }
    s_report.result = stopped ? result : DCU_LOOPBACK_STOP_FAILED;
}

const volatile dcu_can_loopback_report_t *dcu_can_loopback_report(void)
{
    return &s_report;
}
