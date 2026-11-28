#include "hpm_mcan_transport.h"
#include "can_transport.h"
#include "cfg/can_bus_config.h"
#include "detail/can_gateway_state.h"
#include "realtime_port.h"
#include "cycle_config.h"
#include "ethercat_cycle.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_mcan_drv.h"

#if defined(CAN_SOC_TX_RX_BUFFER_ACCESS_WORKAROUND) && CAN_SOC_TX_RX_BUFFER_ACCESS_WORKAROUND == 1
#error "The split TX commit must implement this SoC's MCAN buffer-access workaround"
#endif

_Static_assert(DCU_CAN_MAX_PAYLOAD == 64U, "MCAN FIFO payload setting must match the internal frame");

/* 每路依次为 TX、RX、ERR，使用自研板上已有的 12 个低电平点亮 LED。 */
static const canfd_led_t s_led[DCU_CAN_BUS_COUNT][CANFD_LED_KIND_COUNT] = {
    {
        { BOARD_CAN0_TX_LED_CTRL, BOARD_CAN0_TX_LED_INDEX, BOARD_CAN0_TX_LED_PIN, BOARD_CAN0_TX_LED_ON_LEVEL, BOARD_CAN0_TX_LED_OFF_LEVEL },
        { BOARD_CAN0_RX_LED_CTRL, BOARD_CAN0_RX_LED_INDEX, BOARD_CAN0_RX_LED_PIN, BOARD_CAN0_RX_LED_ON_LEVEL, BOARD_CAN0_RX_LED_OFF_LEVEL },
        { BOARD_CAN0_ERR_LED_CTRL, BOARD_CAN0_ERR_LED_INDEX, BOARD_CAN0_ERR_LED_PIN, BOARD_CAN0_ERR_LED_ON_LEVEL, BOARD_CAN0_ERR_LED_OFF_LEVEL },
    },
    {
        { BOARD_CAN1_TX_LED_CTRL, BOARD_CAN1_TX_LED_INDEX, BOARD_CAN1_TX_LED_PIN, BOARD_CAN1_TX_LED_ON_LEVEL, BOARD_CAN1_TX_LED_OFF_LEVEL },
        { BOARD_CAN1_RX_LED_CTRL, BOARD_CAN1_RX_LED_INDEX, BOARD_CAN1_RX_LED_PIN, BOARD_CAN1_RX_LED_ON_LEVEL, BOARD_CAN1_RX_LED_OFF_LEVEL },
        { BOARD_CAN1_ERR_LED_CTRL, BOARD_CAN1_ERR_LED_INDEX, BOARD_CAN1_ERR_LED_PIN, BOARD_CAN1_ERR_LED_ON_LEVEL, BOARD_CAN1_ERR_LED_OFF_LEVEL },
    },
    {
        { BOARD_CAN2_TX_LED_CTRL, BOARD_CAN2_TX_LED_INDEX, BOARD_CAN2_TX_LED_PIN, BOARD_CAN2_TX_LED_ON_LEVEL, BOARD_CAN2_TX_LED_OFF_LEVEL },
        { BOARD_CAN2_RX_LED_CTRL, BOARD_CAN2_RX_LED_INDEX, BOARD_CAN2_RX_LED_PIN, BOARD_CAN2_RX_LED_ON_LEVEL, BOARD_CAN2_RX_LED_OFF_LEVEL },
        { BOARD_CAN2_ERR_LED_CTRL, BOARD_CAN2_ERR_LED_INDEX, BOARD_CAN2_ERR_LED_PIN, BOARD_CAN2_ERR_LED_ON_LEVEL, BOARD_CAN2_ERR_LED_OFF_LEVEL },
    },
    {
        { BOARD_CAN3_TX_LED_CTRL, BOARD_CAN3_TX_LED_INDEX, BOARD_CAN3_TX_LED_PIN, BOARD_CAN3_TX_LED_ON_LEVEL, BOARD_CAN3_TX_LED_OFF_LEVEL },
        { BOARD_CAN3_RX_LED_CTRL, BOARD_CAN3_RX_LED_INDEX, BOARD_CAN3_RX_LED_PIN, BOARD_CAN3_RX_LED_ON_LEVEL, BOARD_CAN3_RX_LED_OFF_LEVEL },
        { BOARD_CAN3_ERR_LED_CTRL, BOARD_CAN3_ERR_LED_INDEX, BOARD_CAN3_ERR_LED_PIN, BOARD_CAN3_ERR_LED_ON_LEVEL, BOARD_CAN3_ERR_LED_OFF_LEVEL },
    },
};

/* 物理 CANFD1～4 与 MCAN0～3 一一对应。 */
static MCAN_Type *const s_mcan[DCU_CAN_BUS_COUNT] = {
    HPM_MCAN0, HPM_MCAN1, HPM_MCAN2, HPM_MCAN3
};

static const uint32_t s_irq[DCU_CAN_BUS_COUNT] = {
    IRQn_MCAN0, IRQn_MCAN1, IRQn_MCAN2, IRQn_MCAN3
};

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && MCAN_SOC_MSG_BUF_IN_AHB_RAM
/* HPM5E31 的 MCAN Message RAM 位于专用 AHB SRAM，四路必须各自独立。 */
_Static_assert((CANFD_RX_FIFO_DEPTH + CANFD_TX_SLOTS) *
                   (MCAN_MESSAGE_HEADER_SIZE_IN_BYTES + DCU_CAN_MAX_PAYLOAD) +
                   CANFD_TX_FIFO_DEPTH * sizeof(mcan_tx_event_fifo_elem_t) + sizeof(uint32_t) <=
                   MCAN_MSG_BUF_SIZE_IN_WORDS * sizeof(uint32_t),
               "MCAN FIFOs exceed Message RAM");
ATTR_PLACE_AT(".ahb_sram") static uint32_t s_mcan_ram[DCU_CAN_BUS_COUNT][MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

/* One ISR producer and one worker consumer per bus. */
static rx_queue_t s_rx[DCU_CAN_BUS_COUNT];
/* The FIFO layouts do not change after mcan_init. Access message RAM as aligned words. */
static const volatile uint32_t *s_rx_fifo0[DCU_CAN_BUS_COUNT];
static const volatile uint32_t *s_tx_events[DCU_CAN_BUS_COUNT];
_Static_assert(offsetof(mcan_rx_message_t, data_8) == 2 * sizeof(uint32_t) &&
               sizeof(mcan_rx_message_t) == 2 * sizeof(uint32_t) + DCU_CAN_MAX_PAYLOAD,
               "RX FIFO0 must use two header words and a 64-byte element payload");
_Static_assert(sizeof(mcan_tx_event_fifo_elem_t) == 2 * sizeof(uint32_t), "TX event stride");
_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "MCAN ISR requires lock-free queue indices and bus state");
static atomic_uint s_bus_off[DCU_CAN_BUS_COUNT];
#if defined(CONFIG_CAN_BATCH_SAMPLE)
/* Same-priority CAN IRQ handlers on this core do not nest each other. */
static volatile uint32_t can_irq_total_us;
#endif
static volatile uint32_t s_rx_lost[DCU_CAN_BUS_COUNT];
static uint32_t s_tx_progress[DCU_CAN_BUS_COUNT];
static tx_slot_t s_tx[DCU_CAN_BUS_COUNT][CANFD_TX_SLOTS];
static paced_tx_t s_chain[DCU_CAN_BUS_COUNT];
/* Read after the run without stopping the CPU or depending on the SDO mailbox. */
static volatile feedback_stats_t s_feedback_stats[DCU_CAN_BUS_COUNT] __attribute__((section(".noncacheable.bss")));
static volatile uint32_t s_feedback_gap_by_slot[DCU_CAN_BUS_COUNT][CANFD_TX_FIFO_DEPTH]
    __attribute__((section(".noncacheable.bss")));
static volatile uint32_t s_feedback_gap_after[DCU_CAN_BUS_COUNT][CANFD_PACED_SLOTS]
    __attribute__((section(".noncacheable.bss")));
static void collect_tx_eofs(unsigned bus, MCAN_Type *base);
static tx_retirement_t s_tx_retirement[DCU_CAN_BUS_COUNT];
static uint32_t s_tx_gap_ticks[DCU_CAN_BUS_COUNT];
static uint32_t s_tsu_hz[DCU_CAN_BUS_COUNT];
static uint32_t s_raw_tx_cancelled[DCU_CAN_BUS_COUNT];
static dcu_can_tx_id_t s_tx_serial[DCU_CAN_BUS_COUNT];
/* First observed protocol error per active bus/boot, read by JTAG with the target halted. */
static volatile canfd_first_fault_t s_first_fault[DCU_CAN_BUS_COUNT];
static const uint32_t s_protocol_errors =
    MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE | MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE;
static volatile uint8_t s_led_ticks[DCU_CAN_BUS_COUNT][CANFD_LED_KIND_COUNT];
static uint16_t s_heartbeat_ticks;
static dcu_can_channel_config_t s_channels[DCU_CAN_BUS_COUNT];
static uint32_t s_active_mask;

const dcu_can_channel_config_t *dcu_can_channel_config(unsigned bus)
{
    return bus < DCU_CAN_BUS_COUNT && (s_active_mask & (1U << bus)) ? &s_channels[bus] : NULL;
}

static void led_pulse(uint32_t bus, uint32_t kind)
{
    const canfd_led_t *led = &s_led[bus][kind];

    gpio_write_pin(led->ctrl, led->index, led->pin, led->on_level);
    s_led_ticks[bus][kind] = CANFD_LED_PULSE_TICKS;
}

void dcu_mcan_led_tick(void)
{
    for (uint32_t bus = 0; bus < DCU_CAN_BUS_COUNT; bus++) {
        for (uint32_t kind = 0; kind < CANFD_LED_KIND_COUNT; kind++) {
            if (s_led_ticks[bus][kind] != 0U && --s_led_ticks[bus][kind] == 0U) {
                const canfd_led_t *led = &s_led[bus][kind];
                gpio_write_pin(led->ctrl, led->index, led->pin, led->off_level);
            }
        }
    }

    /* 保留原来每秒翻转一次的板载运行心跳灯。 */
    if (++s_heartbeat_ticks >= 1000U) {
        s_heartbeat_ticks = 0U;
        board_led_toggle();
    }
}

/* Only called from MCAN IRQ or under the kernel mask, before clearing IR or touching TX ownership. */
static void latch_protocol_fault(unsigned bus, canfd_fault_source_t source, uint32_t ir, uint32_t psr)
{
    volatile canfd_first_fault_t *dst = &s_first_fault[bus];
    if (dst->valid) return;
    MCAN_Type *base = s_mcan[bus];
    dst->observed_us = dcu_time_now_us();
    dst->source = source;
    dst->ir = ir;
    dst->psr = psr;
    dst->ecr = base->ECR;
    dst->txbrp = base->TXBRP;
    dst->txbto = base->TXBTO;
    dst->txbcf = base->TXBCF;
    dst->txefs = base->TXEFS;
    dst->rxf0s = base->RXF0S;
    dst->nbtp = base->NBTP;
    dst->dbtp = base->DBTP;
    dst->tdcr = base->TDCR;
    dst->cccr = base->CCCR;
    for (unsigned i = 0; i < CANFD_TX_SLOTS; ++i)
        dst->tx_id[i] = s_tx[bus][i].id;
    __atomic_store_n(&dst->valid, 1U, __ATOMIC_RELEASE);
}

#if defined(CONFIG_PDO_PHASE_TRACE)
static uint32_t s_retx_observed;

uint32_t dcu_can_observed_retx_count(void)
{
    return __atomic_load_n(&s_retx_observed, __ATOMIC_RELAXED);
}
#endif

static uint32_t sample_psr(unsigned bus)
{
    const uint32_t psr = s_mcan[bus]->PSR;
#if defined(CONFIG_PDO_PHASE_TRACE)
    /* PSR error codes clear on read. Count observed ACK/bit errors with an
     * owned TX on a timestamp channel, not an invented hardware attempt count. */
    const unsigned lec = MCAN_PSR_LEC_GET(psr), dlec = MCAN_PSR_DLEC_GET(psr);
    if (s_channels[bus].timestamp_eof && ((lec >= 3 && lec <= 5) || (dlec >= 3 && dlec <= 5)) &&
        dcu_can_pending(bus, DCU_CAN_DEVICE_ANY))
        __atomic_fetch_add(&s_retx_observed, 1U, __ATOMIC_RELAXED);
#endif
    return psr;
}

static void capture_pending_protocol_fault(unsigned bus, canfd_fault_source_t source)
{
    if (s_first_fault[bus].valid) return;
    MCAN_Type *base = s_mcan[bus];
    const uint32_t ir = mcan_get_interrupt_flags(base);
    if (ir & s_protocol_errors) latch_protocol_fault(bus, source, ir, sample_psr(bus));
}

static bool is_protocol_error_code(uint32_t code)
{
    return code > 0U && code < 7U;
}

/* Serialize PSR reads with the BO interrupt so an older sample cannot replace a recovery. */
static DCU_FAST_CODE uint32_t read_bus_psr_locked(unsigned bus, uint32_t before, canfd_fault_source_t source)
{
    MCAN_Type *base = s_mcan[bus];
    uint32_t psr = sample_psr(bus);
    if (!s_first_fault[bus].valid) {
        const bool lec = is_protocol_error_code(MCAN_PSR_LEC_GET(psr));
        const bool dlec = is_protocol_error_code(MCAN_PSR_DLEC_GET(psr));
        if (before & s_protocol_errors) {
            latch_protocol_fault(bus, source, before, psr);
        } else {
            const uint32_t after = mcan_get_interrupt_flags(base);
            if (after & s_protocol_errors) {
                if (!((lec && (after & MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE)) ||
                      (dlec && (after & MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE))))
                    psr = sample_psr(bus);
                latch_protocol_fault(bus, source, after, psr);
            } else if (lec || dlec) {
                /* Keep the code even if the event flag was not observed with this PSR read. */
                latch_protocol_fault(bus, source, after, psr);
            }
        }
    }
    atomic_store_explicit(&s_bus_off[bus], psr & MCAN_PSR_BO_MASK, memory_order_relaxed);
    return psr;
}

static DCU_FAST_CODE uint32_t read_bus_psr(unsigned bus)
{
    taskENTER_CRITICAL();
    const uint32_t psr = read_bus_psr_locked(bus, mcan_get_interrupt_flags(s_mcan[bus]), CANFD_FAULT_FROM_STATE_READ);
    taskEXIT_CRITICAL();
    return psr;
}

DCU_FAST_CODE bool dcu_can_is_bus_off_fast(unsigned bus)
{
    return bus < DCU_CAN_BUS_COUNT && atomic_load_explicit(&s_bus_off[bus], memory_order_relaxed) != 0;
}

uint16_t dcu_can_bus_state(uint32_t bus)
{
    if (!dcu_can_channel_config(bus)) return DCU_CAN_BUS_DISABLED;
    const uint32_t psr = read_bus_psr(bus);
    if (psr & MCAN_PSR_BO_MASK) return DCU_CAN_BUS_OFF;
    if (psr & MCAN_PSR_EP_MASK) return DCU_CAN_BUS_ERROR_PASSIVE;
    if (psr & MCAN_PSR_EW_MASK) return DCU_CAN_BUS_ERROR_WARNING;
    return DCU_CAN_BUS_ERROR_ACTIVE;
}

static DCU_FAST_CODE dcu_can_eof_t read_eof(unsigned bus, bool captured, unsigned slot)
{
    MCAN_Type *base = s_mcan[bus];
    if (!captured || !mcan_is_tsu_timestamp_available(base, slot))
        return (dcu_can_eof_t){0};
    dcu_can_eof_t eof = {.tick = mcan_read_32bit_tsu_timestamp(base, slot), .valid = true};
    const uint32_t current = base->ATB;
    const uint32_t now_us = dcu_time_now_us();
    const uint32_t hz = s_tsu_hz[bus], age = current - eof.tick;
    eof.us = now_us - (hz % 1000000U == 0 ? age / (hz / 1000000U) :
                       (uint32_t)((uint64_t)age * 1000000U / hz));
    return eof;
}

/* Bytes beyond DLC remain zero when a shorter frame reuses this SPSC slot.
 * Never cast frame.data (not word aligned) to a uint32_t pointer. */
static DCU_FAST_CODE void copy_rx_payload(dcu_can_frame_t *frame, const volatile uint32_t *src, unsigned bytes)
{
    const unsigned previous = dcu_can_dlc_to_bytes(frame->dlc, (frame->flags & DCU_CAN_FLAG_FD) != 0);
    unsigned pos = 0;
    for (; pos + sizeof(uint32_t) <= bytes; pos += sizeof(uint32_t)) {
        const uint32_t word = *src++;
        frame->data[pos] = (uint8_t)word;
        frame->data[pos + 1U] = (uint8_t)(word >> 8);
        frame->data[pos + 2U] = (uint8_t)(word >> 16);
        frame->data[pos + 3U] = (uint8_t)(word >> 24);
    }
    if (pos < bytes) {
        uint32_t word = *src;
        while (pos < bytes) {
            frame->data[pos++] = (uint8_t)word;
            word >>= 8;
        }
    }
    const unsigned used = previous <= sizeof(frame->data) ? previous : sizeof(frame->data);
    if (used > bytes) memset(frame->data + bytes, 0, used - bytes);
}

static DCU_FAST_CODE void receive_isr(uint32_t bus)
{
#if defined(CONFIG_CAN_BATCH_SAMPLE)
    const uint32_t irq_us = dcu_time_now_us();
#endif
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel) return;
    MCAN_Type *base=s_mcan[bus]; uint32_t flags=mcan_get_interrupt_flags(base);
    if ((flags & MCAN_INT_BUS_OFF_STATUS) || ((flags & s_protocol_errors) && !s_first_fault[bus].valid)) {
        const UBaseType_t saved = portSET_INTERRUPT_MASK_FROM_ISR();
        /* Ack BO first: a later transition must remain pending after this sample. */
        if (flags & MCAN_INT_BUS_OFF_STATUS) mcan_clear_interrupt_flags(base, MCAN_INT_BUS_OFF_STATUS);
        (void)read_bus_psr_locked(bus, flags, CANFD_FAULT_FROM_IRQ);
        portCLEAR_INTERRUPT_MASK_FROM_ISR(saved);
    }
#if defined(CONFIG_PDO_PHASE_TRACE)
    else if (flags & s_protocol_errors) (void)sample_psr(bus);
#endif
    mcan_clear_interrupt_flags(base, flags & ~MCAN_INT_BUS_OFF_STATUS);
    if ((flags & MCAN_INT_TX_EVT_FIFO_NEW_ENTRY) && channel->timestamp_eof) {
        const UBaseType_t mask = portSET_INTERRUPT_MASK_FROM_ISR();
        collect_tx_eofs(bus, base);
        portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    }
    while (flags & MCAN_INT_RXFIFO0_NEW_MSG) {
        const uint32_t status = base->RXF0S;
        if (!MCAN_RXF0S_F0FL_GET(status)) break;
        const unsigned index = MCAN_RXF0S_F0GI_GET(status);
        const volatile uint32_t *element = s_rx_fifo0[bus] + index * (sizeof(mcan_rx_message_t) / sizeof(uint32_t));
        union { uint32_t words[2]; mcan_rx_message_t header; } rx;
        rx.words[0] = element[0];
        rx.words[1] = element[1];
        const mcan_rx_message_t *f = &rx.header;
        dcu_can_eof_t eof = {0};
        if (channel->timestamp_eof)
            eof = read_eof(bus, f->rx_timestamp_captured, f->rx_timestamp_pointer);
        led_pulse(bus,CANFD_LED_RX);
        rx_queue_t *q = &s_rx[bus];
        const unsigned write = atomic_load_explicit(&q->write, memory_order_relaxed);
        const unsigned next = (write + 1U) % ARRAY_SIZE(q->item);
        const uint8_t bytes = dcu_can_dlc_to_bytes(f->dlc, f->canfd_frame);
        if (bytes > DCU_CAN_MAX_PAYLOAD || f->rtr || (f->canfd_frame && !channel->fd) ||
            (f->bitrate_switch && !f->canfd_frame) || next == atomic_load_explicit(&q->read, memory_order_acquire)) {
            base->RXF0A = MCAN_RXF0A_F0AI_SET(index);
            ++s_rx_lost[bus];
            led_pulse(bus, CANFD_LED_ERR);
            continue;
        }
        rx_item_t *item = &q->item[write];
        copy_rx_payload(&item->frame, element + 2, bytes);
        fenceiorw();
        base->RXF0A = MCAN_RXF0A_F0AI_SET(index);
        item->frame.id = f->use_ext_id ? f->ext_id : f->std_id;
        item->frame.dlc = f->dlc;
        item->frame.flags = (f->use_ext_id ? DCU_CAN_FLAG_EXTENDED : 0) |
            (f->canfd_frame ? DCU_CAN_FLAG_FD : 0) | (f->bitrate_switch ? DCU_CAN_FLAG_BRS : 0);
        item->stamp = (dcu_can_rx_stamp_t){.observed_ms = dcu_time_now_ms(), .eof = eof};
        item->stamp.enqueued_us = dcu_time_now_us();
#if defined(CONFIG_CAN_BATCH_SAMPLE)
        item->stamp.irq_us = irq_us;
        item->stamp.can_irq_us = can_irq_total_us + (item->stamp.enqueued_us - irq_us);
#endif
#if defined(CONFIG_PDO_PHASE_TRACE)
        item->observed_us = dcu_time_now_us();
#endif
        atomic_store_explicit(&q->write, next, memory_order_release);
    }
    if (flags & (MCAN_EVENT_TRANSMIT | MCAN_INT_TX_EVT_FIFO_NEW_ENTRY)) led_pulse(bus,CANFD_LED_TX);
    if(flags&MCAN_EVENT_ERROR)led_pulse(bus,CANFD_LED_ERR);
    /* The paced batch is submitted and drained within one worker activation.
     * Only event PDO can have an asleep worker waiting for this RX interrupt. */
    if ((flags & MCAN_INT_RXFIFO0_NEW_MSG) && dcu_cycle_event_enabled())
        dcu_notify_ecat_can_from_isr();
#if defined(CONFIG_CAN_BATCH_SAMPLE)
    can_irq_total_us += dcu_time_now_us() - irq_us;
#endif
}

static void mcan0_isr(void) { receive_isr(0U); }
static void mcan1_isr(void) { receive_isr(1U); }
static void mcan2_isr(void) { receive_isr(2U); }
static void mcan3_isr(void) { receive_isr(3U); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, mcan0_isr);
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN1, mcan1_isr);
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN2, mcan2_isr);
SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, mcan3_isr);

hpm_stat_t dcu_mcan_init(const dcu_can_channel_config_t *channels, size_t count)
{
    // Global acceptance stores RX frames but does not request a TSU capture.
    static const mcan_filter_elem_t timestamp_standard = {
        .filter_type = MCAN_FILTER_TYPE_CLASSIC_FILTER,
        .filter_config = MCAN_FILTER_ELEM_CFG_STORE_IN_RX_FIFO0_IF_MATCH,
        .can_id_type = MCAN_CAN_ID_TYPE_STANDARD,
        .sync_message = 1U,
        .filter_id = 0U,
        .filter_mask = 0U,
    };
    if (s_active_mask || !channels || !count || count > DCU_CAN_BUS_COUNT)
        return status_invalid_argument;
    uint32_t requested = 0;
    for (size_t i = 0; i < count; ++i) {
        const dcu_can_channel_config_t *channel = &channels[i];
        if (channel->bus >= DCU_CAN_BUS_COUNT || (requested & (1U << channel->bus)) ||
            !channel->nominal_bitrate_bps || channel->nominal_bitrate_bps > 1000000U ||
            (channel->fd ? (channel->data_bitrate_bps < channel->nominal_bitrate_bps ||
                            channel->data_bitrate_bps > 8000000U) : channel->data_bitrate_bps != 0))
            return status_invalid_argument;
        requested |= 1U << channel->bus;
    }
    /*
     * PinMux 生成代码会在切换为 GPIO 时短暂写低，而这些 LED 为低电平点亮。
     * 在启用 MCAN 中断前再次显式写 OFF，保证上电且无报文时 12 个灯全部熄灭。
     */
    for (uint32_t bus = 0; bus < DCU_CAN_BUS_COUNT; bus++) {
        for (uint32_t kind = 0; kind < CANFD_LED_KIND_COUNT; kind++) {
            const canfd_led_t *led = &s_led[bus][kind];
            s_led_ticks[bus][kind] = 0U;
            gpio_write_pin(led->ctrl, led->index, led->pin, led->off_level);
        }
    }

    for (size_t i = 0; i < count; ++i) {
        const dcu_can_channel_config_t *channel = &channels[i];
        const unsigned bus = channel->bus;
        mcan_config_t config;
        uint32_t clock_hz;
        hpm_stat_t status;

        board_init_can(s_mcan[bus]);
        clock_hz = board_init_can_clock(s_mcan[bus]);
#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && MCAN_SOC_MSG_BUF_IN_AHB_RAM
        mcan_msg_buf_attr_t attr = {
            .ram_base = (uint32_t)&s_mcan_ram[bus][0],
            .ram_size = sizeof(s_mcan_ram[bus]),
        };
        status = mcan_set_msg_buf_attr(s_mcan[bus], &attr);
        if (status != status_success) {
            return status;
        }
#endif
        mcan_get_default_config(s_mcan[bus], &config);
        config.baudrate = channel->nominal_bitrate_bps;
        config.baudrate_fd = channel->data_bitrate_bps;
        config.enable_canfd = channel->fd;
        config.enable_tdc = channel->fd;
        config.disable_auto_retransmission = false;
        if (channel->timestamp_eof) {
            config.use_timestamping_unit = true;
            config.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;
            config.tsu_config.enable_tsu = true;
            config.tsu_config.capture_on_sof = false;
            config.tsu_config.use_ext_timebase = false;
            config.tsu_config.enable_64bit_timestamp = false;
        }
        mcan_get_default_ram_config(s_mcan[bus], &config.ram_config, channel->fd);

        /*
         * RX/TX FIFO 内部槽支持 64 字节；透传可用长度由所选 PDO 档位决定。
         * 通过标准帧过滤项请求 RX EOF，其余帧保留原有接收策略。
         */
        config.ram_config.enable_std_filter = channel->timestamp_eof;
        config.ram_config.std_filter_elem_count = channel->timestamp_eof ? 1U : 0U;
        config.ram_config.enable_ext_filter = false;
        config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = channel->timestamp_eof ? 1U : 0U;
        config.all_filters_config.std_id_filter_list.filter_elem_list =
            channel->timestamp_eof ? &timestamp_standard : NULL;
        config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0U;
        config.ram_config.rxfifos[0].enable = 1U;
        config.ram_config.rxfifos[0].elem_count = CANFD_RX_FIFO_DEPTH;
        config.ram_config.rxfifos[0].data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        config.ram_config.rxfifos[1].enable = 0U;
        config.ram_config.enable_rxbuf = false;
        config.ram_config.txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        config.ram_config.txbuf_dedicated_txbuf_elem_count = channel->timestamp_eof ? CANFD_PACED_SLOTS : 0U;
        config.ram_config.txbuf_fifo_or_queue_elem_count = CANFD_TX_FIFO_DEPTH;
        config.ram_config.txfifo_or_txqueue_mode = 0U; /* 0=FIFO，保持槽位发送顺序。 */
        config.ram_config.enable_tx_evt_fifo = channel->timestamp_eof;
        config.ram_config.tx_evt_fifo_elem_count = channel->timestamp_eof ? CANFD_TX_FIFO_DEPTH : 0U;
        config.ram_config.tx_evt_fifo_watermark = 0U;
        /* Dedicated paced buffers already raise the TX event FIFO interrupt
         * with the hardware EOF. Ordinary FIFO buffers keep their TC interrupt. */
        config.txbuf_trans_interrupt_mask = ((1UL << CANFD_TX_FIFO_DEPTH) - 1UL) <<
                                            (channel->timestamp_eof ? CANFD_PACED_SLOTS : 0U);
        config.interrupt_mask = MCAN_INT_RXFIFO0_NEW_MSG
                              | (channel->timestamp_eof ? MCAN_INT_TX_EVT_FIFO_NEW_ENTRY : 0U)
                              | MCAN_EVENT_TRANSMIT
                              | MCAN_EVENT_ERROR;
        status = mcan_init(s_mcan[bus], &config, clock_hz);
        if (status != status_success) {
            return status;
        }
        if ((s_mcan[bus]->CCCR & MCAN_CCCR_DAR_MASK) ||
            (channel->timestamp_eof &&
             (!mcan_is_tsu_used(s_mcan[bus]) ||
              (s_mcan[bus]->TSCFG & (MCAN_TSCFG_TSUE_MASK | MCAN_TSCFG_SCP_MASK |
                                    MCAN_TSCFG_TBCS_MASK | MCAN_TSCFG_EN64_MASK)) != MCAN_TSCFG_TSUE_MASK)))
            return status_fail;
        mcan_interrupt_line_select(s_mcan[bus], config.interrupt_mask, 0U);
        if (MCAN_RXESC_F0DS_GET(s_mcan[bus]->RXESC) != MCAN_DATA_FIELD_SIZE_64BYTES ||
            MCAN_RXF0C_F0OM_GET(s_mcan[bus]->RXF0C)) return status_fail;
        s_rx_fifo0[bus] = (const volatile uint32_t *)mcan_get_rxfifo0_base(s_mcan[bus]);
        if (channel->timestamp_eof) {
            const uint32_t event_offset = s_mcan[bus]->TXEFC & MCAN_TXEFC_EFSA_MASK;
            s_tx_events[bus] = (const volatile uint32_t *)(mcan_get_ram_base(s_mcan[bus]) + event_offset);
            /* HPM5E31 ATB uses the AHB timebase, not the CAN bit-timing clock. */
            const uint32_t tsu_clock_hz = clock_get_frequency(clock_ahb0);
            if (!tsu_clock_hz) return status_fail;
            const uint64_t scale = (1U + MCAN_TSCFG_TBPRE_GET(s_mcan[bus]->TSCFG)) * UINT64_C(1000000);
            s_tx_gap_ticks[bus] = (uint32_t)(((uint64_t)tsu_clock_hz * DCU_CAN_TX_EOF_GAP_US + scale - 1U) / scale);
            const uint32_t divisor = 1U + MCAN_TSCFG_TBPRE_GET(s_mcan[bus]->TSCFG);
            s_tsu_hz[bus] = (tsu_clock_hz + divisor - 1U) / divisor;
            s_feedback_stats[bus].hz = s_tsu_hz[bus];
        }
        s_channels[bus] = *channel;
        /* No MCAN IRQ owns this bus yet; seed the snapshot before enabling the line. */
        (void)read_bus_psr_locked(bus, mcan_get_interrupt_flags(s_mcan[bus]), CANFD_FAULT_FROM_STATE_READ);
    }
    s_active_mask = requested;
    for (size_t i = 0; i < count; ++i) {
        const unsigned bus = channels[i].bus;
        board_can_transceiver_phy_set(s_mcan[bus], true);
        intc_m_enable_irq_with_priority(s_irq[bus], DCU_IRQ_CAN_PRIORITY);
    }
    return status_success;
}


uint32_t dcu_can_rx_drops(unsigned bus) { return dcu_can_channel_config(bus) ? s_rx_lost[bus] : 0; }
DCU_FAST_CODE unsigned dcu_can_oldest_rx_head(uint32_t buses, uint32_t before_us, uint32_t *wire_id)
{
    unsigned oldest = DCU_CAN_BUS_COUNT;
    uint32_t oldest_us = before_us;
    // A head published during this scan belongs to the next snapshot. Ties keep the lower bus.
    for (unsigned mask = buses & s_active_mask; mask; mask &= mask - 1U) {
        const unsigned bus = __builtin_ctz(mask);
        const rx_queue_t *q = &s_rx[bus];
        const unsigned read = atomic_load_explicit(&q->read, memory_order_relaxed);
        if (read == atomic_load_explicit(&q->write, memory_order_acquire)) continue;
        const rx_item_t *head = &q->item[read];
        if ((int32_t)(head->stamp.enqueued_us - oldest_us) < 0) {
            oldest = bus;
            oldest_us = head->stamp.enqueued_us;
            *wire_id = head->frame.id;
        }
    }
    return oldest;
}

DCU_FAST_CODE bool dcu_can_receive(unsigned bus, dcu_can_frame_t *f, dcu_can_rx_stamp_t *stamp)
{
    if (!dcu_can_channel_config(bus) || !f || !stamp) return false;
    rx_queue_t *q = &s_rx[bus];
    const unsigned read = atomic_load_explicit(&q->read, memory_order_relaxed);
    if (read == atomic_load_explicit(&q->write, memory_order_acquire)) return false;
#if defined(CONFIG_CAN_BATCH_SAMPLE)
    const uint32_t begin_us = dcu_time_now_us();
#endif
    const rx_item_t *item = &q->item[read];
    if ((int32_t)(s_chain[bus].cutoff_us - dcu_time_now_us()) > 0) {
        for (unsigned i = 0; i < s_chain[bus].count; ++i) {
            const tx_slot_t *slot = &s_tx[bus][i];
            if (slot->feedback && slot->wire_id == item->frame.id &&
                __atomic_load_n(&slot->launched, __ATOMIC_RELAXED))
                return false; /* Deliver this request's terminal event before its reply. */
        }
    }
    *f = item->frame;
    *stamp = item->stamp;
    atomic_store_explicit(&q->read, (read + 1U) % ARRAY_SIZE(q->item), memory_order_release);
#if defined(CONFIG_CAN_BATCH_SAMPLE)
    stamp->dequeue_us = begin_us;
    stamp->dequeued_us = dcu_time_now_us();
    stamp->can_irq_us = can_irq_total_us - stamp->can_irq_us;
#endif
    return true;
}
bool dcu_can_pending(unsigned bus, unsigned device)
{
    if (!dcu_can_channel_config(bus)) return false;
    for (unsigned i = 0; i < CANFD_TX_SLOTS; ++i)
        if (s_tx[bus][i].id && (device == DCU_CAN_DEVICE_ANY || s_tx[bus][i].in_flight_device == device ||
                                (device != DCU_CAN_DEVICE_RAW && s_tx[bus][i].in_flight_device == DCU_CAN_DEVICE_ANY) ||
                                s_tx[bus][i].in_flight_device == DCU_CAN_DEVICE_RAW)) return true;
    return false;
}
unsigned dcu_can_tx_free(unsigned bus)
{
    return dcu_can_channel_config(bus) ? MCAN_TXFQS_TFFL_GET(s_mcan[bus]->TXFQS) : 0;
}
DCU_FAST_CODE bool dcu_can_tx_serial_ready(unsigned bus)
{
    if (!dcu_can_tx_free(bus) || dcu_can_pending(bus, DCU_CAN_DEVICE_ANY)) return false;
    const tx_retirement_t *last = &s_tx_retirement[bus];
    if (!last->seen || (uint32_t)(dcu_time_now_us() - last->observed_us) > DCU_CAN_TX_EOF_GAP_US) return true;
    // A strictly greater elapsed count is conservative at the timestamp quantum.
    return last->eof.valid && (int32_t)(s_mcan[bus]->ATB - last->eof.tick) > (int32_t)s_tx_gap_ticks[bus];
}
uint32_t dcu_can_tx_progress(unsigned bus)
{
    return dcu_can_channel_config(bus) ? s_tx_progress[bus] : 0;
}

static void cancel_tail(unsigned bus)
{
    paced_tx_t *b = &s_chain[bus];
    b->armed = false;
    while (b->next < b->count) s_tx[bus][b->next++].revoked = true;
}

/* Task launches slot zero; only the EOF/timer IRQs can launch its successors.
 * All payloads already reside in dedicated message RAM, outside the ordinary FIFO. */
static DCU_FAST_CODE bool launch_paced(unsigned bus)
{
    paced_tx_t *b = &s_chain[bus];
    MCAN_Type *base = s_mcan[bus];
    const uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    const unsigned i = b->next;
    if (i >= b->count) {
        restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
        return false;
    }
    tx_slot_t *slot = &s_tx[bus][i];
    const bool ok = !slot->revoked && !dcu_tx_inhibited() && !(base->CCCR & MCAN_CCCR_INIT_MASK) &&
        dcu_cyclic_tx_window_current(b->cutoff_us) && (int32_t)(b->cutoff_us - dcu_time_now_us()) > 0;
    if (ok) {
        __atomic_store_n(&slot->launched, true, __ATOMIC_RELAXED);
        ++b->next;
        const uint32_t before = base->ATB;
        fenceiorw();
        mcan_send_add_request(base, i);
        fenceiorw();
        const uint32_t after = base->ATB;
        if (i) {
            const uint32_t gap = before - b->eof_tick;
            s_feedback_gap_by_slot[bus][slot->in_flight_device] = gap;
            s_feedback_gap_after[bus][slot->in_flight_device] = after - b->eof_tick;
            if (!s_feedback_stats[bus].samples || gap < s_feedback_stats[bus].min_gap_ticks)
                s_feedback_stats[bus].min_gap_ticks = gap;
            ++s_feedback_stats[bus].samples;
        }
    }
    restore_global_irq(irq & CSR_MSTATUS_MIE_MASK);
    if (!ok) cancel_tail(bus);
    return ok;
}

static DCU_FAST_CODE void service_gap(unsigned bus)
{
    paced_tx_t *b = &s_chain[bus];
    if (!b->armed) return;
    const uint32_t elapsed = s_mcan[bus]->ATB - b->eof_tick;
    if (elapsed < s_tx_gap_ticks[bus]) {
        const uint32_t remaining = s_tx_gap_ticks[bus] - elapsed;
        const uint32_t hz = s_tsu_hz[bus], per_us = hz / 1000000U;
        const uint32_t wait = hz % 1000000U == 0 ? (remaining + per_us - 1U) / per_us :
            (uint32_t)(((uint64_t)remaining * 1000000U + hz - 1U) / hz);
        dcu_can_arm_gap_at_us(dcu_time_now_us() + wait);
        return;
    }
    b->armed = false;
    (void)launch_paced(bus);
}

DCU_FAST_CODE void dcu_can_gap_elapsed_isr(void)
{
    for (unsigned bus = 0; bus < DCU_CAN_BUS_COUNT; ++bus) service_gap(bus);
}

static DCU_FAST_CODE void collect_tx_eofs(unsigned bus, MCAN_Type *base)
{
    for (unsigned n = 0; n < CANFD_TX_FIFO_DEPTH; ++n) {
        const uint32_t status = base->TXEFS;
        if (!MCAN_TXEFS_EFFL_GET(status)) break;
        const unsigned index = MCAN_TXEFS_EFGI_GET(status);
        const volatile uint32_t *element = s_tx_events[bus] + 2U * index;
        mcan_tx_event_fifo_elem_t event;
        event.words[0] = element[0];
        event.words[1] = element[1];
        fenceiorw();
        base->TXEFA = MCAN_TXEFA_EFAI_SET(index);
        const dcu_can_eof_t eof = read_eof(bus, event.tx_timestamp_captured, event.tx_timestamp_pointer);
        // ET=2 can report transmission despite cancellation; slot retirement also checks revocation.
        if (!eof.valid || (event.event_type != 1 && event.event_type != 2)) continue;
        for (unsigned i = 0; i < CANFD_TX_SLOTS; ++i) {
            tx_slot_t *slot = &s_tx[bus][i];
            if (slot->launched && (uint8_t)slot->id == event.message_marker) {
                slot->eof = eof;
                paced_tx_t *b = &s_chain[bus];
                if (slot->feedback && b->next == i + 1U && b->next < b->count) {
                    if (slot->revoked || event.event_type != 1) cancel_tail(bus);
                    else {
                        b->eof_tick = eof.tick;
                        b->armed = true;
                        service_gap(bus);
                    }
                }
                break;
            }
        }
    }
}

DCU_FAST_CODE bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event)
{
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel || !event) return false;
    const unsigned batch = s_chain[bus].unretired;
    if (!batch && !dcu_can_pending(bus, DCU_CAN_DEVICE_ANY)) return false;
    MCAN_Type *base = s_mcan[bus];
    bool found = false;
    taskENTER_CRITICAL();
    capture_pending_protocol_fault(bus, CANFD_FAULT_BEFORE_TX_EVENT);
    const uint32_t pending = base->TXBRP;
    const uint32_t completed = base->TXBTO;
    const uint32_t cancelled = base->TXBCF;
    const uint32_t terminal = (completed | cancelled) & ~pending;
    if (channel->timestamp_eof && !s_chain[bus].count) collect_tx_eofs(bus, base);
    for (unsigned mask = batch ? batch : (1U << CANFD_TX_SLOTS) - 1U; mask; mask &= mask - 1U) {
        const unsigned i = __builtin_ctz(mask);
        tx_slot_t *slot = &s_tx[bus][i];
        if (!slot->id || (slot->launched ? !(terminal & (1U << i)) : !slot->revoked)) continue;
        const bool sent = slot->launched && !slot->revoked && (completed & (1U << i));
        if (slot->feedback) {
            if (sent && !slot->eof.valid && !dcu_tx_inhibited() &&
                (int32_t)(s_chain[bus].cutoff_us - dcu_time_now_us()) > 0)
                continue;
            if ((!sent || !slot->eof.valid) && s_chain[bus].next == i + 1U) cancel_tail(bus);
            if (sent && slot->in_flight_device < CANFD_TX_FIFO_DEPTH) {
                ++s_feedback_stats[bus].completed[slot->in_flight_device];
                if (!slot->eof.valid) ++s_feedback_stats[bus].completed_without_eof;
            }
        }
        *event = (dcu_can_tx_event_t){.id = slot->id, .status = sent ? DCU_CAN_TX_COMPLETED : DCU_CAN_TX_CANCELLED,
                                     .batch_owner_plus_one = slot->feedback ? slot->in_flight_device + 1U : 0U};
        if (sent) event->eof = slot->eof;
        // Raw slots may retire out of order; observing the drain bounds all of them.
        if (slot->launched) s_tx_retirement[bus] = (tx_retirement_t){
            .eof = slot->in_flight_device != DCU_CAN_DEVICE_RAW ? event->eof : (dcu_can_eof_t){0},
            .observed_us = dcu_time_now_us(), .seen = true,
        };
        if (sent) ++s_tx_progress[bus];
        else if (slot->in_flight_device == DCU_CAN_DEVICE_RAW) ++s_raw_tx_cancelled[bus];
        *slot = (tx_slot_t){0};
        if (batch) {
            s_chain[bus].unretired &= ~(1U << i);
            if (!s_chain[bus].unretired) s_chain[bus].count = 0;
        }
        found = true;
        break;
    }
    taskEXIT_CRITICAL();
    return found;
}

uint32_t dcu_can_raw_tx_cancelled(unsigned bus)
{ return bus < DCU_CAN_BUS_COUNT ? s_raw_tx_cancelled[bus] : 0; }

/* id == 0 revokes every pending request on this bus. Caller masks CAN and
 * watchdog IRQs with the FreeRTOS threshold, leaving Sync0/PDI unmasked. */
static bool cancel_locked(unsigned bus, dcu_can_tx_id_t id)
{
    capture_pending_protocol_fault(bus, CANFD_FAULT_BEFORE_CANCEL);
    const uint32_t pending = s_mcan[bus]->TXBRP;
    uint32_t requests = 0;
    bool stopped = false;
    for (unsigned i = 0; i < CANFD_TX_SLOTS; ++i) {
        tx_slot_t *slot = &s_tx[bus][i];
        if (!slot->id || (id && slot->id != id) || slot->revoked) continue;
        if (slot->feedback && s_chain[bus].next < s_chain[bus].count) {
            cancel_tail(bus);
            stopped = true;
        }
        if (slot->launched && (pending & (1U << i))) {
            slot->revoked = true;
            requests |= 1U << i;
        }
    }
    if (requests) s_mcan[bus]->TXBCR = requests;
    return requests != 0 || stopped;
}

void dcu_can_cancel(unsigned bus)
{
    if (!dcu_can_channel_config(bus)) return;
    taskENTER_CRITICAL();
    (void)cancel_locked(bus, 0);
    taskEXIT_CRITICAL();
}

void dcu_can_cancel_from_isr(unsigned bus)
{
    if (!dcu_can_channel_config(bus)) return;
    const UBaseType_t saved = portSET_INTERRUPT_MASK_FROM_ISR();
    (void)cancel_locked(bus, 0);
    portCLEAR_INTERRUPT_MASK_FROM_ISR(saved);
}

bool dcu_can_cancel_tx(unsigned bus, dcu_can_tx_id_t id)
{
    if (!id || !dcu_can_channel_config(bus)) return false;
    taskENTER_CRITICAL();
    const bool requested = cancel_locked(bus, id);
    taskEXIT_CRITICAL();
    return requested;
}

bool dcu_can_drain_raw(unsigned bus)
{
    if (!dcu_can_channel_config(bus)) return true;
    bool empty = true;
    for (unsigned i = 0; i < CANFD_TX_SLOTS; ++i) {
        const tx_slot_t *slot = &s_tx[bus][i];
        if (slot->id && slot->in_flight_device == DCU_CAN_DEVICE_RAW) {
            empty = false;
            if (!slot->revoked) dcu_can_cancel_tx(bus, slot->id);
        }
    }
    return empty;
}

static mcan_tx_frame_t format_tx(const dcu_can_frame_t *f, bool timestamp, uint8_t marker)
{
    mcan_tx_frame_t tx={0};
    tx.use_ext_id = (f->flags & DCU_CAN_FLAG_EXTENDED) != 0;
    if (tx.use_ext_id) tx.ext_id = f->id; else tx.std_id = f->id;
    tx.dlc = f->dlc;
    tx.canfd_frame = (f->flags & DCU_CAN_FLAG_FD) != 0;
    tx.bitrate_switch = (f->flags & DCU_CAN_FLAG_BRS) != 0;
    if (timestamp) {
        tx.timestamp_capture_enable = true;
        tx.event_fifo_control = true;
        // At most one FIFO's worth of unconsumed tags can coexist per bus.
        tx.message_marker_l = marker;
    }
    memcpy(tx.data_8, f->data, dcu_can_dlc_to_bytes(f->dlc, tx.canfd_frame));
    return tx;
}

DCU_FAST_CODE dcu_can_tx_id_t dcu_can_submit(unsigned bus, unsigned device, const dcu_can_frame_t *f)
{
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel || !f || (!channel->fd && (f->flags & DCU_CAN_FLAG_FD)) || s_chain[bus].count ||
        dcu_tx_inhibited() || s_tx_serial[bus] == UINT64_MAX || !dcu_can_frame_valid(f) ||
        dcu_can_bus_state(bus) == DCU_CAN_BUS_OFF) return 0;
    mcan_tx_frame_t tx = format_tx(f, channel->timestamp_eof, (uint8_t)(s_tx_serial[bus] + 1U));
    uint32_t index;
    intc_m_disable_irq(s_irq[bus]);
    dcu_can_tx_id_t id = 0;
    if (mcan_request_and_fill_txfifo(s_mcan[bus], &tx, &index) == status_success && index < CANFD_TX_SLOTS) {
        /* Stage outside the kernel critical section. Only the final gate, tag
         * and TX request are indivisible with respect to the fault timer.
         * Sync0/PDI remain enabled above the kernel priority threshold. */
        taskENTER_CRITICAL();
        /* Do not reuse a slot until its old result has been consumed. TXBAR
         * clears that slot's TXBTO and TXBCF before starting this request. */
        if (!dcu_tx_inhibited() && !s_tx[bus][index].id) {
            id = ++s_tx_serial[bus];
            s_tx[bus][index] = (tx_slot_t){.id = id, .in_flight_device = device, .launched = true};
            mcan_send_add_request(s_mcan[bus], index);
        }
        taskEXIT_CRITICAL();
    }
    intc_m_enable_irq(s_irq[bus]);
    if (!id) {
        led_pulse(bus,CANFD_LED_ERR);
    }
    return id;
}

static DCU_FAST_CODE bool stage_paced_batch(unsigned bus, const dcu_can_batch_item_t *items,
                                            unsigned count, uint32_t cutoff_us)
{
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel || !channel->timestamp_eof || !items || !count || count > CANFD_PACED_SLOTS ||
        s_tx_serial[bus] > UINT64_MAX - count || !dcu_can_tx_serial_ready(bus) || dcu_tx_inhibited() ||
        dcu_can_bus_state(bus) == DCU_CAN_BUS_OFF) return false;
    const dcu_can_tx_id_t first = s_tx_serial[bus] + 1U;
    unsigned owners = 0;
    for (unsigned i = 0; i < count; ++i) {
        const dcu_can_frame_t *f = items[i].frame;
        const unsigned owner = items[i].owner;
        if (!f || owner >= CANFD_PACED_SLOTS || (owners & (1U << owner)) || !dcu_can_frame_valid(f) ||
            (!channel->fd && (f->flags & DCU_CAN_FLAG_FD))) return false;
        owners |= 1U << owner;
        mcan_tx_frame_t tx = format_tx(f, true, (uint8_t)(first + i));
        if (mcan_write_txbuf(s_mcan[bus], i, &tx) != status_success) return false;
    }
    taskENTER_CRITICAL();
    s_chain[bus] = (paced_tx_t){.cutoff_us = cutoff_us, .count = count, .unretired = (1U << count) - 1U};
    for (unsigned i = 0; i < count; ++i)
        s_tx[bus][i] = (tx_slot_t){.id = first + i, .in_flight_device = items[i].owner,
                                   .feedback = true, .wire_id = items[i].frame->id};
    taskEXIT_CRITICAL();
    return true;
}

DCU_FAST_CODE void dcu_can_submit_batches(dcu_can_bus_batch_t batches[DCU_CAN_BUS_COUNT], uint32_t cutoff_us)
{
    unsigned staged = 0;
    for (unsigned bus = 0; bus < DCU_CAN_BUS_COUNT; ++bus) {
        dcu_can_bus_batch_t *b = &batches[bus];
        b->first_id = 0;
        b->submit_us = 0;
        if (b->count && stage_paced_batch(bus, b->items, b->count, cutoff_us))
            staged |= 1U << bus;
    }
    if (!staged) return;

    /* Only first requests share this short IRQ window. Each bus paces its own tail. */
    taskENTER_CRITICAL();
    for (; staged; staged &= staged - 1U) {
        const unsigned bus = __builtin_ctz(staged);
        dcu_can_bus_batch_t *b = &batches[bus];
#if defined(CONFIG_CAN_BATCH_SAMPLE)
        b->submit_us = dcu_time_now_us();
#endif
        if (launch_paced(bus)) {
            b->first_id = s_tx_serial[bus] + 1U;
            s_tx_serial[bus] += b->count;
        } else {
            memset(s_tx[bus], 0, b->count * sizeof(tx_slot_t));
            s_chain[bus] = (paced_tx_t){0};
        }
    }
    taskEXIT_CRITICAL();
}

DCU_FAST_CODE dcu_can_tx_id_t dcu_can_submit_batch(unsigned bus, const dcu_can_batch_item_t *items,
                                                   unsigned count, uint32_t cutoff_us)
{
    if (bus >= DCU_CAN_BUS_COUNT) return 0;
    dcu_can_bus_batch_t batches[DCU_CAN_BUS_COUNT] = {0};
    batches[bus].items = items;
    batches[bus].count = count;
    dcu_can_submit_batches(batches, cutoff_us);
    return batches[bus].first_id;
}
