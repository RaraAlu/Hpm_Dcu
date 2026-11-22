#include "hpm_mcan_transport.h"
#include "can_transport.h"
#include "cfg/can_bus_config.h"
#include "detail/can_gateway_state.h"
#include "realtime_port.h"
#include "cycle_config.h"
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
_Static_assert((CANFD_RX_FIFO_DEPTH + CANFD_TX_FIFO_DEPTH) *
                   (MCAN_MESSAGE_HEADER_SIZE_IN_BYTES + DCU_CAN_MAX_PAYLOAD) +
                   CANFD_TX_FIFO_DEPTH * sizeof(mcan_tx_event_fifo_elem_t) + sizeof(uint32_t) <=
                   MCAN_MSG_BUF_SIZE_IN_WORDS * sizeof(uint32_t),
               "MCAN FIFOs exceed Message RAM");
ATTR_PLACE_AT(".ahb_sram") static uint32_t s_mcan_ram[DCU_CAN_BUS_COUNT][MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

/* One ISR producer and one worker consumer per bus. */
static rx_queue_t s_rx[DCU_CAN_BUS_COUNT];
static volatile uint32_t s_rx_lost[DCU_CAN_BUS_COUNT];
static uint32_t s_tx_progress[DCU_CAN_BUS_COUNT];
static tx_slot_t s_tx[DCU_CAN_BUS_COUNT][CANFD_TX_FIFO_DEPTH];
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
    for (unsigned i = 0; i < CANFD_TX_FIFO_DEPTH; ++i)
        dst->tx_id[i] = s_tx[bus][i].id;
    __atomic_store_n(&dst->valid, 1U, __ATOMIC_RELEASE);
}

static void capture_pending_protocol_fault(unsigned bus, canfd_fault_source_t source)
{
    if (s_first_fault[bus].valid) return;
    MCAN_Type *base = s_mcan[bus];
    const uint32_t ir = mcan_get_interrupt_flags(base);
    if (ir & s_protocol_errors) latch_protocol_fault(bus, source, ir, base->PSR);
}

static bool is_protocol_error_code(uint32_t code)
{
    return code > 0U && code < 7U;
}

static uint32_t read_bus_psr(unsigned bus)
{
    MCAN_Type *base = s_mcan[bus];
    if (s_first_fault[bus].valid) return base->PSR;

    /* PSR consumes the error code. Keep any sample taken before the IRQ can run. */
    taskENTER_CRITICAL();
    const uint32_t before = mcan_get_interrupt_flags(base);
    uint32_t psr = base->PSR;
    if (!s_first_fault[bus].valid) {
        const bool lec = is_protocol_error_code(MCAN_PSR_LEC_GET(psr));
        const bool dlec = is_protocol_error_code(MCAN_PSR_DLEC_GET(psr));
        if (before & s_protocol_errors) {
            latch_protocol_fault(bus, CANFD_FAULT_FROM_STATE_READ, before, psr);
        } else {
            const uint32_t after = mcan_get_interrupt_flags(base);
            if (after & s_protocol_errors) {
                if (!((lec && (after & MCAN_INT_PROTOCOL_ERR_IN_ARB_PHASE)) ||
                      (dlec && (after & MCAN_INT_PROTOCOL_ERR_IN_DATA_PHASE))))
                    psr = base->PSR;
                latch_protocol_fault(bus, CANFD_FAULT_FROM_STATE_READ, after, psr);
            } else if (lec || dlec) {
                /* Keep the code even if the event flag was not observed with this PSR read. */
                latch_protocol_fault(bus, CANFD_FAULT_FROM_STATE_READ, after, psr);
            }
        }
    }
    taskEXIT_CRITICAL();
    return psr;
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

static dcu_can_eof_t read_eof(MCAN_Type *base, bool captured, unsigned slot)
{
    if (!captured || !mcan_is_tsu_timestamp_available(base, slot))
        return (dcu_can_eof_t){0};
    return (dcu_can_eof_t){mcan_read_32bit_tsu_timestamp(base, slot), true};
}

static void receive_isr(uint32_t bus)
{
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel) return;
    MCAN_Type *base=s_mcan[bus]; uint32_t flags=mcan_get_interrupt_flags(base);
    if ((flags & s_protocol_errors) && !s_first_fault[bus].valid) {
        const UBaseType_t saved = portSET_INTERRUPT_MASK_FROM_ISR();
        if (!s_first_fault[bus].valid)
            latch_protocol_fault(bus, CANFD_FAULT_FROM_IRQ, flags, base->PSR);
        portCLEAR_INTERRUPT_MASK_FROM_ISR(saved);
    }
    mcan_clear_interrupt_flags(base,flags);
    while(mcan_get_rxfifo_fill_level(base,0U)) {
        mcan_rx_message_t f;
        if(mcan_read_rxfifo(base,0,&f)!=status_success)break;
        dcu_can_eof_t eof = {0};
        if (channel->single_shot)
            eof = read_eof(base, f.rx_timestamp_captured, f.rx_timestamp_pointer);
        led_pulse(bus,CANFD_LED_RX);
        rx_queue_t *q=&s_rx[bus]; uint8_t next=(q->write+1)&31;
        const uint8_t bytes = dcu_can_dlc_to_bytes(f.dlc, f.canfd_frame);
        if (bytes > DCU_CAN_MAX_PAYLOAD || f.rtr || (f.canfd_frame && !channel->fd) ||
            (f.bitrate_switch && !f.canfd_frame) || next == q->read) {
            ++s_rx_lost[bus];
            led_pulse(bus, CANFD_LED_ERR);
            continue;
        }
        rx_item_t *item=&q->item[q->write];
        item->frame=(dcu_can_frame_t){.id=f.use_ext_id?f.ext_id:f.std_id,.dlc=f.dlc,
            .flags=(f.use_ext_id ? DCU_CAN_FLAG_EXTENDED : 0) |
                   (f.canfd_frame ? DCU_CAN_FLAG_FD : 0) | (f.bitrate_switch ? DCU_CAN_FLAG_BRS : 0)};
        memcpy(item->frame.data, f.data_8, bytes);
        item->stamp = (dcu_can_rx_stamp_t){dcu_time_now_ms(), eof};
#if defined(CONFIG_PDO_PHASE_TRACE)
        item->observed_us = dcu_time_now_us();
#endif
        __asm__ volatile("" ::: "memory");q->write=next;
    }
    if (flags & MCAN_EVENT_TRANSMIT) led_pulse(bus, CANFD_LED_TX);
    if(flags&MCAN_EVENT_ERROR)led_pulse(bus,CANFD_LED_ERR);
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
        config.disable_auto_retransmission = channel->single_shot;
#if defined(CONFIG_CAN3_TDCO_AB_TEST)
        /* Match the wheeled chassis TDCO trial without changing the TDCF baseline. */
        if (bus == 3U && channel->fd && channel->single_shot && clock_hz == 80000000U &&
            config.baudrate == 1000000U && config.baudrate_fd == 5000000U) {
            config.tdc_config.ssp_offset = 13U;
            config.tdc_config.filter_window_length = 12U;
        }
#endif
        if (channel->single_shot) {
            config.use_timestamping_unit = true;
            config.timestamp_cfg.timestamp_selection = MCAN_TIMESTAMP_SEL_EXT_TS_VAL_USED;
            config.tsu_config.enable_tsu = true;
            config.tsu_config.capture_on_sof = false;
            config.tsu_config.use_ext_timebase = false;
            config.tsu_config.enable_64bit_timestamp = false;
        }
        mcan_get_default_ram_config(s_mcan[bus], &config.ram_config, channel->fd);

        /*
         * RX/TX FIFO 内部槽支持 64 字节，独立于 Raw PDO 的 8 字节线上布局。
         * 单发模式通过标准帧过滤项请求 RX EOF，其余帧保留原有接收策略。
         */
        config.ram_config.enable_std_filter = channel->single_shot;
        config.ram_config.std_filter_elem_count = channel->single_shot ? 1U : 0U;
        config.ram_config.enable_ext_filter = false;
        config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = channel->single_shot ? 1U : 0U;
        config.all_filters_config.std_id_filter_list.filter_elem_list =
            channel->single_shot ? &timestamp_standard : NULL;
        config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0U;
        config.ram_config.rxfifos[0].enable = 1U;
        config.ram_config.rxfifos[0].elem_count = CANFD_RX_FIFO_DEPTH;
        config.ram_config.rxfifos[0].data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        config.ram_config.rxfifos[1].enable = 0U;
        config.ram_config.enable_rxbuf = false;
        config.ram_config.txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_64BYTES;
        config.ram_config.txbuf_dedicated_txbuf_elem_count = 0U;
        config.ram_config.txbuf_fifo_or_queue_elem_count = CANFD_TX_FIFO_DEPTH;
        config.ram_config.txfifo_or_txqueue_mode = 0U; /* 0=FIFO，保持槽位发送顺序。 */
        config.ram_config.enable_tx_evt_fifo = channel->single_shot;
        config.ram_config.tx_evt_fifo_elem_count = channel->single_shot ? CANFD_TX_FIFO_DEPTH : 0U;
        config.ram_config.tx_evt_fifo_watermark = 0U;
        /*
         * TX 完成中断有两级开关：除全局 TC 外，还必须在 TXBTIE 中使能
         * 对应 TX FIFO 元素；否则报文能正常上总线，但不会产生 TX 灯事件。
         */
        config.txbuf_trans_interrupt_mask = (1UL << CANFD_TX_FIFO_DEPTH) - 1UL;
        config.interrupt_mask = MCAN_INT_RXFIFO0_NEW_MSG
                              | MCAN_EVENT_TRANSMIT
                              | MCAN_EVENT_ERROR;
        status = mcan_init(s_mcan[bus], &config, clock_hz);
        if (status != status_success) {
            return status;
        }
        if (channel->single_shot &&
            (!mcan_is_tsu_used(s_mcan[bus]) ||
             !(s_mcan[bus]->CCCR & MCAN_CCCR_DAR_MASK) ||
             (s_mcan[bus]->TSCFG & (MCAN_TSCFG_TSUE_MASK | MCAN_TSCFG_SCP_MASK |
                                   MCAN_TSCFG_TBCS_MASK | MCAN_TSCFG_EN64_MASK)) != MCAN_TSCFG_TSUE_MASK))
            return status_fail;
        mcan_interrupt_line_select(s_mcan[bus], config.interrupt_mask, 0U);
        s_channels[bus] = *channel;
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
bool dcu_can_receive(unsigned bus, dcu_can_frame_t *f, dcu_can_rx_stamp_t *stamp)
{
    if (!dcu_can_channel_config(bus) || !f || !stamp) return false;
    rx_queue_t *q=&s_rx[bus];intc_m_disable_irq(s_irq[bus]);
    bool available=q->read!=q->write;
    if(available){*f=q->item[q->read].frame;*stamp=q->item[q->read].stamp;q->read=(q->read+1)&31;}
    intc_m_enable_irq(s_irq[bus]);return available;
}
bool dcu_can_pending(unsigned bus)
{
    if (!dcu_can_channel_config(bus)) return false;
    for (unsigned i = 0; i < CANFD_TX_FIFO_DEPTH; ++i)
        if (s_tx[bus][i].id) return true;
    return false;
}
unsigned dcu_can_tx_free(unsigned bus)
{
    return dcu_can_channel_config(bus) ? MCAN_TXFQS_TFFL_GET(s_mcan[bus]->TXFQS) : 0;
}
uint32_t dcu_can_tx_progress(unsigned bus)
{
    return dcu_can_channel_config(bus) ? s_tx_progress[bus] : 0;
}

static void collect_tx_eofs(unsigned bus, MCAN_Type *base)
{
    mcan_tx_event_fifo_elem_t event;
    for (unsigned n = 0; n < CANFD_TX_FIFO_DEPTH && mcan_read_tx_evt_fifo(base, &event) == status_success; ++n) {
        const dcu_can_eof_t eof = read_eof(base, event.tx_timestamp_captured, event.tx_timestamp_pointer);
        // DAR may report a successful transmission as ET=2. TXBTO is still required.
        if (!eof.valid || (event.event_type != 1 && event.event_type != 2)) continue;
        for (unsigned i = 0; i < CANFD_TX_FIFO_DEPTH; ++i) {
            tx_slot_t *slot = &s_tx[bus][i];
            if (slot->id && (uint8_t)slot->id == event.message_marker) {
                slot->eof = eof;
                break;
            }
        }
    }
}

bool dcu_can_take_tx_event(unsigned bus, dcu_can_tx_event_t *event)
{
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel || !event) return false;
    MCAN_Type *base = s_mcan[bus];
    bool found = false;
    taskENTER_CRITICAL();
    capture_pending_protocol_fault(bus, CANFD_FAULT_BEFORE_TX_EVENT);
    const uint32_t pending = base->TXBRP;
    const uint32_t completed = base->TXBTO;
    const uint32_t cancelled = base->TXBCF;
    const uint32_t terminal = (completed | cancelled) & ~pending;
    if (channel->single_shot) collect_tx_eofs(bus, base);
    for (unsigned i = 0; i < CANFD_TX_FIFO_DEPTH; ++i) {
        tx_slot_t *slot = &s_tx[bus][i];
        if (!slot->id || !(terminal & (1U << i))) continue;
        const bool sent = !slot->revoked && (completed & (1U << i));
        *event = (dcu_can_tx_event_t){.id = slot->id, .status = sent ? DCU_CAN_TX_COMPLETED : DCU_CAN_TX_CANCELLED};
        if (sent) event->eof = slot->eof;
        if (sent) ++s_tx_progress[bus];
        slot->id = 0;
        found = true;
        break;
    }
    taskEXIT_CRITICAL();
    return found;
}

/* id == 0 revokes every pending request on this bus. Caller masks CAN and
 * watchdog IRQs with the FreeRTOS threshold, leaving Sync0/PDI unmasked. */
static bool cancel_locked(unsigned bus, dcu_can_tx_id_t id)
{
    capture_pending_protocol_fault(bus, CANFD_FAULT_BEFORE_CANCEL);
    const uint32_t pending = s_mcan[bus]->TXBRP;
    uint32_t requests = 0;
    for (unsigned i = 0; i < CANFD_TX_FIFO_DEPTH; ++i) {
        tx_slot_t *slot = &s_tx[bus][i];
        if (slot->id && (!id || slot->id == id) && !slot->revoked && (pending & (1U << i))) {
            slot->revoked = true;
            requests |= 1U << i;
        }
    }
    if (requests) s_mcan[bus]->TXBCR = requests;
    return requests != 0;
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

dcu_can_tx_id_t dcu_can_submit(unsigned bus, const dcu_can_frame_t *f)
{
    const dcu_can_channel_config_t *channel = dcu_can_channel_config(bus);
    if (!channel || !f || (!channel->fd && (f->flags & DCU_CAN_FLAG_FD))) return 0;
    if (dcu_tx_inhibited() || s_tx_serial[bus] == UINT64_MAX) return 0;
    if (!dcu_can_frame_valid(f) || dcu_can_bus_state(bus) == DCU_CAN_BUS_OFF) return 0;
    mcan_tx_frame_t tx={0};uint32_t index;
    tx.use_ext_id = (f->flags & DCU_CAN_FLAG_EXTENDED) != 0;
    if (tx.use_ext_id) tx.ext_id = f->id; else tx.std_id = f->id;
    tx.dlc = f->dlc;
    tx.canfd_frame = (f->flags & DCU_CAN_FLAG_FD) != 0;
    tx.bitrate_switch = (f->flags & DCU_CAN_FLAG_BRS) != 0;
    if (channel->single_shot) {
        tx.timestamp_capture_enable = true;
        tx.event_fifo_control = true;
        // At most one FIFO's worth of unconsumed tags can coexist per bus.
        tx.message_marker_l = (uint8_t)(s_tx_serial[bus] + 1U);
    }
    memcpy(tx.data_8, f->data, dcu_can_dlc_to_bytes(f->dlc, tx.canfd_frame));
    intc_m_disable_irq(s_irq[bus]);
    dcu_can_tx_id_t id = 0;
    if (mcan_request_and_fill_txfifo(s_mcan[bus], &tx, &index) == status_success && index < CANFD_TX_FIFO_DEPTH) {
        /* Stage outside the kernel critical section. Only the final gate, tag
         * and TX request are indivisible with respect to the fault timer.
         * Sync0/PDI remain enabled above the kernel priority threshold. */
        taskENTER_CRITICAL();
        /* Do not reuse a slot until its old result has been consumed. TXBAR
         * clears that slot's TXBTO and TXBCF before starting this request. */
        if (!dcu_tx_inhibited() && !s_tx[bus][index].id) {
            id = ++s_tx_serial[bus];
            s_tx[bus][index] = (tx_slot_t){.id = id};
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
