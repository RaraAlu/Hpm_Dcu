#include "ethercat_canfd_gateway.h"

#include <string.h>
#include "motor_control.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_mcan_drv.h"

#define CANFD_NOMINAL_BITRATE         1000000U
#define CANFD_DATA_BITRATE            5000000U
#define CANFD_RX_FIFO_DEPTH           16U
#define CANFD_TX_FIFO_DEPTH           8U
#define CANFD_OUTPUT_WATCHDOG_CYCLES  3U
/* 事件保持两个 1 ms 节拍，保证跨定时边界时实际点亮时间仍不短于 1 ms。 */
#define CANFD_LED_PULSE_TICKS          2U

/* bus_state 写入 TxPDO 的稳定编码，供 TwinCAT 直接诊断。 */
enum {
    CANFD_BUS_ERROR_ACTIVE = 0,
    CANFD_BUS_ERROR_WARNING = 1,
    CANFD_BUS_ERROR_PASSIVE = 2,
    CANFD_BUS_OFF = 3,
};

typedef struct { dcu_can_frame_t frame; uint32_t timestamp; } rx_item_t;
typedef struct { rx_item_t item[32]; volatile uint8_t read, write; } rx_queue_t;

typedef struct {
    GPIO_Type *ctrl;
    uint32_t index;
    uint32_t pin;
    uint8_t on_level;
    uint8_t off_level;
} canfd_led_t;

enum {
    CANFD_LED_TX = 0,
    CANFD_LED_RX,
    CANFD_LED_ERR,
    CANFD_LED_KIND_COUNT,
};

/* 每路依次为 TX、RX、ERR，使用自研板上已有的 12 个低电平点亮 LED。 */
static const canfd_led_t s_led[CANFD_GATEWAY_BUS_COUNT][CANFD_LED_KIND_COUNT] = {
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
static MCAN_Type *const s_mcan[CANFD_GATEWAY_BUS_COUNT] = {
    HPM_MCAN0, HPM_MCAN1, HPM_MCAN2, HPM_MCAN3
};

static const uint32_t s_irq[CANFD_GATEWAY_BUS_COUNT] = {
    IRQn_MCAN0, IRQn_MCAN1, IRQn_MCAN2, IRQn_MCAN3
};

#if defined(MCAN_SOC_MSG_BUF_IN_AHB_RAM) && MCAN_SOC_MSG_BUF_IN_AHB_RAM
/* HPM5E31 的 MCAN Message RAM 位于专用 AHB SRAM，四路必须各自独立。 */
ATTR_PLACE_AT(".ahb_sram") static uint32_t s_mcan_ram[CANFD_GATEWAY_BUS_COUNT][MCAN_MSG_BUF_SIZE_IN_WORDS];
#endif

/* 下行使用完整快照；上行每路使用两个接收窗口，避免 ISR 改写正在发布的 PDO。 */
static rx_queue_t s_rx[4];
static volatile uint32_t s_rx_lost[4];
static volatile uint32_t s_tx_complete[ENCOS_CHANNEL_COUNT][ENCOS_MOTORS_PER_CHANNEL];
static volatile uint32_t s_tx_progress[CANFD_GATEWAY_BUS_COUNT];
static uint8_t s_tx_axis[4][8];
static volatile uint8_t s_led_ticks[CANFD_GATEWAY_BUS_COUNT][CANFD_LED_KIND_COUNT];
static uint16_t s_heartbeat_ticks;
static volatile uint32_t s_now_ms;
static bool s_initialized;

static void led_pulse(uint32_t bus, uint32_t kind)
{
    const canfd_led_t *led = &s_led[bus][kind];

    gpio_write_pin(led->ctrl, led->index, led->pin, led->on_level);
    s_led_ticks[bus][kind] = CANFD_LED_PULSE_TICKS;
}

void ethercat_canfd_gateway_led_tick(void)
{
    ++s_now_ms;
    if(s_initialized) motor_control_tick(s_now_ms);
    for (uint32_t bus = 0; bus < CANFD_GATEWAY_BUS_COUNT; bus++) {
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

uint16_t ethercat_canfd_bus_state(uint32_t bus)
{
    mcan_protocol_status_t status = {0};

    if (mcan_get_protocol_status(s_mcan[bus], &status) != status_success) {
        return CANFD_BUS_ERROR_WARNING;
    }
    if (status.in_bus_off_state) {
        return CANFD_BUS_OFF;
    }
    if (status.in_error_passive_state) {
        return CANFD_BUS_ERROR_PASSIVE;
    }
    if (status.in_warning_state) {
        return CANFD_BUS_ERROR_WARNING;
    }
    return CANFD_BUS_ERROR_ACTIVE;
}

static void receive_isr(uint32_t bus)
{
    MCAN_Type *base=s_mcan[bus]; uint32_t flags=mcan_get_interrupt_flags(base);
    mcan_clear_interrupt_flags(base,flags);
    while(mcan_get_rxfifo_fill_level(base,0U)) {
        mcan_rx_message_t f;
        if(mcan_read_rxfifo(base,0,&f)!=status_success)break;
        led_pulse(bus,CANFD_LED_RX);
        rx_queue_t *q=&s_rx[bus]; uint8_t next=(q->write+1)&31;
        if(f.dlc>8||f.rtr||next==q->read){++s_rx_lost[bus];led_pulse(bus,CANFD_LED_ERR);continue;}
        rx_item_t *item=&q->item[q->write];
        item->frame=(dcu_can_frame_t){.id=f.use_ext_id?f.ext_id:f.std_id,.dlc=f.dlc,
            .flags=(f.use_ext_id?2:0)|(f.canfd_frame?8:0)|(f.bitrate_switch?16:0)};
        memcpy(item->frame.data,f.data_8,f.dlc);item->timestamp=s_now_ms;
        __asm__ volatile("" ::: "memory");q->write=next;
    }
    if(flags & MCAN_EVENT_TRANSMIT) {
        uint32_t done=base->TXBTO;
        for(unsigned i=0;i<8;i++)if((done&(1u<<i)) && s_tx_axis[bus][i]!=255){
            ++s_tx_progress[bus];
            if(s_tx_axis[bus][i]<ENCOS_MOTORS_PER_CHANNEL)++s_tx_complete[bus][s_tx_axis[bus][i]];
            s_tx_axis[bus][i]=255;
        }
        led_pulse(bus,CANFD_LED_TX);
    }
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

hpm_stat_t ethercat_canfd_gateway_init(void)
{
    /*
     * PinMux 生成代码会在切换为 GPIO 时短暂写低，而这些 LED 为低电平点亮。
     * 在启用 MCAN 中断前再次显式写 OFF，保证上电且无报文时 12 个灯全部熄灭。
     */
    for (uint32_t bus = 0; bus < CANFD_GATEWAY_BUS_COUNT; bus++) {
        for (uint32_t kind = 0; kind < CANFD_LED_KIND_COUNT; kind++) {
            const canfd_led_t *led = &s_led[bus][kind];
            s_led_ticks[bus][kind] = 0U;
            gpio_write_pin(led->ctrl, led->index, led->pin, led->off_level);
        }
    }

    /* 四路统一使用 1 Mbit/s 仲裁段、5 Mbit/s 数据段和 CAN FD+BRS 能力。 */
    for (uint32_t bus = 0; bus < CANFD_GATEWAY_BUS_COUNT; bus++) {
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
        config.baudrate = CANFD_NOMINAL_BITRATE;
        config.baudrate_fd = CANFD_DATA_BITRATE;
        config.enable_canfd = true;
        config.enable_tdc = true;
        mcan_get_default_ram_config(s_mcan[bus], &config.ram_config, true);

        /*
         * 网关 PDO 固定透传 8 字节数据帧。驱动的 CAN FD 默认配置按 64 字节帧
         * 分配 FIFO；RX FIFO0 扩到 16 项后会超过每路 2560 字节 Message RAM，
         * 因此这里只保留网关实际使用的 RX FIFO0 和 TX FIFO。
         */
        config.ram_config.enable_std_filter = false;
        config.ram_config.enable_ext_filter = false;
        /* 不分配过滤表时也必须清零默认过滤项，否则驱动会拒绝该配置。 */
        config.all_filters_config.std_id_filter_list.mcan_filter_elem_count = 0U;
        config.all_filters_config.ext_id_filter_list.mcan_filter_elem_count = 0U;
        config.ram_config.rxfifos[0].enable = 1U;
        config.ram_config.rxfifos[0].elem_count = CANFD_RX_FIFO_DEPTH;
        config.ram_config.rxfifos[0].data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
        config.ram_config.rxfifos[1].enable = 0U;
        config.ram_config.enable_rxbuf = false;
        config.ram_config.txbuf_data_field_size = MCAN_DATA_FIELD_SIZE_8BYTES;
        config.ram_config.txbuf_dedicated_txbuf_elem_count = 0U;
        config.ram_config.txbuf_fifo_or_queue_elem_count = CANFD_TX_FIFO_DEPTH;
        config.ram_config.txfifo_or_txqueue_mode = 0U; /* 0=FIFO，保持槽位发送顺序。 */
        config.ram_config.enable_tx_evt_fifo = false;
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
        mcan_interrupt_line_select(s_mcan[bus], config.interrupt_mask, 0U);
        board_can_transceiver_phy_set(s_mcan[bus], true);
        intc_m_enable_irq_with_priority(s_irq[bus], 2U);
    }
    memset(s_tx_axis,255,sizeof(s_tx_axis));
    s_initialized=true;
    return status_success;
}


uint32_t ethercat_canfd_now_ms(void) { return s_now_ms; }
uint32_t ethercat_canfd_now_us(void)
{
    const uint32_t ticks_per_us = clock_get_core_clock_ticks_per_us();
    return ticks_per_us ? (uint32_t)(hpm_csr_get_core_cycle() / ticks_per_us) : 0U;
}
uint32_t ethercat_canfd_rx_drops(unsigned bus) { return s_rx_lost[bus]; }
uint32_t ethercat_canfd_completions(unsigned axis) { return s_tx_complete[axis/ENCOS_MOTORS_PER_CHANNEL][axis%ENCOS_MOTORS_PER_CHANNEL]; }
bool ethercat_canfd_receive(unsigned bus, dcu_can_frame_t *f, uint32_t *stamp)
{
    rx_queue_t *q=&s_rx[bus];uint32_t irq=disable_global_irq(CSR_MSTATUS_MIE_MASK);
    bool available=q->read!=q->write;
    if(available){*f=q->item[q->read].frame;*stamp=q->item[q->read].timestamp;q->read=(q->read+1)&31;}
    restore_global_irq(irq&CSR_MSTATUS_MIE_MASK);return available;
}
bool ethercat_canfd_pending(unsigned bus) { return s_mcan[bus]->TXBRP!=0; }
bool ethercat_canfd_axis_pending(unsigned axis)
{
    if(axis>=ENCOS_AXIS_COUNT)return false;
    unsigned bus=axis/ENCOS_MOTORS_PER_CHANNEL;
    uint8_t tag=axis%ENCOS_MOTORS_PER_CHANNEL;
    uint32_t irq=disable_global_irq(CSR_MSTATUS_MIE_MASK);
    uint32_t pending=s_mcan[bus]->TXBRP;
    bool found=false;
    for(unsigned i=0;i<CANFD_TX_FIFO_DEPTH;i++)if((pending&(1u<<i))&&s_tx_axis[bus][i]==tag){found=true;break;}
    restore_global_irq(irq&CSR_MSTATUS_MIE_MASK);
    return found;
}
unsigned ethercat_canfd_tx_free(unsigned bus)
{
    return bus<CANFD_GATEWAY_BUS_COUNT?MCAN_TXFQS_TFFL_GET(s_mcan[bus]->TXFQS):0;
}
uint32_t ethercat_canfd_tx_progress(unsigned bus)
{
    return bus<CANFD_GATEWAY_BUS_COUNT?s_tx_progress[bus]:0;
}
void ethercat_canfd_cancel(unsigned bus) { for(unsigned i=0;i<8;i++)mcan_cancel_tx_buf_send_request(s_mcan[bus],i); }
static void cancel_tag(unsigned bus,uint8_t tag)
{
    if(bus>=CANFD_GATEWAY_BUS_COUNT)return;
    uint32_t irq=disable_global_irq(CSR_MSTATUS_MIE_MASK),pending=s_mcan[bus]->TXBRP;
    for(unsigned i=0;i<CANFD_TX_FIFO_DEPTH;i++)if((pending&(1u<<i))&&s_tx_axis[bus][i]==tag)mcan_cancel_tx_buf_send_request(s_mcan[bus],i);
    restore_global_irq(irq&CSR_MSTATUS_MIE_MASK);
}
void ethercat_canfd_cancel_axis(unsigned axis)
{
    if(axis<ENCOS_AXIS_COUNT)cancel_tag(axis/ENCOS_MOTORS_PER_CHANNEL,axis%ENCOS_MOTORS_PER_CHANNEL);
}
void ethercat_canfd_cancel_service(unsigned bus) { cancel_tag(bus,254); }
bool ethercat_canfd_send(unsigned bus,const dcu_can_frame_t *f,unsigned axis)
{
    if(bus>=4||f->dlc>8||(f->flags&~26u)||((f->flags&16)&&!(f->flags&8))||f->id>((f->flags&2)?0x1FFFFFFFu:0x7FFu)||ethercat_canfd_bus_state(bus)==3)return false;
    mcan_tx_frame_t tx={0};uint32_t index;
    tx.use_ext_id=(f->flags&2)!=0;if(tx.use_ext_id)tx.ext_id=f->id;else tx.std_id=f->id;
    tx.dlc=f->dlc;tx.canfd_frame=(f->flags&8)!=0;tx.bitrate_switch=(f->flags&16)!=0;memcpy(tx.data_8,f->data,f->dlc);
    uint32_t irq=disable_global_irq(CSR_MSTATUS_MIE_MASK);
    bool ok=mcan_transmit_via_txfifo_nonblocking(s_mcan[bus],&tx,&index)==status_success;
    if(ok)s_tx_axis[bus][index]=axis<ENCOS_AXIS_COUNT?axis%ENCOS_MOTORS_PER_CHANNEL:254;
    restore_global_irq(irq&CSR_MSTATUS_MIE_MASK);
    if(!ok) {
        led_pulse(bus,CANFD_LED_ERR);
    }
    return ok;
}
