#ifndef ETHERCAT_CANFD_GATEWAY_H
#define ETHERCAT_CANFD_GATEWAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ENCOS_CHANNEL_COUNT 4u
#define ENCOS_MOTORS_PER_CHANNEL 7u
#define ENCOS_AXIS_COUNT (ENCOS_CHANNEL_COUNT * ENCOS_MOTORS_PER_CHANNEL)
#include "hpm_common.h"

/* 固定过程映像：4 路总线，每路 7 个帧槽，每个方向共 512 字节。 */
#define CANFD_GATEWAY_BUS_COUNT       4U
#define CANFD_GATEWAY_SLOTS_PER_BUS   7U
#define CANFD_GATEWAY_PROCESS_SIZE    512U
/* 保持 16 字节 PDO 槽不变：支持 DLC 0～8，DLC 9～15 不做截断透传。 */
#define CANFD_GATEWAY_MAX_DATA_BYTES  8U

#define CANFD_FLAG_VALID              (1U << 0)
#define CANFD_FLAG_IDE                (1U << 1)
#define CANFD_FLAG_RTR                (1U << 2)
#define CANFD_FLAG_FDF                (1U << 3)
#define CANFD_FLAG_BRS                (1U << 4)
#define CANFD_FLAG_ON_CHANGE          (1U << 5)

#define CANFD_INPUT_FLAG_NEW          CANFD_FLAG_VALID
#define CANFD_INPUT_FLAG_ESI          CANFD_FLAG_ON_CHANGE

/* 单帧 PDO 槽。packed 保证与 TwinCAT/ESI 的 16 字节布局完全一致。 */
typedef struct __attribute__((packed)) {
    uint32_t can_id;     /* 标准帧使用低 11 bit，扩展帧使用低 29 bit。 */
    uint8_t flags;       /* VALID/IDE/RTR/FDF/BRS/ON_CHANGE 或上行 NEW/ESI。 */
    uint8_t dlc;         /* 支持 DLC 0～8；仅前 dlc 个数据字节有效。 */
    uint16_t seq;        /* 下行用于变化检测，上行由从站逐帧递增。 */
    uint8_t data[CANFD_GATEWAY_MAX_DATA_BYTES]; /* 未使用的尾部字节保持为 0。 */
} canfd_slot_pdo_t;

/* 单路 PDO：16 字节总线头 + 7 × 16 字节帧槽。 */
typedef struct __attribute__((packed)) {
    uint16_t cycle_seq;  /* EtherCAT 周期序号；连续 3 周期不变则停止发送。 */
    uint8_t valid_mask;  /* Bit0～6 对应 Slot1～7；Bit7 必须为 0。 */
    uint8_t mode;        /* 下行保留；上行为本周期接收帧数。 */
    uint16_t deadline_us;/* 下行发送预算，首版固定映射但不在 ISR 内等待。 */
    uint16_t bus_state;  /* 上行：0正常、1警告、2被动、3 bus-off。 */
    uint32_t counter0;   /* 上行累计接收丢弃数。 */
    uint32_t counter1;   /* 上行累计 bus-off 次数。 */
    canfd_slot_pdo_t slot[CANFD_GATEWAY_SLOTS_PER_BUS];
} canfd_bus_pdo_t;

typedef struct __attribute__((packed)) {
    canfd_bus_pdo_t bus[CANFD_GATEWAY_BUS_COUNT];
} ethercat_canfd_process_image_t;

_Static_assert(sizeof(canfd_slot_pdo_t) == 16, "CAN FD PDO slot must be 16 bytes");
_Static_assert(offsetof(canfd_slot_pdo_t, data) == 8, "CAN FD payload offset must be 8");
_Static_assert(sizeof(canfd_bus_pdo_t) == 128, "CAN FD bus PDO must be 128 bytes");
_Static_assert(sizeof(ethercat_canfd_process_image_t) == CANFD_GATEWAY_PROCESS_SIZE,
               "EtherCAT process image must be 512 bytes");

#include "encos_motor.h"
hpm_stat_t ethercat_canfd_gateway_init(void);
void ethercat_canfd_gateway_led_tick(void);
uint32_t ethercat_canfd_now_ms(void);
uint32_t ethercat_canfd_now_us(void);
uint32_t ethercat_canfd_rx_drops(unsigned bus);
uint32_t ethercat_canfd_completions(unsigned axis);
uint16_t ethercat_canfd_bus_state(uint32_t bus);
bool ethercat_canfd_receive(unsigned bus, encos_frame_t *frame, uint32_t *stamp);
bool ethercat_canfd_pending(unsigned bus);
bool ethercat_canfd_axis_pending(unsigned axis);
unsigned ethercat_canfd_tx_free(unsigned bus);
uint32_t ethercat_canfd_tx_progress(unsigned bus);
void ethercat_canfd_cancel(unsigned bus);
void ethercat_canfd_cancel_axis(unsigned axis);
void ethercat_canfd_cancel_service(unsigned bus);
bool ethercat_canfd_send(unsigned bus,const encos_frame_t *frame,unsigned axis);
#endif
