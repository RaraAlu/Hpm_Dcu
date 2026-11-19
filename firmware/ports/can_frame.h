#ifndef DCU_CAN_FRAME_H
#define DCU_CAN_FRAME_H

#include <stdbool.h>
#include <stdint.h>

/* Internal capacity; EtherCAT wire formats define their own limits. */
#define DCU_CAN_MAX_PAYLOAD 64U
#define DCU_CAN_CLASSIC_MAX_PAYLOAD 8U

enum {
    DCU_CAN_FLAG_EXTENDED = 1U << 1,
    DCU_CAN_FLAG_FD = 1U << 3,
    DCU_CAN_FLAG_BRS = 1U << 4
};

typedef struct {
    uint32_t id;
    uint8_t dlc; /* Wire DLC code, not a byte count beyond 8. */
    uint8_t flags;
    uint8_t data[DCU_CAN_MAX_PAYLOAD];
} dcu_can_frame_t;

/* UINT8_MAX denotes a DLC outside the supported range for this frame format. */
static inline uint8_t dcu_can_dlc_to_bytes(uint8_t dlc, bool fd)
{
    static const uint8_t fd_bytes[] = {12U, 16U, 20U, 24U, 32U, 48U, 64U};
    if (dlc <= DCU_CAN_CLASSIC_MAX_PAYLOAD) return dlc;
    if (!fd || dlc > 15U) return UINT8_MAX;
    return fd_bytes[dlc - 9U];
}

static inline bool dcu_can_frame_valid(const dcu_can_frame_t *frame)
{
    const uint8_t allowed = DCU_CAN_FLAG_EXTENDED | DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    const bool fd = (frame->flags & DCU_CAN_FLAG_FD) != 0U;
    return !(frame->flags & ~allowed) && (fd || !(frame->flags & DCU_CAN_FLAG_BRS)) &&
           dcu_can_dlc_to_bytes(frame->dlc, fd) <= sizeof(frame->data) &&
           frame->id <= ((frame->flags & DCU_CAN_FLAG_EXTENDED) ? 0x1fffffffU : 0x7ffU);
}

#endif
