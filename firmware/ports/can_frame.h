#ifndef DCU_CAN_FRAME_H
#define DCU_CAN_FRAME_H

#include <stdint.h>

/* The current transport and EtherCAT compatibility path support DLC 0..8. */
#define DCU_CAN_MAX_PAYLOAD 8U

enum {
    DCU_CAN_FLAG_EXTENDED = 1U << 1,
    DCU_CAN_FLAG_FD = 1U << 3,
    DCU_CAN_FLAG_BRS = 1U << 4
};

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t flags;
    uint8_t data[DCU_CAN_MAX_PAYLOAD];
} dcu_can_frame_t;

#endif
