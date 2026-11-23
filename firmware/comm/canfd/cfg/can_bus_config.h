#ifndef DCU_CAN_BUS_CONFIG_H
#define DCU_CAN_BUS_CONFIG_H

#define CANFD_RX_FIFO_DEPTH 16U
#define CANFD_TX_FIFO_DEPTH 8U
#define CANFD_PACED_SLOTS 8U
#define CANFD_TX_SLOTS (CANFD_PACED_SLOTS + CANFD_TX_FIFO_DEPTH)
/* Two 1 ms ticks retain at least one full tick of visible activity. */
#define CANFD_LED_PULSE_TICKS 2U

#endif
