#ifndef TEST_LOOPBACK_BOARD_H
#define TEST_LOOPBACK_BOARD_H
#include "hpm_mcan_drv.h"
MCAN_Type *test_loopback_controller(void);
#define HPM_MCAN0 (test_loopback_controller())
#define IRQn_MCAN0 46U
#ifdef DCU_DATA_STATIC_PROFILE
MCAN_Type *test_loopback_controller_at(unsigned bus);
#define HPM_MCAN1 (test_loopback_controller_at(1U))
#define HPM_MCAN2 (test_loopback_controller_at(2U))
#define HPM_MCAN3 (test_loopback_controller_at(3U))
#define IRQn_MCAN1 47U
#define IRQn_MCAN2 48U
#define IRQn_MCAN3 49U
#endif
uint32_t board_init_can_clock(MCAN_Type *base);
#endif
