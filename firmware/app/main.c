/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include "board.h"
#include "ecat_def.h"
#include "ecatappl.h"
#include "ecatslv.h"
#include "applInterface.h"
#include "digital_io.h"
#include "ethercat_canfd_gateway.h"
#include "motor_control.h"
#include "cycle_runtime.h"
#include "coeappl.h"
#include "hpm_ecat_e2p_emulation.h"
extern TOBJ1018 sIdentity;
#include "hpm_ecat_hw.h"
#include "hpm_l1c_drv.h"

#ifndef CONFIG_CAN_LED_TEST
#define CONFIG_CAN_LED_TEST 0
#endif

#if CONFIG_CAN_LED_TEST
#include "can_led_test.h"
#endif

int main(void)
{
    hpm_stat_t stat;
    board_init();
#if defined(CONFIG_CAN_LED_TEST) && CONFIG_CAN_LED_TEST
    can_led_test_run();
#endif
    board_init_led_pins();
    board_init_can_led_pins();
    /* 1 ms 非阻塞节拍同时服务 CAN 活动灯；回调内部每 1000 次翻转原心跳灯。 */
    board_timer_create(1, ethercat_canfd_gateway_led_tick);
    board_init_ethercat(HPM_ESC); /* init ESC function pins */
    board_init_switch_led();      /* init switch and led for ECAT display */
    printf("EtherCAT IO sample\n");

    /* hardware init */
    stat = ecat_hardware_init(HPM_ESC);
    if (stat != status_success) {
        printf("Init ESC peripheral and related devices(EEPROM/PHY) failed!\n");
        return 0;
    }
    /* Keep 0x1018:04 aligned with the per-board serial retained in SII. */
    sIdentity.u32Serialnumber = ecat_flash_eeprom_serial();

    /* 初始化四路 CAN FD；任一路失败都不启动 EtherCAT 应用，避免部分总线误运行。 */
    stat = ethercat_canfd_gateway_init();
    if (stat != status_success) {
        printf("Init EtherCAT/CAN FD gateway failed: %d\n", stat);
        return 0;
    }

    motor_control_init();
    dcu_cycle_init();
    MainInit(); /* SSC Initilize the stack */
    sSyncManOutPar.u16SyncTypesSupported = SYNCTYPE_DCSYNC0SUPP;
    sSyncManInPar.u16SyncTypesSupported = SYNCTYPE_DCSYNC0SUPP;

#if defined(ESC_EEPROM_EMULATION) && ESC_EEPROM_EMULATION
    pAPPL_EEPROM_Read  = ecat_eeprom_emulation_read;
    pAPPL_EEPROM_Write = ecat_eeprom_emulation_write;
    pAPPL_EEPROM_Reload = ecat_eeprom_emulation_reload;
    pAPPL_EEPROM_Store  = ecat_eeprom_emulation_store;
#endif

    /* Create basic mapping */
    APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize);

    /* Set stack run flag */
    bRunApplication = TRUE;
    dcu_rtos_start();

    /* hardware deinit */

    return 0;
}
