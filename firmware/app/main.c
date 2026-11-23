/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>
#include "cfg/diagnostic_config.h"
#include "board.h"
#include "ecat_def.h"
#include "ecatappl.h"
#include "ecatslv.h"
#include "applInterface.h"
#include "digital_io.h"
#include "hpm_mcan_transport.h"
#include "production_profile.h"
#include "gateway_identity.h"
#include "production_init.h"
#include "motor_ethercat.h"
#include "ethercat_cycle.h"
#include "ethercat_can_task.h"
#include "cycle_timers.h"
#include "coeappl.h"
#include "hpm_ecat_e2p_emulation.h"
extern TOBJ1018 sIdentity;
#include "hpm_ecat_hw.h"
#include "hpm_l1c_drv.h"

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
    if (!dcu_timebase_init()) {
        printf("Init cycle timebase failed!\n");
        return 0;
    }

    board_timer_create(1, dcu_mcan_led_tick);
    const dcu_production_profile_t *profile = dcu_production_profile();
    stat = dcu_mcan_init(profile->channels, profile->channel_count);
    if (stat != status_success) {
        printf("Init CAN Profile failed: %d\n", stat);
        return 0;
    }
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

    if (!motor_control_init() || !motor_control_verify_contract()) {
        printf("Init device Profile failed!\n");
        return 0;
    }
    const uint64_t boot_id = dcu_gateway_ids_boot(dcu_gateway_flash_storage());
    if (!motor_control_boot_startup(boot_id))
        printf("Gateway startup fault: business transmission inhibited\n");
    dcu_cycle_init();
    MainInit(); /* SSC Initilize the stack */
    sSyncManOutPar.u16SyncTypesSupported = SYNCTYPE_SYNCHRONSUPP | SYNCTYPE_DCSYNC0SUPP;
    sSyncManInPar.u16SyncTypesSupported = SYNCTYPE_SYNCHRONSUPP | SYNCTYPE_DCSYNC0SUPP;

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
