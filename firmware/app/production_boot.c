/*
 * Copyright (c) 2024 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "production_init.h"
#include <stdio.h>
#include "board.h"
#include "ecat_def.h"
#include "ecatappl.h"
#include "ecatslv.h"
#include "applInterface.h"
#include "hpm_mcan_transport.h"
#include "gateway_identity.h"
#include "motor_ethercat.h"
#include "ethercat_cycle.h"
#include "cycle_timers.h"
#include "coeappl.h"
#include "hpm_ecat_e2p_emulation.h"
#include "hpm_ecat_hw.h"

extern TOBJ1018 sIdentity;
static bool init_board_and_buses(const dcu_production_profile_t *profile);
static void init_ethercat_al(void);

bool dcu_production_boot(dcu_pdo_mode_t boot_mode)
{
    const dcu_production_profile_t *profile = dcu_production_profile();
    if (!init_board_and_buses(profile)) return false;
    if (!dcu_motor_axes_init(profile)) {
        printf("Init motor axes failed!\n");
        return false;
    }
    if (!dcu_pdo_layout_init(profile, boot_mode) || !motor_control_verify_contract()) {
        printf("Init PDO layout or contract failed!\n");
        return false;
    }
    if (!dcu_ssc_bind_sdo_write(DCU_SDO_2101_PDO_LAYOUT, dcu_pdo_layout_sdo_write) ||
        !dcu_ssc_bind_sdo_write(DCU_SDO_2103_PDO_TOPOLOGY, dcu_pdo_topology_sdo_write)) {
        printf("Bind PDO reload SDOs failed!\n");
        return false;
    }
    const uint64_t boot_id = dcu_gateway_ids_boot(dcu_gateway_flash_storage());
    if (!dcu_gateway_readiness_boot(&profile->readiness, boot_id))
        printf("Gateway startup fault: business transmission inhibited\n");

    dcu_cycle_init();
    init_ethercat_al();
    bRunApplication = TRUE;
    return true;
}

static bool init_board_and_buses(const dcu_production_profile_t *profile)
{
    hpm_stat_t stat;
    board_init();
    board_init_led_pins();
    board_init_can_led_pins();
    if (!dcu_timebase_init()) {
        printf("Init cycle timebase failed!\n");
        return false;
    }

    board_timer_create(1, dcu_mcan_led_tick);
    stat = dcu_mcan_init(profile->channels, profile->channel_count);
    if (stat != status_success) {
        printf("Init CAN Profile failed: %d\n", stat);
        return false;
    }
    board_init_ethercat(HPM_ESC); /* init ESC function pins */
    board_init_switch_led();      /* init switch and led for ECAT display */
    printf("EtherCAT IO sample\n");

    /* hardware init */
    stat = ecat_hardware_init(HPM_ESC);
    if (stat != status_success) {
        printf("Init ESC peripheral and related devices(EEPROM/PHY) failed!\n");
        return false;
    }
    /* Keep 0x1018:04 aligned with the per-board serial retained in SII. */
    sIdentity.u32Serialnumber = ecat_flash_eeprom_serial();
    return true;
}

static void init_ethercat_al(void)
{
    MainInit();
    sSyncManOutPar.u16SyncTypesSupported = SYNCTYPE_SYNCHRONSUPP | SYNCTYPE_DCSYNC0SUPP;
    sSyncManInPar.u16SyncTypesSupported = SYNCTYPE_SYNCHRONSUPP | SYNCTYPE_DCSYNC0SUPP;

#if defined(ESC_EEPROM_EMULATION) && ESC_EEPROM_EMULATION
    pAPPL_EEPROM_Read  = ecat_eeprom_emulation_read;
    pAPPL_EEPROM_Write = ecat_eeprom_emulation_write;
    pAPPL_EEPROM_Reload = ecat_eeprom_emulation_reload;
    pAPPL_EEPROM_Store  = ecat_eeprom_emulation_store;
#endif
}
