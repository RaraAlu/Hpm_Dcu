#include "bench_esc.h"
#include "cycle_timers.h"
#include "data_contract.h"
#include "ecat_def.h"
#include "applInterface.h"
#include "coeappl.h"
#include "ecatappl.h"
#include "ecatslv.h"
#include "hpm_ecat_hw.h"
#include "hpm_ecat_phy.h"
#include "realtime_port.h"
#ifdef DCU_DATA_BACKEND_MCAN
#include "mcan_echo.h"
#endif
#ifdef DCU_DATA_STATIC_PROFILE
#include "static_profile.h"
#endif
#ifdef DCU_DATA_SESSION_GATE
#include "session_bridge.h"
#endif
#ifdef DCU_DATA_CIH408_MIT_ONCE
#include "mit_once.h"
#endif

#include <stdio.h>

extern TOBJ1018 sIdentity;

static UINT16 sii_read(UINT32 word)
{
    return dcu_bench_sii_prepare_read(dcu_data_sii_read, word, EepromReadSize) ?
        0 : ESC_EEPROM_EMULATION_ACK_ERROR;
}

static UINT16 sii_deny_write(UINT32 word)
{
    (void)word;
    return ESC_EEPROM_EMULATION_ACK_ERROR;
}

static void data_poll(void)
{
    uint32_t last_ms = dcu_time_now_ms();
    for (;;) {
        (void)dcu_bench_sii_service(dcu_data_sii_read);
        MainLoop();
#ifdef DCU_DATA_SESSION_GATE
        dcu_data_session_step(bEcatOutputUpdateRunning && (nAlStatus & (STATE_MASK | STATE_CHANGE)) == STATE_OP);
#endif
        if (bEscIntEnabled && (SWAPWORD(HW_GetALEventRegister()) & PROCESS_OUTPUT_EVENT))
            PDI_Isr();
#ifdef DCU_DATA_BACKEND_MCAN
        dcu_data_mcan_poll();
#endif
#ifdef DCU_DATA_CIH408_MIT_ONCE
        mit_once_poll();
#endif
        uint32_t now_ms = dcu_time_now_ms();
        if (now_ms != last_ms) {
            last_ms = now_ms;
            ECAT_CheckTimer();
        }
    }
}

int main(void)
{
    board_init();
    dcu_data_reset();
    bool ready = dcu_timebase_init();
#ifdef DCU_DATA_CIH408_MIT_ONCE
    ready = ready && mit_once_init();
#endif
#ifdef DCU_DATA_BACKEND_MCAN
    ready = ready && dcu_data_mcan_init();
#endif
#ifdef DCU_DATA_SESSION_GATE
    ready = ready && dcu_data_session_boot();
#endif
    hpm_stat_t status = ready ?
        dcu_bench_esc_start(dcu_data_sii_read, DCU_DATA_LOAD_TIMEOUT_MS) : status_fail;
    UINT16 ssc_status = status == status_success ? MainInit() : 0;
    if (status != status_success || ssc_status != 0) {
        ecat_phy_keep_reset();
        printf("EtherCAT data init failed: ESC=%u SSC=%u\n", (unsigned)status, (unsigned)ssc_status);
        for (;;) __asm__ volatile("nop");
    }

    sIdentity.u32VendorID = DCU_DATA_VENDOR;
    sIdentity.u32Productcode = DCU_DATA_PRODUCT;
    sIdentity.u32Revision = DCU_DATA_REVISION;
    sIdentity.u32Serialnumber = 0;
    sSyncManOutPar.u16SyncTypesSupported = SYNCTYPE_SYNCHRONSUPP;
    sSyncManInPar.u16SyncTypesSupported = SYNCTYPE_SYNCHRONSUPP;
    pAPPL_EEPROM_Read = sii_read;
    pAPPL_EEPROM_Write = sii_deny_write;
    pAPPL_EEPROM_Reload = HW_EepromReload;

#ifdef DCU_DATA_STATIC_PROFILE
    const dcu_static_profile_t *profile = dcu_static_profile();
    printf("Static profile: MCAN%u internal, device %s\n", profile->bus, profile->device_id);
#endif
    printf("EtherCAT data ready: read-only SDO, %u/%u byte PDO echo\n",
           DCU_DATA_RX_BYTES, DCU_DATA_TX_BYTES);
    data_poll();
}
