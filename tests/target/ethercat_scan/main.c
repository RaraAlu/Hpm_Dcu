#include "bench_esc.h"
#include "hpm_ecat_phy.h"
#include "scan_contract.h"
#include "cfg/scan_config.h"

#include <stdio.h>

static volatile hpm_stat_t s_scan_status = status_fail;

__attribute__((noinline)) static void scan_poll(void)
{
    for (;;) {
        (void)dcu_bench_sii_service(dcu_scan_sii_read);
        if (HPM_ESC->AL_EVT_REQ & ESC_AL_EVT_REQ_ALC_EVT_MASK) {
            uint16_t control = HPM_ESC->AL_CTRL;
            uint16_t status_code;
            uint16_t status = dcu_scan_al_status(HPM_ESC->AL_STAT, control, &status_code);
            HPM_ESC->AL_STAT_CODE = status_code;
            HPM_ESC->AL_STAT = status;
        }
    }
}

int main(void)
{
    board_init();
    s_scan_status = dcu_bench_esc_start(dcu_scan_sii_read, DCU_SCAN_LOAD_TIMEOUT_MS);
    if (s_scan_status != status_success) {
        ecat_phy_keep_reset();
        printf("EtherCAT scan init failed: %u\n", (unsigned)s_scan_status);
        for (;;) __asm__ volatile("nop");
    }
    printf("EtherCAT scan ready: no mailbox, PDO or CAN\n");
    scan_poll();
}
