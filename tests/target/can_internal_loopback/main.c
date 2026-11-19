#include <stdio.h>
#include "board.h"
#include "can_internal_loopback.h"

__attribute__((noinline)) void dcu_can_loopback_finished(void)
{
    for (;;) {
        __asm__ volatile("nop");
    }
}

int main(void)
{
    board_init();
    dcu_can_loopback_run();
    const volatile dcu_can_loopback_report_t *report = dcu_can_loopback_report();
    printf("MCAN0 INTERNAL LOOPBACK ONLY: result=%u checked=%u tx=%u rx=%u stopped=%u\n",
           (unsigned)report->result, (unsigned)report->frames_checked,
           (unsigned)report->tx_completed, (unsigned)report->rx_received,
           (unsigned)report->controller_stopped);
    for (unsigned i = 0; i < report->frames_checked; ++i) {
        printf("frame=%u matched=%u elapsed_us=%u\n", i,
               (unsigned)report->frames[i].matched, (unsigned)report->frames[i].elapsed_us);
    }
    printf("%s: no external CAN, EtherCAT, or production gateway acceptance\n",
           report->result == DCU_LOOPBACK_PASS ? "PASS" : "FAIL");
    dcu_can_loopback_finished();
    return 0;
}
