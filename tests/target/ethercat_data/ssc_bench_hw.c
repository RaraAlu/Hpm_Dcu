#include "hpm_ecat_hw.h"
#include "bench_esc.h"
#include "data_contract.h"
#include "realtime_port.h"

#include <string.h>

MEM_ADDR ESCMEM *pEsc = (MEM_ADDR ESCMEM *)HPM_ESC_BASE;
static uint32_t s_timer_origin_ms;

UINT32 HW_GetTimer(void) { return dcu_time_now_ms() - s_timer_origin_ms; }
void HW_ClearTimer(void) { s_timer_origin_ms = dcu_time_now_ms(); }

/* The bench services the ESC in one polling loop, with no ESC IRQs enabled. */
void DISABLE_ESC_INT(void) {}
void ENABLE_ESC_INT(void) {}

UINT16 HW_EepromReload(void)
{
    return dcu_bench_sii_prepare_reload(dcu_data_sii_read) ?
        0 : ESC_EEPROM_EMULATION_ACK_ERROR;
}

void hw_esc_read_4bytes(void *dest, const void *src, uint16_t size)
{
    uint8_t *out = dest;
    const volatile uint32_t *in = src;
    while (size) {
        uint32_t word = *in++;
        uint16_t len = size < sizeof(word) ? size : sizeof(word);
        memcpy(out, &word, len);
        out += len;
        size -= len;
    }
}
