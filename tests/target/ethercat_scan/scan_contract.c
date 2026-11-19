#include "scan_contract.h"
#include "cfg/scan_config.h"

static const uint8_t s_sii[DCU_SCAN_SII_BYTES] = {
    /* Reuse the board's PDI configuration and its CRC, not the production identity. */
    [0] = 0x80, 0x0c, 0x81, 0x66,
    [14] = 0x62,
    [16] = DCU_SCAN_VENDOR & 0xffU, (DCU_SCAN_VENDOR >> 8) & 0xffU,
    (DCU_SCAN_VENDOR >> 16) & 0xffU, (DCU_SCAN_VENDOR >> 24) & 0xffU,
    [20] = DCU_SCAN_PRODUCT & 0xffU, (DCU_SCAN_PRODUCT >> 8) & 0xffU,
    (DCU_SCAN_PRODUCT >> 16) & 0xffU, (DCU_SCAN_PRODUCT >> 24) & 0xffU,
    [24] = DCU_SCAN_REVISION & 0xffU, (DCU_SCAN_REVISION >> 8) & 0xffU,
    (DCU_SCAN_REVISION >> 16) & 0xffU, (DCU_SCAN_REVISION >> 24) & 0xffU,
    [124] = DCU_SCAN_SII_BYTES / 128U - 1U,
    [126] = 1U,
    /* Four disabled SM descriptors; no mailbox protocol or mapped PDO. */
    [128] = 41, 0, 16, 0,
    [152] = 0x24, [155] = 3, /* SM2: output direction, zero capacity. */
    [160] = 0x20, [163] = 4, /* SM3: input direction, zero capacity. */
    [164] = 0xff, 0xff,
};

bool dcu_scan_sii_read(uint32_t word_addr, unsigned size, uint64_t *data)
{
    if (!data || (size != 4U && size != 8U) ||
        word_addr > (DCU_SCAN_SII_BYTES - size) / 2U) return false;
    uint64_t result = 0U;
    for (unsigned i = 0; i < size; ++i) {
        result |= (uint64_t)s_sii[word_addr * 2U + i] << (8U * i);
    }
    *data = result;
    return true;
}

uint16_t dcu_scan_al_status(uint16_t current_status, uint16_t control, uint16_t *status_code)
{
    uint16_t current = current_status & 0x0fU;
    uint16_t requested = control & 0x0fU;
    if (current != DCU_SCAN_AL_PREOP) current = DCU_SCAN_AL_INIT;
    if (requested != DCU_SCAN_AL_INIT && requested != DCU_SCAN_AL_PREOP) {
        *status_code = DCU_SCAN_AL_INVALID_TRANSITION;
        return current | DCU_SCAN_AL_ERROR;
    }
    if ((current_status & DCU_SCAN_AL_ERROR) && !(control & DCU_SCAN_AL_ERROR) &&
        requested != DCU_SCAN_AL_INIT) {
        *status_code = DCU_SCAN_AL_INVALID_TRANSITION;
        return current | DCU_SCAN_AL_ERROR;
    }
    *status_code = 0U;
    return requested;
}
