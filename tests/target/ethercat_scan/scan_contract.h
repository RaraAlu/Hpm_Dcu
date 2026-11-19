#ifndef DCU_ETHERCAT_SCAN_CONTRACT_H
#define DCU_ETHERCAT_SCAN_CONTRACT_H

#include <stdbool.h>
#include <stdint.h>

enum {
    DCU_SCAN_AL_INIT = 1,
    DCU_SCAN_AL_PREOP = 2,
    DCU_SCAN_AL_ERROR = 0x10,
    DCU_SCAN_AL_INVALID_TRANSITION = 0x11,
};

bool dcu_scan_sii_read(uint32_t word_addr, unsigned size, uint64_t *data);
uint16_t dcu_scan_al_status(uint16_t current_status, uint16_t control, uint16_t *status_code);

#endif
