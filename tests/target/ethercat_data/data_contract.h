#ifndef DCU_ETHERCAT_DATA_CONTRACT_H
#define DCU_ETHERCAT_DATA_CONTRACT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "cfg/data_config.h"

bool dcu_data_sii_read(uint32_t word, unsigned size, uint64_t *data);
void dcu_data_reset(void);
bool dcu_data_capture(const void *rx, size_t size);
bool dcu_data_copy(void *tx, size_t size);
size_t dcu_data_read(uint16_t index, uint8_t sub, void *data, size_t capacity);

#endif
