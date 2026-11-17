#ifndef EEPROM_TEST_FIXTURE_H
#define EEPROM_TEST_FIXTURE_H
#include "hpm_common.h"
/* The real emulation port is compiled; only register/ROM access is stubbed. */
#define HPM_ECAT_HW_H
#define E2P_ATTR
#define CSR_MSTATUS_MIE_MASK 8U
enum { status_success = 0, status_fail = 1, status_invalid_argument = 2 };
uint32_t disable_global_irq(uint32_t mask);
void restore_global_irq(uint32_t mask);
uint8_t ecat_calculate_eeprom_config_data_checksum(const uint8_t *data, size_t size);
#endif
