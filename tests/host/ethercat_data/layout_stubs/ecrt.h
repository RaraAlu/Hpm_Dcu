#pragma once
#include <cstddef>
#include <cstdint>

struct ec_master_t {};
struct ec_domain_t {};
struct ec_slave_config_t {};
enum ec_direction_t { EC_DIR_OUTPUT, EC_DIR_INPUT };
struct ec_sync_info_t { unsigned n_pdos; ec_direction_t dir; };
struct ec_pdo_info_t { uint16_t index; unsigned n_entries; };
struct ec_pdo_entry_info_t { uint16_t index; uint8_t subindex, bit_length; };
int ecrt_master_sdo_upload(ec_master_t *, uint16_t, uint16_t, uint8_t, uint8_t *, size_t, size_t *, uint32_t *);
int ecrt_master_get_sync_manager(ec_master_t *, uint16_t, uint8_t, ec_sync_info_t *);
int ecrt_master_get_pdo(ec_master_t *, uint16_t, uint8_t, uint16_t, ec_pdo_info_t *);
int ecrt_master_get_pdo_entry(ec_master_t *, uint16_t, uint8_t, uint16_t, uint16_t, ec_pdo_entry_info_t *);
int ecrt_slave_config_reg_pdo_entry_pos(ec_slave_config_t *, uint8_t, unsigned, unsigned, ec_domain_t *, unsigned *);
