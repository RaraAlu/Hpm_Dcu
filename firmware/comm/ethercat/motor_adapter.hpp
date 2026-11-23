#ifndef DCU_MOTOR_ADAPTER_HPP
#define DCU_MOTOR_ADAPTER_HPP

#include <cstdint>
#include "production_profile.h"
#include "motor_pdo_layout.hpp"

namespace dcu::ethercat::motor_adapter {
// Project the gateway's validated, immutable Profile into the SSC dictionary.
bool initialize(const dcu_production_profile_t &profile);
bool stage_selection(uint32_t buses, uint32_t axes, uint32_t generation);
const pdo_layout::Layout &requested_layout();
bool commit_selection();
uint32_t topology_generation();
void clear_layout_commands();
const dcu_motor_device_profile_t *configured_device(unsigned slot);
// Unmapped slots/devices return the corresponding capacity sentinel.
unsigned device_for_slot(unsigned slot);
unsigned slot_for_device(unsigned device);
uint32_t slots_for_devices(uint32_t devices);
void refresh_axis_feedback(unsigned axis);
void commit_sdo_command(unsigned axis, uint32_t now_ms);
}
#endif
