#ifndef DCU_MOTOR_ADAPTER_HPP
#define DCU_MOTOR_ADAPTER_HPP

#include <cstdint>
#include "production_profile.h"

namespace dcu::ethercat::motor_adapter {
// Project the gateway's validated, immutable Profile into the SSC dictionary.
bool initialize(const dcu_production_profile_t &profile);
const dcu_motor_device_profile_t *configured_device(unsigned slot);
// Unmapped slots/devices return the corresponding capacity sentinel.
unsigned device_for_slot(unsigned slot);
unsigned slot_for_device(unsigned device);
uint32_t slots_for_devices(uint32_t devices);
void refresh_axis_feedback(unsigned axis);
void commit_sdo_command(unsigned axis, uint32_t now_ms);
}
#endif
