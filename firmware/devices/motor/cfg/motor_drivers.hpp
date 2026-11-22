#ifndef DCU_MOTOR_DRIVERS_CONFIG_HPP
#define DCU_MOTOR_DRIVERS_CONFIG_HPP

#include <cstdint>

namespace dcu::motor::config {

// Existing ENCOS custom ranges, not a physical motor model.
inline constexpr uint16_t encos_model_profile_id = 0x8000;

}
#endif
