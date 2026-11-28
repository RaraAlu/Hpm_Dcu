#ifndef DCU_ENCOS_MOTOR_HPP
#define DCU_ENCOS_MOTOR_HPP

#include "encos_motor.h"
#include "motor_driver.hpp"

namespace dcu::encos {

// Only for models with the lower-midpoint wire convention. Does not use feedback scales.
bool encode_centered_zero(const motor::Command &, const motor::Routing &, motor::CanFrame &) noexcept;

// Use validated routing/profile limits. Failed conversions leave output unchanged.
// CAN identity and transport flags are checked by the device driver.
bool encode_motion(const motor::Command &, const motor::Routing &,
                   const encos_profile_t &, motor::CanFrame &) noexcept;
bool decode_feedback(const motor::CanFrame &, const encos_profile_t &,
                     motor::Feedback &) noexcept;

}
#endif
