#ifndef DCU_ENCOS_MOTOR_HPP
#define DCU_ENCOS_MOTOR_HPP

#include "encos_motor.h"
#include "motor_driver.hpp"

namespace dcu::encos {

// Use validated routing/profile limits. Failed conversions leave output unchanged.
// CAN identity and transport flags are checked by the device driver.
bool encode_motion(const motor::Command &, const mc_routing_t &,
                   const encos_profile_t &, motor::CanFrame &) noexcept;
bool decode_feedback(const motor::CanFrame &, const encos_profile_t &,
                     motor::Feedback &) noexcept;

}
#endif
