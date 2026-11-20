#ifndef DCU_MOTOR_DRIVERS_CONFIG_HPP
#define DCU_MOTOR_DRIVERS_CONFIG_HPP

#include <cstdint>

namespace dcu::motor::config {

enum class MotorDriverKind : uint8_t { Encos, Bjcx };
struct MotorDriverRegistration {
    uint16_t model_profile_id;
    MotorDriverKind kind;
};

// Existing reviewed custom ranges; this is not an inferred physical motor model.
inline constexpr MotorDriverRegistration motor_drivers[] = {
    {0x8000, MotorDriverKind::Encos},
    {0x8101, MotorDriverKind::Bjcx}, // HRA40H-101
    {0x8102, MotorDriverKind::Bjcx}, // HRA55H-101
};

}
#endif
