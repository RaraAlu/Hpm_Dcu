#ifndef DCU_BJCX_CONTROL_HPP
#define DCU_BJCX_CONTROL_HPP

#include "motor_driver.hpp"

namespace dcu::bjcx {

// CiH408 wire values, not EtherCAT axis units or model-specific limits.
struct MitWireCommand {
    uint16_t kp_deci{}, kd_deci{};
    float position_deg{}, velocity_rpm{};
    int16_t feedforward_raw{};
    uint8_t counter{};
};

bool encode_enable(uint16_t can_id, bool enabled, motor::CanFrame &) noexcept;
bool matches_enable_reply(uint16_t can_id, bool enabled, const motor::CanFrame &) noexcept;
bool encode_mit(uint16_t can_id, const MitWireCommand &, motor::CanFrame &) noexcept;

}
#endif
