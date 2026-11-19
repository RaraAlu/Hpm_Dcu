#ifndef DCU_MOTOR_LIMITS_HPP
#define DCU_MOTOR_LIMITS_HPP

#include "motor_driver.hpp"

namespace dcu::motor {

constexpr bool within_magnitude(int32_t value, int64_t limit) noexcept
{
    return value >= -limit && value <= limit;
}

constexpr bool within_limits(const Command &command, const Routing &route) noexcept
{
    const bool position_mode = command.mode == ControlMode::Position || command.mode == ControlMode::Impedance;
    return (!position_mode || (command.position_urad >= route.position_min_urad &&
                               command.position_urad <= route.position_max_urad)) &&
           within_magnitude(command.velocity_urad_s, route.velocity_limit_urad_s) &&
           within_magnitude(command.torque_mNm, route.torque_limit_mNm) &&
           within_magnitude(command.current_cA, int64_t{route.current_limit_dA} * 10) &&
           command.current_limit_dA <= route.current_limit_dA;
}

}
#endif
