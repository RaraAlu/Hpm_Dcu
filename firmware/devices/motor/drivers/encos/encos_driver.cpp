#include "encos_driver.hpp"
#include "encos_motor.hpp"

#include <math.h>

namespace dcu::motor {

bool EncosMotorDriver::validate(const Routing &r, const encos_settings_t &settings,
                                uint8_t format) noexcept
{
    const auto &p = settings.profile;
    if (r.can_id < 1 || r.can_id > 8 || !encos_profile_valid(&p))
        return false;
    return r.position_min_urad < r.position_max_urad &&
        r.position_min_urad >= p.pos[0] * 1e6 && r.position_max_urad <= p.pos[1] * 1e6 &&
        r.velocity_limit_urad_s && r.velocity_limit_urad_s <= fmin(-p.vel[0], p.vel[1]) * 1e6 &&
        r.current_limit_dA && r.current_limit_dA <= fmin(-p.current[0], p.current[1]) * 10 &&
        r.torque_limit_mNm && r.torque_limit_mNm <= fmin(-p.torque[0], p.torque[1]) * 1000 &&
        settings.frame_format == format && format <= 1 && settings.timeout_ms && settings.timeout_ms <= 500;
}

bool EncosMotorDriver::configure(const Routing &route, const encos_settings_t &settings,
                                 uint8_t format) noexcept
{
    configured_ = false;
    capabilities_ = {};
    if (!validate(route, settings, format))
        return false;
    route_ = route;
    profile_ = settings.profile;
    capabilities_.control_modes = mode_bit(ControlMode::Position) | mode_bit(ControlMode::Velocity) |
        mode_bit(ControlMode::Current) | mode_bit(ControlMode::Impedance) | mode_bit(ControlMode::Torque);
    capabilities_.feedback_fields = Position | Velocity | Current | MotorTemperature | MosTemperature;
    if (profile_.kt > 0)
        capabilities_.feedback_fields |= Torque;
    capabilities_.torque_constant_Nm_A = profile_.kt;
    configured_ = true;
    return true;
}

Result EncosMotorDriver::encode(const Command &command, CanFrame &frame) const noexcept
{
    if (!configured_)
        return Result::NotConfigured;
    return encos::encode_motion(command, route_, profile_, frame) ? Result::Ok : Result::InvalidCommand;
}

Result EncosMotorDriver::decode(const CanFrame &frame, Feedback &feedback) const noexcept
{
    if (!configured_)
        return Result::NotConfigured;
    if (frame.id != route_.can_id || (frame.flags != 0 && frame.flags != 24))
        return Result::InvalidFrame;
    return encos::decode_feedback(frame, profile_, feedback) ? Result::Ok : Result::InvalidFrame;
}

}
