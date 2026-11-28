#include "encos_driver.hpp"
#include "encos_motor.hpp"
#include "motor_limits.hpp"

#include <math.h>

namespace dcu::motor {

void EncosMotorDriver::configure_queries(const Routing &route) noexcept
{
    // Parameter replies carry physical values; MIT frames require a verified model encoding.
    configured_ = false;
    centered_mit_zero_ = false;
    route_ = route;
    profile_ = {};
    capabilities_ = {};
    capabilities_.legacy_parameter_queries = true;
    capabilities_.feedback_fields = Position | Velocity | Current;
}

bool EncosMotorDriver::validate(const Routing &r, const encos_settings_t &settings,
                                uint8_t format) noexcept
{
    const auto &p = settings.profile;
    if (r.can_id < 1 || r.can_id > 8 || !encos_profile_valid(&p))
        return false;
    return r.position_min_urad < r.position_max_urad &&
        r.position_min_urad / 1e6 >= p.pos[0] && r.position_max_urad / 1e6 <= p.pos[1] &&
        r.velocity_limit_urad_s && r.velocity_limit_urad_s / 1e6 <= fmin(-p.vel[0], p.vel[1]) &&
        r.current_limit_dA && r.current_limit_dA / 10.0 <= fmin(-p.current[0], p.current[1]) &&
        r.torque_limit_mNm && r.torque_limit_mNm / 1000.0 <= fmin(-p.torque[0], p.torque[1]) &&
        settings.frame_format == format && format <= 1 && settings.timeout_ms && settings.timeout_ms <= 500;
}

bool EncosMotorDriver::configure_impedance(const Routing &route, const encos_config::Model &model) noexcept
{
    configure_queries(route);
    if (!model.centered_mit_zero || !model.ack1.known() || route.can_id < 1 || route.can_id > 8 ||
        route.position_min_urad || route.position_max_urad || route.velocity_limit_urad_s ||
        route.torque_limit_mNm || !route.current_limit_dA || route.current_limit_dA > model.ack1.current_A * 10)
        return false;
    const auto &a = model.ack1;
    profile_.pos[0] = -(profile_.pos[1] = a.position_rad);
    profile_.vel[0] = -(profile_.vel[1] = a.velocity_rad_s);
    profile_.current[0] = -(profile_.current[1] = a.current_A);
    capabilities_.control_modes = mode_bit(ControlMode::Impedance);
    capabilities_.feedback_fields |= MotorTemperature | MosTemperature;
    centered_mit_zero_ = configured_ = true;
    return true;
}

bool EncosMotorDriver::configure(const Routing &route, const encos_settings_t &settings,
                                 uint8_t format, bool centered_mit_zero) noexcept
{
    configured_ = false;
    capabilities_ = {};
    if (!validate(route, settings, format))
        return false;
    route_ = route;
    profile_ = settings.profile;
    centered_mit_zero_ = centered_mit_zero && profile_.kp[0] == 0 && profile_.kd[0] == 0 &&
        profile_.pos[0] == -profile_.pos[1] && profile_.vel[0] == -profile_.vel[1] &&
        profile_.torque[0] == -profile_.torque[1];
    capabilities_.control_modes = mode_bit(ControlMode::Position) | mode_bit(ControlMode::Velocity) |
        mode_bit(ControlMode::Current) | mode_bit(ControlMode::Impedance) | mode_bit(ControlMode::Torque);
    capabilities_.feedback_fields = Position | Velocity | Current | MotorTemperature | MosTemperature;
    if (profile_.kt > 0)
        capabilities_.feedback_fields |= Torque;
    capabilities_.torque_constant_Nm_A = profile_.kt;
    capabilities_.legacy_parameter_queries = true;
    configured_ = true;
    return true;
}

Result EncosMotorDriver::encode(const Command &command, CanFrame &frame) const noexcept
{
    if (!configured_)
        return Result::NotConfigured;
    if (centered_mit_zero_ && is_zero_mit(command))
        return encos::encode_centered_zero(command, route_, frame) ? Result::Ok : Result::InvalidCommand;
    if (capabilities_.control_modes == mode_bit(ControlMode::Impedance))
        return Result::InvalidCommand;
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
