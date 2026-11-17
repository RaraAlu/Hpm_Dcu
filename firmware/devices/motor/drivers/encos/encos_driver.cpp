#include "encos_driver.hpp"
#include "encos_motor.hpp"

#include <math.h>

namespace dcu::motor {

bool EncosMotorDriver::validate(const mc_routing_t &r, const mc_basic_t &b,
                                const mc_range_t &range, uint8_t format) noexcept
{
    encos_profile_t p{};
    if (r.CanId < 1 || r.CanId > 8 || r.StopPolicy != 0 || r.FeedbackPolicy != 1 ||
        !encos_profile(&range, &b, &p))
        return false;
    return r.PositionLimitMin < r.PositionLimitMax &&
        r.PositionLimitMin >= p.pos[0] * 1e6 && r.PositionLimitMax <= p.pos[1] * 1e6 &&
        r.VelocityLimit && r.VelocityLimit <= fmin(-p.vel[0], p.vel[1]) * 1e6 &&
        r.CurrentLimit && r.CurrentLimit <= fmin(-p.current[0], p.current[1]) * 10 &&
        r.TorqueLimit && r.TorqueLimit <= fmin(-p.torque[0], p.torque[1]) * 1000 &&
        (!r.PowerQueryPeriodMs || (r.PowerQueryPeriodMs >= 20 && r.PowerQueryPeriodMs <= 1000)) &&
        b.CommunicationMode == format && format <= 1 && b.CanTimeoutMs <= 500;
}

bool EncosMotorDriver::configure(const mc_routing_t &route, const mc_basic_t &basic,
                                 const mc_range_t &range, uint8_t format) noexcept
{
    configured_ = false;
    capabilities_ = {};
    if (!validate(route, basic, range, format))
        return false;
    route_ = route;
    encos_profile(&range, &basic, &profile_);
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
    if (frame.id != route_.CanId || (frame.flags != 0 && frame.flags != 24))
        return Result::InvalidFrame;
    return encos::decode_feedback(frame, profile_, feedback) ? Result::Ok : Result::InvalidFrame;
}

}
