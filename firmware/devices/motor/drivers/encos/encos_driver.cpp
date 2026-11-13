#include "encos_driver.hpp"

#include <math.h>
#include <climits>

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
    if (command.current_cA < INT16_MIN || command.current_cA > INT16_MAX)
        return Result::InvalidCommand;
    mc_axis_rx_t wire{};
    wire.Mode = static_cast<uint8_t>(command.mode);
    wire.ControlWord = command.control_bits;
    wire.Flags = command.reserved_flags;
    wire.TargetPosition = command.position_urad;
    wire.TargetVelocity = command.velocity_urad_s;
    wire.TargetTorque = command.torque_mNm;
    wire.TargetCurrent = static_cast<int16_t>(command.current_cA);
    wire.CurrentLimit = command.current_limit_dA;
    wire.Kp = command.kp_centi;
    wire.Kd = command.kd_milli;
    CanFrame candidate{};
    if (!encos_encode(&wire, &route_, &profile_, &candidate))
        return Result::InvalidCommand;
    frame = candidate;
    return Result::Ok;
}

Result EncosMotorDriver::decode(const CanFrame &frame, Feedback &feedback) const noexcept
{
    if (!configured_)
        return Result::NotConfigured;
    if (frame.id != route_.CanId || (frame.flags != 0 && frame.flags != 24))
        return Result::InvalidFrame;
    mc_axis_tx_t wire{};
    if (!encos_feedback(&frame, &profile_, &wire))
        return Result::InvalidFrame;
    Feedback candidate{};
    candidate.valid_fields = Position | Velocity | Current | MotorTemperature | MosTemperature;
    if (wire.ValidFlags & 8U)
        candidate.valid_fields |= Torque;
    candidate.position_urad = wire.ActualPosition;
    candidate.velocity_urad_s = wire.ActualVelocity;
    candidate.current_cA = wire.ActualCurrent;
    candidate.torque_mNm = wire.EstimatedTorque;
    candidate.motor_temperature_deciC = wire.MotorTemperature;
    candidate.mos_temperature_deciC = wire.MosTemperature;
    candidate.motor_fault = wire.MotorFaultRaw;
    feedback = candidate;
    return Result::Ok;
}

}
