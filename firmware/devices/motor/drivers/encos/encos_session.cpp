#include "encos_driver.hpp"
#include "encos_parameter_transactions.hpp"
#include "cfg/encos_runtime.hpp"
#include <cmath>

namespace dcu::motor {
namespace parameters = dcu::motor::encos_parameters;
using namespace encos_runtime;

namespace {
uint16_t age_ms(uint32_t elapsed, bool seen)
{
    return !seen || elapsed > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(elapsed);
}
uint32_t age_us(uint32_t elapsed, bool seen)
{
    return !seen || elapsed > UINT32_MAX / 1000 ? UINT32_MAX : elapsed * 1000;
}
}

void EncosMotorDriver::bind_session(unsigned device, unsigned bus, uint8_t format, uint16_t power_period)
    noexcept
{
    queries_ = {};
    queries_.device = device;
    queries_.bus = bus;
    queries_.frame_format = format;
    queries_.power_period_ms = power_period;
}

bool EncosMotorDriver::query_ready(uint32_t now) const
{
    const auto sample = query_sample(now);
    return sample.online && (sample.valid_fields & (Position | Velocity | Current)) ==
           (Position | Velocity | Current);
}

bool EncosMotorDriver::prepare_start(const Command &command, uint32_t now, Command &first)
{
    if (!query_ready(now) || std::fabs(double(queries_.velocity)) > 20000.0 ||
        ((command.mode == ControlMode::Position || command.mode == ControlMode::Impedance) &&
         std::fabs(double(command.position_urad) - queries_.position) > 100000.0))
        return false;

    parameters::abort_internal(queries_.bus);
    first = {};
    first.control_bits = Arm;
    first.mode = ControlMode::Position;
    first.position_urad = queries_.position;
    first.current_limit_dA = route_.current_limit_dA < 4095 ? route_.current_limit_dA : 4095;
    CanFrame frame{};
    return encode(first, frame) == Result::Ok;
}

bool EncosMotorDriver::start_query(uint32_t now, bool active)
{
    auto &q = queries_;
    for (unsigned k = active ? 3u : 0u; k < 4; ++k) {
        const auto period = k == 3 ? q.power_period_ms : query_period_ms[k];
        if (!period || ((q.requested_mask & (1u << k)) && now - q.request_ms[k] < period))
            continue;
        if (!parameters::start_query(q.bus, q.device, k + 1, 1, now, ParameterDialect::EncosLegacy,
                                     route_.can_id, q.frame_format))
            return false;
        q.request_ms[k] = now;
        q.requested_mask |= 1u << k;
        return true;
    }
    return false;
}

ObservationResult EncosMotorDriver::observe(const CanFrame &frame, uint32_t stamp, uint32_t now,
                                           bool active, Feedback &sample, int32_t &position, int32_t &velocity)
{
    auto &q = queries_;
    const auto observation = parameters::consume(q.bus, q.device, frame, now);
    if (observation.acknowledged) return ObservationResult::Acknowledged;
    if (!observation.valid) return ObservationResult::Invalid;

    switch (observation.field) {
    case parameters::Field::Position:
        position = q.position = observation.value;
        q.position_ms = stamp;
        q.position_seen = true;
        if (observation.update_actual && !active) sample.position_urad = observation.value;
        break;
    case parameters::Field::Velocity:
        velocity = q.velocity = observation.value;
        q.velocity_ms = stamp;
        q.velocity_seen = true;
        if (observation.update_actual && !active) sample.velocity_urad_s = observation.value;
        break;
    case parameters::Field::Current:
        q.current_ms = stamp;
        q.current_seen = true;
        if (!active) {
            sample.current_cA = static_cast<int16_t>(observation.value);
            const double torque = double(sample.current_cA) * capabilities_.torque_constant_Nm_A * 10;
            if (capabilities_.torque_constant_Nm_A > 0 && torque >= INT32_MIN && torque <= INT32_MAX)
                sample.torque_mNm = static_cast<int32_t>(torque);
        }
        break;
    case parameters::Field::Power:
        q.power_mW = observation.value;
        q.power_ms = stamp;
        q.power_seen = q.new_power = true;
        break;
    case parameters::Field::None:
        break;
    }
    q.online_ms = stamp;
    q.online_seen = true;
    const auto type = frame.data[0] >> 5;
    if (type >= 1 && type <= 5) sample.motor_fault = frame.data[0] & 31;
    return ObservationResult::Valid;
}

void EncosMotorDriver::observe_cyclic(uint32_t stamp, bool first)
{
    auto &q = queries_;
    if (first) q.position_seen = q.velocity_seen = q.current_seen = false;
    q.online_seen = true;
    q.online_ms = stamp;
}

QuerySample EncosMotorDriver::query_sample(uint32_t now) const
{
    const auto &q = queries_;
    const auto pos = now - q.position_ms, vel = now - q.velocity_ms, cur = now - q.current_ms;
    QuerySample sample{};
    sample.online = q.online_seen && now - q.online_ms <= query_online_ms;
    if (q.position_seen && pos <= query_position_ms) sample.valid_fields |= Position;
    if (q.velocity_seen && vel <= query_velocity_ms) sample.valid_fields |= Velocity;
    if (q.current_seen && cur <= query_current_ms) sample.valid_fields |= Current;
    if (sample.valid_fields == (Position | Velocity | Current)) {
        auto oldest = pos > vel ? pos : vel;
        if (cur > oldest) oldest = cur;
        sample.feedback_age_ms = age_ms(oldest, true);
    }
    sample.position_age_us = age_us(pos, q.position_seen);
    sample.velocity_age_us = age_us(vel, q.velocity_seen);
    sample.has_motion_data = q.position_seen || q.velocity_seen || q.current_seen;
    sample.power_mW = q.power_mW;
    sample.power_age_ms = age_ms(now - q.power_ms, q.power_seen);
    sample.power_valid = q.power_period_ms && q.power_seen && now - q.power_ms < q.power_period_ms * 3u;
    sample.new_power = q.new_power;
    return sample;
}

void EncosMotorDriver::acknowledge_queries() { queries_.new_power = false; }

}
