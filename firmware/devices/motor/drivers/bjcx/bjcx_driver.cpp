#include "bjcx_driver.hpp"
#include "bjcx_status.hpp"
#include "bjcx_control.hpp"
#include "motor_limits.hpp"

namespace dcu::motor {
namespace {
constexpr uint32_t status_fields = Position | Velocity | Current | MotorTemperature | MosTemperature;
}

bool BjcxMotorDriver::validate(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms, bool neutral_mit) noexcept
{
    if (!can_id || can_id > 0x3ff) return false;
    return neutral_mit ? !period_ms && !timeout_ms :
           timeout_ms >= 2 && timeout_ms <= 20 && period_ms >= 2U * timeout_ms && period_ms <= 1000;
}

bool BjcxMotorDriver::configure(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms, bool neutral_mit,
                                bool cyclic_mit) noexcept
{
    if ((cyclic_mit && neutral_mit) || !validate(can_id, period_ms, timeout_ms, neutral_mit)) return false;
    can_id_ = can_id;
    period_ms_ = period_ms;
    timeout_ms_ = timeout_ms;
    neutral_mit_ = neutral_mit;
    cyclic_mit_ = cyclic_mit;
    counter_ = 0;
    requested_ = received_ = waiting_for_status_ = ambiguous_status_ = false;
    valid_fields_ = 0;
    capabilities_.feedback_fields = status_fields;
    capabilities_.control_modes = cyclic_mit ? mode_bit(ControlMode::Impedance) : 0;
    return true;
}

Result BjcxMotorDriver::encode(const Command &command, CanFrame &frame) const noexcept
{
    if (!cyclic_mit_) return Result::Unsupported;
    if (!is_zero_mit(command) || (command.control_bits & ~(Arm | StartMotion))) return Result::InvalidCommand;
    bjcx::MitWireCommand wire{};
    wire.counter = counter_;
    return bjcx::encode_mit(can_id_, wire, frame) ? Result::Ok : Result::InvalidCommand;
}

bool BjcxMotorDriver::query_ready(uint32_t now) const
{
    constexpr uint32_t required = Position | Velocity | Current;
    const auto sample = query_sample(now);
    return cyclic_mit_ && !waiting_for_status_ && !ambiguous_status_ && sample.online &&
           (sample.valid_fields & required) == required;
}

bool BjcxMotorDriver::prepare_start(const Command &cmd, uint32_t now, Command &first)
{
    CanFrame frame{};
    if (!query_ready(now) || !within_magnitude(velocity_urad_s_, 20000) || encode(cmd, frame) != Result::Ok)
        return false;
    first = cmd;
    return true;
}

bool BjcxMotorDriver::encode_neutral_mit(CanFrame &frame, uint8_t counter) const
{
    bjcx::MitWireCommand command{};
    command.counter = counter;
    return neutral_mit_ && bjcx::encode_mit(can_id_, command, frame);
}

bool BjcxMotorDriver::prepare_status_poll(uint32_t now, StatusPoll &poll) const
{
    if (!can_id_ || neutral_mit_ || (requested_ && now - requested_ms_ < period_ms_)) return false;
    poll = {bjcx::status_request(can_id_), timeout_ms_, bjcx::status_reply_bytes};
    return true;
}

void BjcxMotorDriver::status_poll_submitted(uint32_t now)
{
    requested_ = true;
    // A late 0x80 has no echoed request number. Do not use it to authorize a control exchange.
    ambiguous_status_ |= waiting_for_status_;
    waiting_for_status_ = true;
    requested_ms_ = now;
}

Result BjcxMotorDriver::decode(const CanFrame &frame, Feedback &out) const noexcept
{
    if (!can_id_) return Result::NotConfigured;
    return bjcx::decode_status_reply(can_id_, frame, out) ? Result::Ok : Result::InvalidFrame;
}

uint8_t BjcxMotorDriver::max_rx_bytes() const
{
    return bjcx::status_reply_bytes;
}

bool BjcxMotorDriver::observe_status_reply(const CanFrame &frame, uint32_t stamp, Feedback &sample,
                                             int32_t &position, int32_t &velocity)
{
    if (decode(frame, sample) != Result::Ok) return false;
    received_ = true;
    waiting_for_status_ = false;
    received_ms_ = stamp;
    valid_fields_ = sample.valid_fields;
    if (valid_fields_ & Position) position = sample.position_urad;
    if (valid_fields_ & Velocity) velocity = velocity_urad_s_ = sample.velocity_urad_s;
    return true;
}

QuerySample BjcxMotorDriver::query_sample(uint32_t now) const
{
    QuerySample snapshot{};
    const uint32_t age = now - received_ms_;
    snapshot.online = received_ && age < (neutral_mit_ ? 10U : period_ms_ + timeout_ms_);
    if (!snapshot.online) return snapshot;
    snapshot.valid_fields = valid_fields_;
    snapshot.has_motion_data = (valid_fields_ & (Position | Velocity | Current)) != 0;
    if (valid_fields_ & Position) snapshot.position_age_us = age * 1000;
    if (valid_fields_ & Velocity) snapshot.velocity_age_us = age * 1000;
    snapshot.feedback_age_ms = static_cast<uint16_t>(age);
    return snapshot;
}

}
