#include "bjcx_driver.hpp"
#include "bjcx_status.hpp"

namespace dcu::motor {
namespace {
constexpr uint32_t status_fields = Position | Velocity | Current | MotorTemperature | MosTemperature;
}

bool BjcxMotorDriver::validate(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms) noexcept
{
    return can_id && can_id <= 0x3ff && timeout_ms >= 2 && timeout_ms <= 20 &&
           period_ms >= 2U * timeout_ms && period_ms <= 1000;
}

bool BjcxMotorDriver::configure(uint16_t can_id, uint16_t period_ms, uint16_t timeout_ms) noexcept
{
    if (!validate(can_id, period_ms, timeout_ms)) return false;
    can_id_ = can_id;
    period_ms_ = period_ms;
    timeout_ms_ = timeout_ms;
    requested_ = received_ = false;
    valid_fields_ = 0;
    capabilities_.feedback_fields = status_fields;
    return true;
}

bool BjcxMotorDriver::prepare_status_poll(uint32_t now, StatusPoll &poll)
{
    if (!can_id_ || (requested_ && now - requested_ms_ < period_ms_)) return false;
    poll = {bjcx::status_request(can_id_), timeout_ms_, bjcx::status_reply_bytes};
    requested_ = true;
    requested_ms_ = now;
    return true;
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
    received_ms_ = stamp;
    valid_fields_ = sample.valid_fields;
    if (valid_fields_ & Position) position = sample.position_urad;
    if (valid_fields_ & Velocity) velocity = sample.velocity_urad_s;
    return true;
}

QuerySample BjcxMotorDriver::query_sample(uint32_t now) const
{
    QuerySample snapshot{};
    const uint32_t age = now - received_ms_;
    snapshot.online = received_ && age < period_ms_ + timeout_ms_;
    if (!snapshot.online) return snapshot;
    snapshot.valid_fields = valid_fields_;
    snapshot.has_motion_data = (valid_fields_ & (Position | Velocity | Current)) != 0;
    if (valid_fields_ & Position) snapshot.position_age_us = age * 1000;
    if (valid_fields_ & Velocity) snapshot.velocity_age_us = age * 1000;
    snapshot.feedback_age_ms = static_cast<uint16_t>(age);
    return snapshot;
}

}
