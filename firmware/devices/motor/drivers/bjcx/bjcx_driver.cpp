#include "bjcx_driver.hpp"
#include <cmath>
#include <cstring>
#include <limits>

namespace dcu::motor {
namespace {
constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
constexpr uint32_t status_fields = Position | Velocity | Current | MotorTemperature | MosTemperature;
constexpr double pi = 3.14159265358979323846;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);

bool decode_scaled(const uint8_t *data, double scale, int32_t &value)
{
    const uint32_t bits = (uint32_t{data[0]} << 24) | (uint32_t{data[1]} << 16) |
                          (uint32_t{data[2]} << 8) | data[3];
    float wire;
    std::memcpy(&wire, &bits, sizeof(wire));
    const double converted = double(wire) * scale;
    if (!std::isfinite(converted) || converted < INT32_MIN || converted > INT32_MAX) return false;
    value = static_cast<int32_t>(::round(converted));
    return true;
}
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
    poll = {};
    poll.frame.id = can_id_;
    poll.frame.flags = fd_brs;
    poll.frame.dlc = 1;
    poll.frame.data[0] = 0x17;
    poll.timeout_ms = timeout_ms_;
    poll.reply_bytes = 16;
    requested_ = true;
    requested_ms_ = now;
    return true;
}

Result BjcxMotorDriver::decode(const CanFrame &frame, Feedback &out) const noexcept
{
    if (!can_id_) return Result::NotConfigured;
    if (frame.id != can_id_ || frame.flags != fd_brs || frame.dlc != 10 || frame.data[0] != 0x80)
        return Result::InvalidFrame;
    Feedback sample{};
    const auto *data = frame.data;
    // Invalid position/velocity must not hide an independent motor alarm.
    sample.motor_fault = (uint32_t{data[1] & 0x0fU} << 8) | data[2];
    if (data[1] >> 4 <= 5) {
        if (decode_scaled(data + 3, pi * 1e6 / 180, sample.position_urad)) sample.valid_fields |= Position;
        if (decode_scaled(data + 7, pi * 1e6 / 30, sample.velocity_urad_s)) sample.valid_fields |= Velocity;
    }
    const int32_t current = (int32_t{data[11]} << 8) | data[12];
    sample.current_cA = current < 32768 ? current : current - 65536;
    sample.motor_temperature_deciC = (int(data[13]) - 50) * 10;
    sample.mos_temperature_deciC = (int(data[14]) - 50) * 10;
    sample.valid_fields |= Current | MotorTemperature | MosTemperature;
    out = sample;
    return Result::Ok;
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
