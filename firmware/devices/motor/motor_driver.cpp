#include "motor_driver.hpp"

namespace dcu::motor {

const Capabilities &MotorDevice::capabilities() const noexcept
{
    static constexpr Capabilities unavailable{};
    return driver_ ? driver_->capabilities() : unavailable;
}

Result MotorDevice::encode(const Command &command, CanFrame &frame) const noexcept
{
    if (!driver_)
        return Result::NotConfigured;
    if (!(capabilities().control_modes & mode_bit(command.mode)))
        return Result::Unsupported;
    CanFrame candidate{};
    const auto result = driver_->encode(command, candidate);
    if (result != Result::Ok)
        return result;
    if (!dcu_can_frame_valid(&candidate))
        return Result::InvalidFrame;
    frame = candidate;
    return Result::Ok;
}

Result MotorDevice::decode(const CanFrame &frame, Feedback &feedback) const noexcept
{
    if (!driver_)
        return Result::NotConfigured;
    if (!dcu_can_frame_valid(&frame))
        return Result::InvalidFrame;
    Feedback candidate{};
    const auto result = driver_->decode(frame, candidate);
    if (result != Result::Ok)
        return result;
    constexpr uint32_t known = Position | Velocity | Current | Torque | MotorTemperature | MosTemperature;
    if (!candidate.valid_fields || (candidate.valid_fields & ~(capabilities().feedback_fields & known)))
        return Result::InvalidFrame;
    feedback = candidate;
    return Result::Ok;
}

}
