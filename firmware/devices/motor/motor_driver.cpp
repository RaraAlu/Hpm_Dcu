#include "motor_driver.hpp"

namespace dcu::motor {
namespace {
bool valid_frame(const CanFrame &frame) noexcept
{
    constexpr unsigned flags = DCU_CAN_FLAG_EXTENDED | DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    return frame.dlc <= DCU_CAN_MAX_PAYLOAD && !(frame.flags & ~flags) &&
        (!(frame.flags & DCU_CAN_FLAG_BRS) || (frame.flags & DCU_CAN_FLAG_FD)) &&
        frame.id <= ((frame.flags & DCU_CAN_FLAG_EXTENDED) ? 0x1FFFFFFFU : 0x7FFU);
}
}

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
    if (!valid_frame(candidate))
        return Result::InvalidFrame;
    frame = candidate;
    return Result::Ok;
}

Result MotorDevice::decode(const CanFrame &frame, Feedback &feedback) const noexcept
{
    if (!driver_)
        return Result::NotConfigured;
    if (!valid_frame(frame))
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
