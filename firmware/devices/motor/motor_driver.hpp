#ifndef DCU_MOTOR_DRIVER_HPP
#define DCU_MOTOR_DRIVER_HPP

#include <cstdint>
#include "can_frame.h"

namespace dcu::motor {

using CanFrame = dcu_can_frame_t;

enum class Result : uint8_t {
    Ok, NotConfigured, Unsupported, InvalidCommand, InvalidFrame
};

enum class ControlMode : uint8_t {
    Position = 1, Velocity = 2, Current = 3, Impedance = 4, Torque = 5
};

enum ControlBit : uint16_t {
    Arm = 1U << 0, Stop = 1U << 1, ResetFault = 1U << 2, StartMotion = 1U << 3
};

constexpr uint32_t mode_bit(ControlMode mode) noexcept
{
    const auto value = static_cast<uint8_t>(mode);
    return value < 32 ? uint32_t{1} << value : 0;
}

enum FeedbackField : uint32_t {
    Position = 1U << 0, Velocity = 1U << 1, Current = 1U << 2,
    Torque = 1U << 3, MotorTemperature = 1U << 4, MosTemperature = 1U << 5
};

struct Command {
    ControlMode mode{};
    uint16_t control_bits{};
    uint8_t reserved_flags{};
    int32_t position_urad{};
    int32_t velocity_urad_s{};
    int32_t torque_mNm{};
    int32_t current_cA{};
    uint16_t current_limit_dA{};
    uint16_t kp_centi{};
    uint16_t kd_milli{};
};

struct Feedback {
    uint32_t valid_fields{};
    int32_t position_urad{};
    int32_t velocity_urad_s{};
    int32_t current_cA{};
    int32_t torque_mNm{};
    int16_t motor_temperature_deciC{};
    int16_t mos_temperature_deciC{};
    uint32_t motor_fault{};
};

struct Routing {
    uint16_t can_id{};
    int32_t position_min_urad{}, position_max_urad{};
    uint32_t velocity_limit_urad_s{}, torque_limit_mNm{};
    uint32_t current_limit_dA{};
};

struct Capabilities {
    uint32_t control_modes{};
    uint32_t feedback_fields{};
    double torque_constant_Nm_A{};
    bool active_stop_command{};
};

// Drivers encode/decode only. Bus arbitration and physical stop policy are separate.
class MotorDriver {
public:
    virtual const Capabilities &capabilities() const noexcept = 0;
    virtual Result encode(const Command &, CanFrame &) const noexcept = 0;
    virtual Result decode(const CanFrame &, Feedback &) const noexcept = 0;

protected:
    ~MotorDriver() = default;
};

class MotorDevice {
public:
    // The driver must outlive this device; production bindings use static storage.
    void bind(const MotorDriver &driver) noexcept { driver_ = &driver; }
    void reset() noexcept { driver_ = nullptr; }
    bool bound() const noexcept { return driver_ != nullptr; }
    const Capabilities &capabilities() const noexcept;
    Result encode(const Command &, CanFrame &) const noexcept;
    Result decode(const CanFrame &, Feedback &) const noexcept;

private:
    const MotorDriver *driver_{};
};

}
#endif
