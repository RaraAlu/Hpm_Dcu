#include "motor_driver_binding.hpp"

#include <climits>
#include "cfg/motor_drivers.hpp"
#include "device_config.h"
#include "encos_driver.hpp"

namespace dcu::app {
namespace {
static_assert(motor::Arm == MC_AXIS_ARM && motor::Stop == MC_AXIS_STOP &&
              motor::ResetFault == MC_AXIS_RESET && motor::StartMotion == MC_AXIS_START_MOTION,
              "Legacy control intent mapping changed");

constexpr bool unique_profiles() noexcept
{
    for (const auto &entry : config::motor_drivers) {
        unsigned matches = 0;
        for (const auto &other : config::motor_drivers)
            matches += entry.model_profile_id == other.model_profile_id;
        if (matches != 1)
            return false;
    }
    return true;
}
static_assert(unique_profiles(), "Motor profile registrations must be unique");

motor::EncosMotorDriver encos_drivers[ENCOS_AXIS_COUNT];

const config::MotorDriverRegistration *registration(uint16_t profile) noexcept
{
    for (const auto &entry : config::motor_drivers)
        if (entry.model_profile_id == profile)
            return &entry;
    return nullptr;
}
}

ParameterDialect motor_parameter_dialect(uint16_t profile) noexcept
{
    const auto *entry = registration(profile);
    return entry && entry->kind == config::MotorDriverKind::Encos ?
        ParameterDialect::EncosLegacy : ParameterDialect::None;
}

bool validate_motor_configuration(const mc_routing_t &route, const mc_basic_t &basic,
                                  const mc_range_t &range, uint8_t format) noexcept
{
    const auto *entry = registration(route.ModelProfileId);
    return entry && entry->kind == config::MotorDriverKind::Encos &&
        motor::EncosMotorDriver::validate(route, basic, range, format);
}

MotorBinding configure_motor_driver(unsigned axis, const mc_routing_t &route,
                                    const mc_basic_t &basic, const mc_range_t &range,
                                    uint8_t format) noexcept
{
    if (axis >= ENCOS_AXIS_COUNT || !validate_motor_configuration(route, basic, range, format))
        return {};
    if (!encos_drivers[axis].configure(route, basic, range, format))
        return {};
    return {&encos_drivers[axis], ParameterDialect::EncosLegacy};
}

motor::Command motor_command_from_wire(const mc_axis_rx_t &wire) noexcept
{
    motor::Command command{};
    command.mode = static_cast<motor::ControlMode>(wire.Mode);
    command.control_bits = wire.ControlWord;
    command.reserved_flags = wire.Flags;
    command.position_urad = wire.TargetPosition;
    command.velocity_urad_s = wire.TargetVelocity;
    command.torque_mNm = wire.TargetTorque;
    command.current_cA = wire.TargetCurrent;
    command.current_limit_dA = wire.CurrentLimit;
    command.kp_centi = wire.Kp;
    command.kd_milli = wire.Kd;
    return command;
}

bool encode_motor_command(const motor::MotorDevice &device, const mc_axis_rx_t &wire,
                          motor::CanFrame &frame) noexcept
{
    return device.encode(motor_command_from_wire(wire), frame) == motor::Result::Ok;
}

bool decode_motor_feedback(const motor::MotorDevice &device, const motor::CanFrame &frame,
                           mc_axis_tx_t &wire) noexcept
{
    // The existing startup/ready logic requires all three fields in one snapshot.
    constexpr uint32_t required = motor::Position | motor::Velocity | motor::Current;
    motor::Feedback feedback{};
    if (device.decode(frame, feedback) != motor::Result::Ok ||
        (feedback.valid_fields & required) != required ||
        feedback.current_cA < INT16_MIN || feedback.current_cA > INT16_MAX ||
        feedback.motor_fault > UINT8_MAX)
        return false;
    wire.ActualPosition = feedback.position_urad;
    wire.ActualVelocity = feedback.velocity_urad_s;
    wire.ActualCurrent = static_cast<int16_t>(feedback.current_cA);
    wire.EstimatedTorque = feedback.torque_mNm;
    wire.MotorTemperature = feedback.motor_temperature_deciC;
    wire.MosTemperature = feedback.mos_temperature_deciC;
    wire.MotorFaultRaw = static_cast<uint8_t>(feedback.motor_fault);
    uint8_t valid = wire.ValidFlags & 64U;
    if (feedback.valid_fields & motor::Position) valid |= 1U;
    if (feedback.valid_fields & motor::Velocity) valid |= 2U;
    if (feedback.valid_fields & motor::Current) valid |= 4U;
    if (feedback.valid_fields & motor::Torque) valid |= 8U;
    if (feedback.valid_fields & motor::MotorTemperature) valid |= 16U;
    if (feedback.valid_fields & motor::MosTemperature) valid |= 32U;
    wire.ValidFlags = valid;
    return true;
}

}
