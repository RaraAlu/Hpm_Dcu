#ifndef DCU_MOTOR_DRIVER_BINDING_HPP
#define DCU_MOTOR_DRIVER_BINDING_HPP

#include "motor_driver.hpp"
#include "encos_interface.h"

namespace dcu::app {

enum class ParameterDialect : uint8_t { None, EncosLegacy };
struct MotorBinding {
    const motor::MotorDriver *driver{};
    ParameterDialect parameter_dialect{};
};

ParameterDialect motor_parameter_dialect(uint16_t model_profile_id) noexcept;
bool validate_motor_configuration(const mc_routing_t &, const mc_basic_t &,
                                  const mc_range_t &, uint8_t frame_format) noexcept;
MotorBinding configure_motor_driver(unsigned axis, const mc_routing_t &, const mc_basic_t &,
                                    const mc_range_t &, uint8_t frame_format) noexcept;
motor::Command motor_command_from_wire(const mc_axis_rx_t &) noexcept;
bool encode_motor_command(const motor::MotorDevice &, const mc_axis_rx_t &,
                          motor::CanFrame &) noexcept;
bool decode_motor_feedback(const motor::MotorDevice &, const motor::CanFrame &,
                           mc_axis_tx_t &) noexcept;

}
#endif
