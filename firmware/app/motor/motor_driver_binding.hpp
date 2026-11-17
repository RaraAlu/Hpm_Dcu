#ifndef DCU_MOTOR_DRIVER_BINDING_HPP
#define DCU_MOTOR_DRIVER_BINDING_HPP

#include "axis_contract.hpp"
#include "parameter_dialect.hpp"

namespace dcu::app {

struct MotorBinding {
    const motor::MotorDriver *driver{};
    ParameterDialect parameter_dialect{};
};

ParameterDialect motor_parameter_dialect(uint16_t model_profile_id) noexcept;
bool validate_motor_configuration(const AxisConfiguration &, uint8_t frame_format) noexcept;
MotorBinding configure_motor_driver(unsigned axis, const AxisConfiguration &, uint8_t frame_format) noexcept;

}
#endif
