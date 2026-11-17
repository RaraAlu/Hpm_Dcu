#include "motor_driver_binding.hpp"

#include "cfg/motor_drivers.hpp"
#include "device_config.h"
#include "encos_driver.hpp"

namespace dcu::app {
namespace {
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

bool validate_motor_configuration(const AxisConfiguration &config, uint8_t format) noexcept
{
    const auto *entry = registration(config.model_profile_id);
    return entry && entry->kind == config::MotorDriverKind::Encos &&
        config.stop_policy == 0 && config.feedback_policy == 1 &&
        (!config.power_query_period_ms || (config.power_query_period_ms >= 20 && config.power_query_period_ms <= 1000)) &&
        motor::EncosMotorDriver::validate(config.route, config.encos, format);
}

MotorBinding configure_motor_driver(unsigned axis, const AxisConfiguration &config, uint8_t format) noexcept
{
    if (axis >= ENCOS_AXIS_COUNT || !validate_motor_configuration(config, format))
        return {};
    if (!encos_drivers[axis].configure(config.route, config.encos, format))
        return {};
    return {&encos_drivers[axis], ParameterDialect::EncosLegacy};
}

}
