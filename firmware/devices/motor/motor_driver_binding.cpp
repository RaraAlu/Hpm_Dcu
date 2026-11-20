#include "motor_driver_binding.hpp"

#include "cfg/motor_drivers.hpp"
#include "device_config.h"
#include "encos_driver.hpp"
#include "bjcx_driver.hpp"

namespace dcu::motor {
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

EncosMotorDriver encos_drivers[DCU_MOTOR_DEVICE_CAPACITY];
BjcxMotorDriver bjcx_drivers[DCU_MOTOR_DEVICE_CAPACITY];

const config::MotorDriverRegistration *registration(uint16_t profile) noexcept
{
    for (const auto &entry : config::motor_drivers)
        if (entry.model_profile_id == profile)
            return &entry;
    return nullptr;
}
}

bool validate_motor_configuration(const MotorConfiguration &config, uint8_t format) noexcept
{
    const auto *entry = registration(config.model_profile_id);
    if (!entry) return false;
    if (entry->kind == config::MotorDriverKind::Bjcx)
        return format == 1 && !config.power_query_period_ms &&
            BjcxMotorDriver::validate(config.route.can_id, config.status_poll_period_ms,
                                       config.status_reply_timeout_ms);
    return !config.status_poll_period_ms && !config.status_reply_timeout_ms &&
        (!config.power_query_period_ms || (config.power_query_period_ms >= 20 && config.power_query_period_ms <= 1000)) &&
        EncosMotorDriver::validate(config.route, config.encos, format);
}

MotorSessionDriver *configure_motor_driver(unsigned device, const MotorConfiguration &config, uint8_t format) noexcept
{
    if (device >= DCU_MOTOR_DEVICE_CAPACITY || config.bus >= DCU_DEVICE_CAN_BUS_COUNT ||
        !validate_motor_configuration(config, format))
        return nullptr;
    if (registration(config.model_profile_id)->kind == config::MotorDriverKind::Bjcx) {
        auto &driver = bjcx_drivers[device];
        return driver.configure(config.route.can_id, config.status_poll_period_ms,
                                config.status_reply_timeout_ms) ? &driver : nullptr;
    }
    auto &driver = encos_drivers[device];
    if (!driver.configure(config.route, config.encos, format)) return nullptr;
    driver.bind_session(device, config.bus, format, config.power_query_period_ms);
    return &driver;
}

}
