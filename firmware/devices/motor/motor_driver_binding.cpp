#include "motor_driver_binding.hpp"

#include "cfg/motor_drivers.hpp"
#include "device_config.h"
#include "encos_driver.hpp"
#include "bjcx_driver.hpp"

namespace dcu::motor {
namespace {
EncosMotorDriver encos_drivers[DCU_MOTOR_DEVICE_CAPACITY];
BjcxMotorDriver bjcx_drivers[DCU_MOTOR_DEVICE_CAPACITY];
}

bool validate_motor_configuration(const MotorConfiguration &config, uint8_t format) noexcept
{
    switch (config.protocol) {
    case DCU_MOTOR_PROTOCOL_BJCX:
        return config.model_profile_id == 0 && format == 1 && !config.power_query_period_ms &&
            BjcxMotorDriver::validate(config.route.can_id, config.status_poll_period_ms,
                                      config.status_reply_timeout_ms, config.neutral_mit_only);
    case DCU_MOTOR_PROTOCOL_ENCOS:
        return config.model_profile_id == config::encos_model_profile_id && !config.neutral_mit_only &&
            !config.status_poll_period_ms && !config.status_reply_timeout_ms &&
            (!config.power_query_period_ms || (config.power_query_period_ms >= 20 && config.power_query_period_ms <= 1000)) &&
            EncosMotorDriver::validate(config.route, config.encos, format);
    default:
        return false;
    }
}

MotorSessionDriver *configure_motor_driver(unsigned device, const MotorConfiguration &config, uint8_t format) noexcept
{
    if (device >= DCU_MOTOR_DEVICE_CAPACITY || config.bus >= DCU_DEVICE_CAN_BUS_COUNT ||
        !validate_motor_configuration(config, format))
        return nullptr;
    if (config.protocol == DCU_MOTOR_PROTOCOL_BJCX) {
        auto &driver = bjcx_drivers[device];
        return driver.configure(config.route.can_id, config.status_poll_period_ms,
                                config.status_reply_timeout_ms, config.neutral_mit_only) ? &driver : nullptr;
    }
    auto &driver = encos_drivers[device];
    if (!driver.configure(config.route, config.encos, format)) return nullptr;
    driver.bind_session(device, config.bus, format, config.power_query_period_ms);
    return &driver;
}

}
