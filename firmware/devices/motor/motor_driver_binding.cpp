#include "motor_driver_binding.hpp"

#include "drivers/bjcx/cfg/bjcx_models.h"
#include "drivers/encos/cfg/encos_models.h"
#include "device_config.h"
#include "encos_driver.hpp"
#include "bjcx_driver.hpp"

namespace dcu::motor {
namespace {
EncosMotorDriver encos_drivers[DCU_MOTOR_DEVICE_CAPACITY];
BjcxMotorDriver bjcx_drivers[DCU_MOTOR_DEVICE_CAPACITY];

bool matches_encos_model(const MotorConfiguration &motor, uint8_t format) noexcept
{
    const auto *model = motor.type ? encos_config::find_model(motor.type) : nullptr;
    if (!model || format != 1 || !model->mit.verified()) return false;
    const auto &p = motor.encos.profile;
    const auto &r = model->mit;
    return p.kt == r.torque_constant_Nm_per_A &&
        p.vel[0] == -r.max_speed_rad_s && p.vel[1] == r.max_speed_rad_s &&
        p.torque[0] == -r.max_torque_Nm && p.torque[1] == r.max_torque_Nm &&
        p.current[0] == -r.max_current_A && p.current[1] == r.max_current_A;
}
}

bool known_motor_model(dcu_motor_protocol_t protocol, const char *type) noexcept
{
    if (!type) return false;
    switch (protocol) {
    case DCU_MOTOR_PROTOCOL_ENCOS: return encos_config::find_model(type) != nullptr;
    case DCU_MOTOR_PROTOCOL_BJCX: return bjcx_config::find_model(type) != nullptr;
    default: return false;
    }
}

bool validate_motor_configuration(const MotorConfiguration &config, uint8_t format) noexcept
{
    switch (config.protocol) {
    case DCU_MOTOR_PROTOCOL_BJCX:
        return !config.zero_mit_loopback && known_motor_model(config.protocol, config.type) &&
            config.model_profile_id == 0 && format == 1 && !config.power_query_period_ms &&
            BjcxMotorDriver::validate(config.route.can_id, config.status_poll_period_ms,
                                      config.status_reply_timeout_ms, config.neutral_mit_only);
    case DCU_MOTOR_PROTOCOL_ENCOS:
        return config.model_profile_id == encos_config::coe_encoding_id && !config.neutral_mit_only &&
            (!config.zero_mit_loopback || (config.type && format == 1 &&
             config.route.position_min_urad <= 0 && config.route.position_max_urad >= 0 &&
             config.encos.profile.kp[0] == 0 && config.encos.profile.kd[0] == 0)) &&
            !config.status_poll_period_ms && !config.status_reply_timeout_ms &&
            (!config.power_query_period_ms || (config.power_query_period_ms >= 20 && config.power_query_period_ms <= 1000)) &&
            matches_encos_model(config, format) && EncosMotorDriver::validate(config.route, config.encos, format);
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
    if (!driver.configure(config.route, config.encos, format, config.zero_mit_loopback)) return nullptr;
    driver.bind_session(device, config.bus, format, config.power_query_period_ms);
    return &driver;
}

}
