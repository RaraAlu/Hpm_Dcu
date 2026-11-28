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

bool zero_target_bounds(const Routing &route)
{
    return !route.position_min_urad && !route.position_max_urad && !route.velocity_limit_urad_s &&
        !route.torque_limit_mNm && route.current_limit_dA;
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
    if (config.motion_parameters_pending && (config.neutral_mit_only ||
        config.model_profile_id || format != 1 || !known_motor_model(config.protocol, config.type))) return false;
    switch (config.protocol) {
    case DCU_MOTOR_PROTOCOL_BJCX:
        return known_motor_model(config.protocol, config.type) &&
            (config.model_profile_id == 0 ||
             (config.model_profile_id == bjcx_config::coe_encoding_id && !config.neutral_mit_only &&
              zero_target_bounds(config.route))) &&
            format == 1 && !config.power_query_period_ms &&
            BjcxMotorDriver::validate(config.route.can_id, config.status_poll_period_ms,
                                      config.status_reply_timeout_ms, config.neutral_mit_only);
    case DCU_MOTOR_PROTOCOL_ENCOS:
        if (config.motion_parameters_pending)
            return config.route.can_id >= 1 && config.route.can_id <= 8 && !config.status_poll_period_ms &&
                   !config.status_reply_timeout_ms && !config.power_query_period_ms;
        if (zero_target_bounds(config.route)) {
            const auto *model = config.type ? encos_config::find_model(config.type) : nullptr;
            return config.model_profile_id == encos_config::coe_encoding_id && format == 1 && !config.neutral_mit_only &&
                !config.status_poll_period_ms && !config.status_reply_timeout_ms && !config.power_query_period_ms &&
                model && model->centered_mit_zero && model->ack1.known() &&
                config.route.current_limit_dA <= model->ack1.current_A * 10 && config.route.can_id >= 1 && config.route.can_id <= 8;
        }
        return config.model_profile_id == encos_config::coe_encoding_id && !config.neutral_mit_only &&
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
                                config.status_reply_timeout_ms, config.neutral_mit_only,
                                config.model_profile_id == bjcx_config::coe_encoding_id) ? &driver : nullptr;
    }
    auto &driver = encos_drivers[device];
    if (config.motion_parameters_pending) driver.configure_queries(config.route);
    else if (zero_target_bounds(config.route)) {
        if (!driver.configure_impedance(config.route, *encos_config::find_model(config.type))) return nullptr;
    }
    else if (!driver.configure(config.route, config.encos, format,
                               encos_config::find_model(config.type)->centered_mit_zero)) return nullptr;
    driver.bind_session(device, config.bus, format, config.power_query_period_ms);
    return &driver;
}

}
