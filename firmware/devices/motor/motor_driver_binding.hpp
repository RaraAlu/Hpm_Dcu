#ifndef DCU_MOTOR_DRIVER_BINDING_HPP
#define DCU_MOTOR_DRIVER_BINDING_HPP

#include "motor_session_driver.hpp"
#include "encos_motor.h"
#include "motor_protocol.h"

namespace dcu::motor {

struct MotorConfiguration {
    Routing route{};
    encos_settings_t encos{};
    uint16_t model_profile_id{}, power_query_period_ms{};
    unsigned bus{};
    uint16_t status_poll_period_ms{}, status_reply_timeout_ms{};
    dcu_motor_protocol_t protocol{};
    bool neutral_mit_only{};
    const char *type{};
};

bool known_motor_model(dcu_motor_protocol_t protocol, const char *type) noexcept;
bool validate_motor_configuration(const MotorConfiguration &, uint8_t frame_format) noexcept;
MotorSessionDriver *configure_motor_driver(unsigned device, const MotorConfiguration &, uint8_t frame_format) noexcept;

}
#endif
