#ifndef DCU_AXIS_CONTRACT_HPP
#define DCU_AXIS_CONTRACT_HPP

#include "motor_driver.hpp"
#include "encos_motor.h"

namespace dcu::app {

struct AxisConfiguration {
    motor::Routing route{};
    encos_settings_t encos{};
    uint16_t model_profile_id{}, power_query_period_ms{};
    uint8_t stop_policy{}, feedback_policy{};
};

enum AxisStatus : uint16_t {
    QuerySource = 1U << 12,
    ArmedQueryOnly = 1U << 13,
};

struct AxisFeedback {
    motor::Feedback sample{};
    int32_t power_mW{};
    uint16_t status_word{}, bridge_fault{}, feedback_age_ms{}, power_age_ms{};
    motor::ControlMode mode{};
    bool power_valid{};
};

struct AxisDiagnostics {
    uint32_t tx_enqueue_count{}, tx_drop_count{}, rx_valid_count{}, protocol_error_count{};
    uint32_t active_fault_bits{}, tx_complete_count{};
    int32_t queried_position_urad{}, queried_velocity_urad_s{};
    uint32_t position_query_age_us{}, velocity_query_age_us{};
};

}
#endif
