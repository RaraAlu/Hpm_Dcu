#ifndef DCU_MOTOR_FEEDBACK_HPP
#define DCU_MOTOR_FEEDBACK_HPP

#include "motor_driver.hpp"

namespace dcu::motor {

enum MotorStatus : uint16_t {
    QuerySource = 1U << 12,
    ArmedQueryOnly = 1U << 13,
};

struct MotorFeedback {
    motor::Feedback sample{};
    int32_t power_mW{};
    uint16_t status_word{}, bridge_fault{}, feedback_age_ms{}, power_age_ms{};
    motor::ControlMode mode{};
    bool power_valid{};
};

struct MotorDiagnostics {
    uint32_t tx_enqueue_count{}, tx_drop_count{}, rx_valid_count{}, protocol_error_count{};
    uint32_t active_fault_bits{}, tx_complete_count{};
    int32_t queried_position_urad{}, queried_velocity_urad_s{};
    uint32_t position_query_age_us{}, velocity_query_age_us{};
};

}
#endif
