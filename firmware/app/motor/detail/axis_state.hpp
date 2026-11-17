#ifndef DCU_AXIS_STATE_HPP
#define DCU_AXIS_STATE_HPP

#include "cfg/motor_policy.hpp"
#include "motor_driver_binding.hpp"
#include <cstdint>

namespace dcu::app::detail {

enum class AxisPhase : uint8_t {
    Stopped,
    ReadyForArm,
    QueryOnly,
    StartupQueued,
    AwaitingFeedback,
    Running,
};

struct AxisState {
    motor::Routing route{};
    uint16_t power_query_period_ms{};
    dcu::motor::MotorDevice device{};
    ParameterDialect parameter_dialect{};
    motor::Command command{}, startup_command{};
    AxisFeedback feedback{};
    uint32_t feedback_ms{}, power_ms{}, online_ms{}, position_ms{}, velocity_ms{}, current_ms{};
    uint32_t active_ms{}, lease_ms{}, commit_ms{}, type1_count{};
    uint32_t query_request_ms[config::motor_control::query_period_ms.size()]{};
    uint8_t query_requested_mask{};
    uint16_t previous_control{};
    AxisPhase phase{AxisPhase::Stopped};
    bool configured{};
    bool power_seen{}, online_seen{}, position_seen{}, velocity_seen{}, current_seen{};
    bool new_feedback{}, new_power{};
};

// Apply resets control state, not lifetime counters or the last queried values.
struct AxisHistory {
    uint32_t tx_enqueue_count{}, tx_drop_count{}, rx_valid_count{}, protocol_error_count{}, active_fault_bits{};
    int32_t queried_position{}, queried_velocity{};
};

}
#endif
