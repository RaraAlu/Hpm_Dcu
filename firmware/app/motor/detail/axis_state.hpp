#ifndef DCU_AXIS_STATE_HPP
#define DCU_AXIS_STATE_HPP

#include "cfg/motor_policy.hpp"
#include "motor_driver_binding.hpp"
#include <cstdint>

namespace dcu::app::detail {

struct AxisState {
    mc_routing_t route{};
    dcu::motor::MotorDevice device{};
    ParameterDialect parameter_dialect{};
    mc_axis_rx_t command{}, startup_command{};
    mc_axis_tx_t feedback{};
    uint32_t feedback_ms{}, power_ms{}, online_ms{}, position_ms{}, velocity_ms{}, current_ms{};
    uint32_t active_ms{}, lease_ms{}, commit_ms{}, type1_count{};
    uint32_t query_request_ms[config::motor_control::query_period_ms.size()]{};
    uint8_t query_requested_mask{};
    uint16_t previous_control{};
    // A permitted ARM=0, STOP=0 command must precede a new ARM edge.
    bool arm_edge_ready{};
    bool configured{}, active{}, query_armed{}, control_sent{};
    bool feedback_seen{}, power_seen{}, online_seen{}, position_seen{}, velocity_seen{}, current_seen{};
    bool new_feedback{}, new_power{};
};

// Apply resets control state, not lifetime counters or the last queried values.
struct AxisHistory {
    uint32_t tx_enqueue_count{}, rx_valid_count{}, protocol_error_count{}, active_fault_bits{};
    int32_t queried_position{}, queried_velocity{};
};

}
#endif
