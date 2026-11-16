#ifndef DCU_AXIS_CONTROLLER_HPP
#define DCU_AXIS_CONTROLLER_HPP

#include "cfg/motor_policy.hpp"
#include "motor_driver_binding.hpp"
#include <cstdint>

namespace dcu::app {

struct AxisPublication {
    bool online;
    bool fault;
};

// One task-owned controller per logical motor axis. No ISR may access it.
class AxisController {
public:
    // Call once with a valid logical axis before configuration or session work.
    void initialize(unsigned axis);
    bool configure(bool enabled, uint8_t frame_format);
    void disable();
    void stop(uint16_t fault);
    // After stop(), clear command-edge tracking without discarding RX history.
    void reset_session();
    bool accept(const mc_axis_rx_t &command, bool permitted, uint32_t now_ms);
    void check_timeouts(uint32_t now_ms, bool sdo_owner, uint16_t feedback_timeout_ms);
    void check_startup(uint32_t now_ms);
    bool query_ready(uint32_t now_ms) const;
    bool start_query(uint32_t now_ms, uint8_t owner, uint8_t frame_format);
    void consume_feedback(const dcu_can_frame_t &frame, uint32_t timestamp_ms, uint32_t now_ms);
    bool encode_control(dcu_can_frame_t &frame) const;
    void control_submitted(uint32_t now_ms);
    void set_sdo_lease(uint16_t lease_ms, uint32_t now_ms);
    AxisPublication publish(uint32_t now_ms, uint16_t feedback_timeout_ms);

    bool configured() const { return state_.configured; }
    bool active() const { return state_.active; }
    bool control_sent() const { return state_.control_sent; }
    bool matches(uint32_t can_id) const { return state_.configured && state_.route.CanId == can_id; }
    uint32_t type1_count() const { return state_.type1_count; }

private:
    struct State {
        mc_routing_t route{};
        mc_basic_t basic{};
        mc_range_t range{};
        dcu::motor::MotorDevice device{};
        ParameterDialect parameter_dialect{};
        mc_axis_rx_t command{}, startup_command{};
        uint32_t feedback_ms{}, power_ms{}, online_ms{}, position_ms{}, velocity_ms{}, current_ms{};
        uint32_t active_ms{}, lease_ms{}, commit_ms{}, type1_count{};
        uint32_t query_request_ms[config::motor_control::query_period_ms.size()]{};
        uint8_t query_requested_mask{};
        uint16_t previous_control{};
        bool configured{}, active{}, armed{}, query_armed{}, control_sent{};
        bool feedback_seen{}, power_seen{}, online_seen{}, position_seen{}, velocity_seen{}, current_seen{};
        bool new_feedback{}, new_power{};
    } state_{};
    unsigned axis_{};
};

}
#endif
