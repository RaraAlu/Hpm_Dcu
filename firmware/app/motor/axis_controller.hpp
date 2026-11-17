#ifndef DCU_AXIS_CONTROLLER_HPP
#define DCU_AXIS_CONTROLLER_HPP

#include "detail/axis_state.hpp"
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
    bool configure(bool enabled, uint8_t frame_format, const mc_routing_t &route,
                   const mc_basic_t &basic, const mc_range_t &range);
    void disable();
    void stop(uint16_t fault);
    // After stop(), clear command-edge tracking without discarding RX history.
    void reset_session();
    bool accept(const mc_axis_rx_t &command, bool permitted, uint32_t now_ms);
    void check_timeouts(uint32_t now_ms, bool sdo_owner, uint16_t feedback_timeout_ms);
    void check_startup(uint32_t now_ms);
    bool query_ready(uint32_t now_ms) const;
    bool start_query(uint32_t now_ms, uint8_t owner, uint8_t frame_format);
    // service_dialect belongs to a matching transaction, including pre-Apply reads.
    void consume_feedback(const dcu_can_frame_t &frame, uint32_t timestamp_ms, uint32_t now_ms,
                          ParameterDialect service_dialect);
    bool encode_control(dcu_can_frame_t &frame) const;
    void control_submitted(uint32_t now_ms);
    void set_sdo_lease(uint16_t lease_ms, uint32_t now_ms);
    // Copy a snapshot without consuming event flags. Preserve adapter-owned
    // TxDropCount and LastServiceError in diagnostics.
    AxisPublication read_feedback(uint32_t now_ms, uint16_t feedback_timeout_ms,
                                  mc_axis_tx_t &feedback, mc_diag_t &diagnostics) const;
    // Only the task's PDO publication path acknowledges the copied event flags.
    void acknowledge_publication();

    bool configured() const { return state_.configured; }
    bool active() const { return state_.active; }
    bool control_sent() const { return state_.control_sent; }
    bool matches(uint32_t can_id) const { return state_.configured && state_.route.CanId == can_id; }
    uint32_t type1_count() const { return state_.type1_count; }

private:
    detail::AxisState state_{};
    detail::AxisHistory history_{};
    unsigned axis_{};
};

}
#endif
