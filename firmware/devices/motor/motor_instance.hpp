#ifndef DCU_MOTOR_INSTANCE_HPP
#define DCU_MOTOR_INSTANCE_HPP

#include "can_transport.h"
#include "detail/motor_instance_state.hpp"
#include "motor_session_driver.hpp"
#include <cstdint>

namespace dcu::motor {

struct MotorConfiguration;

struct ControlPreview {
    Command command;
    CanFrame frame;
};

struct MotorPublication {
    bool online;
    bool fault;
};

// One task-owned instance per configured motor, independent of EtherCAT slots.
class MotorInstance {
public:
    bool bind(unsigned device, const MotorConfiguration &, uint8_t frame_format);
    void disable();
    void stop(uint16_t fault);
    // After stop(), clear command-edge tracking without discarding RX history.
    void reset_session();
    // A cyclic caller supplies a destination and an optional preview for this axis and sequence.
    bool accept(const Command &command, bool permitted, uint32_t now_ms, CanFrame *cyclic,
                const ControlPreview *preview, bool &used_preview);
    void check_timeouts(uint32_t now_ms, bool sdo_owner, uint16_t feedback_timeout_ms);
    void check_startup(uint32_t now_ms);
    bool query_ready(uint32_t now_ms) const;
    bool start_query(uint32_t now_ms, bool cyclic_owner);
    bool prepare_status_poll(uint32_t now_ms, StatusPoll &poll);
    bool consume_status_reply(const CanFrame &, uint32_t timestamp_ms);
    bool neutral_mit_only() const { return driver_ && driver_->neutral_mit_only(); }
    bool accept_neutral_mit(const Command &, bool permitted, bool continuing);
    bool neutral_mit_requested() const { return state_.neutral_pending; }
    bool take_neutral_mit(CanFrame &, uint8_t counter);
    bool consume_neutral_mit(const CanFrame &, uint32_t timestamp_ms);
    void abort_neutral_mit(uint16_t fault);
    bool accepts_payload(uint8_t bytes) const { return driver_ && bytes <= driver_->max_rx_bytes(); }
    bool supports_legacy_queries() const { return driver_ && driver_->capabilities().legacy_parameter_queries; }
    bool consume_feedback(const CanFrame &, uint32_t timestamp_ms, uint32_t now_ms);
    bool encode_control(dcu_can_frame_t &frame) const;
    bool preview_control(const Command &command, CanFrame &frame) const {
        return driver_ && driver_->encode(command, frame) == Result::Ok;
    }
    const Command &effective_control() const {
        return state_.phase == detail::MotorPhase::Running ? state_.command : state_.startup_command;
    }
    void control_submitted(uint32_t now_ms, dcu_can_tx_id_t id);
    bool finish_tx(const dcu_can_tx_event_t &event);
    void set_sdo_lease(uint16_t lease_ms, uint32_t now_ms);
    void record_tx_drop() { ++history_.tx_drop_count; }
    // Copy a complete task-owned snapshot without consuming event flags.
    MotorPublication read_feedback(uint32_t now_ms, uint16_t feedback_timeout_ms,
                                  MotorFeedback &feedback, MotorDiagnostics &diagnostics) const;
    // Only the task's PDO publication path acknowledges the copied event flags.
    void acknowledge_publication();

    bool configured() const { return driver_ != nullptr; }
    unsigned bus() const { return bus_; }
    // Meaningful only for a bound device.
    uint32_t can_id() const { return route_.can_id; }
    bool active() const
    {
        return state_.phase == detail::MotorPhase::StartupQueued || control_sent();
    }
    bool control_sent() const
    {
        return state_.phase == detail::MotorPhase::AwaitingFeedback || state_.phase == detail::MotorPhase::Running;
    }
    bool matches(uint32_t can_id) const { return driver_ && route_.can_id == can_id; }
    uint32_t cyclic_feedback_count() const { return state_.cyclic_feedback_count; }

private:
    detail::MotorState state_{};
    detail::MotorHistory history_{};
    dcu_can_tx_id_t control_tx_id_{};
    Routing route_{};
    MotorSessionDriver *driver_{};
    unsigned bus_{};
};

}
#endif
