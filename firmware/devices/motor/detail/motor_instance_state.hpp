#ifndef DCU_MOTOR_INSTANCE_STATE_HPP
#define DCU_MOTOR_INSTANCE_STATE_HPP

#include "motor_feedback.hpp"

namespace dcu::motor::detail {

enum class MotorPhase : uint8_t {
    Stopped,
    ReadyForArm,
    QueryOnly,
    StartupQueued,
    AwaitingFeedback,
    Running,
};

struct MotorState {
    Command command{}, startup_command{};
    MotorFeedback feedback{};
    uint32_t feedback_ms{}, online_ms{}, active_ms{};
    uint32_t cyclic_feedback_count{};
    uint16_t previous_control{};
    MotorPhase phase{MotorPhase::Stopped};
    bool new_feedback{};
    bool neutral_armed{}, neutral_pending{}, neutral_reply{};
};

struct MotorHistory {
    uint32_t tx_enqueue_count{}, tx_drop_count{}, rx_valid_count{}, protocol_error_count{}, active_fault_bits{};
    uint32_t tx_complete_count{};
    int32_t queried_position{}, queried_velocity{};
};

}
#endif
