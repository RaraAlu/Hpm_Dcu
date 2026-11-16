#ifndef DCU_MOTOR_APPLICATION_STATE_HPP
#define DCU_MOTOR_APPLICATION_STATE_HPP

#include "../cfg/motor_policy.hpp"
#include "can_bus_scheduler.hpp"
#include "encos_motor.h"
#include "motor_driver_binding.hpp"
#include "motor_pdo_layout.hpp"
#include <cstdint>

namespace dcu::app::motor_application::detail {
namespace motor_cfg = dcu::app::config::motor_control;
namespace pdo_layout = dcu::ethercat::pdo_layout;

typedef struct
{
    mc_routing_t route;
    mc_basic_t basic;
    mc_range_t range;
    dcu::motor::MotorDevice device;
    dcu::app::ParameterDialect parameter_dialect;
    mc_axis_rx_t command;
    mc_axis_rx_t startup_command;
    uint32_t feedback_ms, power_ms, online_ms, position_ms, velocity_ms, current_ms, active_ms, lease_ms, commit_ms;
    uint32_t type1_count;
    uint32_t query_request_ms[motor_cfg::query_period_ms.size()];
    uint8_t query_requested_mask;
    uint16_t previous_control;
    bool configured, active, armed, query_armed, control_sent, feedback_seen, power_seen, online_seen, position_seen, velocity_seen, current_seen, new_feedback, new_power;
} axis_t;
typedef struct
{
    encos_frame_t frames[motor_cfg::raw_queue_capacity];
    uint8_t read, write;
    uint32_t drops;
} raw_queue_t;
/* MainLoop, SDO callbacks and motor state have one RTOS task owner. ISR paths
 * exchange complete PDO snapshots and CAN queue entries, never these objects. */
struct MotorControlState {
    axis_t axes[ENCOS_AXIS_COUNT]{};
    raw_queue_t raw_rx[ENCOS_CHANNEL_COUNT]{};
    mc_channel_config_t channels[ENCOS_CHANNEL_COUNT]{};
    mc_board_config_t config{};

    mc_board_rx_t board_command{};
    mc_channel_rx_t channel_command[ENCOS_CHANNEL_COUNT]{};
    uint8_t candidate[pdo_layout::max_output_bytes]{};
    uint16_t output_size{};

    bool operational{}, pending{}, session_armed{}, resume_pdo_owner{};
    bool seen_cycle{}, config_ready{}, raw_pending{}, raw_seen{};
    uint32_t last_command_ms{}, last_cycle{}, last_raw_batch{}, rx_total{}, tx_total{};
    uint32_t last_process_us{};
    dcu::app::BusScheduler schedule[ENCOS_CHANNEL_COUNT]{};
    uint32_t last_full_cycle[ENCOS_CHANNEL_COUNT]{}, raw_budget_ms[ENCOS_CHANNEL_COUNT]{};
    uint8_t raw_budget[ENCOS_CHANNEL_COUNT]{};
    uint8_t query_slot[ENCOS_CHANNEL_COUNT]{};
    bool full_seen[ENCOS_CHANNEL_COUNT]{};
    uint16_t board_fault{};
    uint32_t channel_tx_drop[ENCOS_CHANNEL_COUNT]{};
};

}
#endif
