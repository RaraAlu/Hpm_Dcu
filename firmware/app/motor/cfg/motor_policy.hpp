#ifndef DCU_MOTOR_POLICY_HPP
#define DCU_MOTOR_POLICY_HPP

#include "cycle_config.h"
#include <array>
#include <cstdint>

namespace dcu::app::config::motor_control {

inline constexpr uint32_t process_period_us = DCU_WORKER_PERIOD_US;
inline constexpr uint32_t control_response_guard_us = 100;
inline constexpr unsigned can_budget_us_per_ms = 800;
inline constexpr unsigned can_budget_cap_us = 2400;
inline constexpr unsigned budget_refill_window_ms =
    (can_budget_cap_us + can_budget_us_per_ms - 1) / can_budget_us_per_ms;
inline constexpr unsigned raw_queue_capacity = 32;
inline constexpr unsigned raw_batch_capacity = 7;
inline constexpr uint32_t query_online_ms = 300;
inline constexpr uint32_t query_position_ms = 100;
inline constexpr uint32_t query_velocity_ms = 150;
inline constexpr uint32_t query_current_ms = 300;
inline constexpr uint32_t parameter_timeout_ms = 100;
inline constexpr uint32_t parameter_hold_ms = 500;
// Group 3 uses the per-axis PowerQueryPeriodMs at runtime.
inline constexpr std::array<uint16_t, 4> query_period_ms{100, 100, 200, 1000};

static_assert(can_budget_us_per_ms <= 1000 && can_budget_cap_us >= can_budget_us_per_ms);
static_assert(raw_queue_capacity > 1 && raw_queue_capacity <= 256 &&
              (raw_queue_capacity & (raw_queue_capacity - 1)) == 0);
static_assert(control_response_guard_us >= process_period_us);

}
#endif
