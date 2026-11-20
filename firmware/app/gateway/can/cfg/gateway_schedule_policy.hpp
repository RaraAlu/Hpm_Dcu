#ifndef DCU_GATEWAY_SCHEDULE_POLICY_HPP
#define DCU_GATEWAY_SCHEDULE_POLICY_HPP

#include "cycle_config.h"
#include <cstdint>

namespace dcu::app::config::gateway_schedule {

inline constexpr uint32_t process_period_us = DCU_WORKER_PERIOD_US;
inline constexpr uint32_t control_response_guard_us = 100;
inline constexpr unsigned can_budget_us_per_ms = 800;
inline constexpr unsigned can_budget_cap_us = 2400;
inline constexpr unsigned budget_refill_window_ms =
    (can_budget_cap_us + can_budget_us_per_ms - 1) / can_budget_us_per_ms;

static_assert(can_budget_us_per_ms <= 1000 && can_budget_cap_us >= can_budget_us_per_ms);
static_assert(control_response_guard_us >= process_period_us);

}
#endif
