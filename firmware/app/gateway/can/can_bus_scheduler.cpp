#include "can_bus_scheduler.hpp"
#include "can_transport.h"
#include "cfg/gateway_schedule_policy.hpp"
#include <climits>

namespace dcu::app {
namespace cfg = config::gateway_schedule;

unsigned BusScheduler::frame_cost(bool fd, unsigned bytes)
{
    // Engineering estimates for the configured 1/5 Mbps buses, not measurements.
    return fd ? 48u + 2u * bytes : 50u + 11u * bytes;
}
void BusScheduler::reset_budget(uint32_t now)
{
    credit_us_ = cfg::can_budget_cap_us;
    last_budget_ms_ = now;
}
void BusScheduler::refill(uint32_t now)
{
    uint32_t elapsed = now - last_budget_ms_;
    if (!elapsed) return;
    last_budget_ms_ = now;
    unsigned credit = elapsed >= cfg::budget_refill_window_ms ? cfg::can_budget_cap_us :
        credit_us_ + elapsed * cfg::can_budget_us_per_ms;
    credit_us_ = credit > cfg::can_budget_cap_us ? cfg::can_budget_cap_us : credit;
}
bool BusScheduler::query_window(ControlSet controls, uint32_t now,
                                unsigned query_cost, unsigned control_cost) const
{
    if (!controls.active_mask) return true;
    unsigned active = 0;
    uint32_t slack = UINT32_MAX;
    for (unsigned slot = 0; slot < DCU_MOTOR_DEVICES_PER_BUS; ++slot) {
        if (!(controls.active_mask & (1u << slot))) continue;
        ++active;
        if ((controls.first_send_mask & (1u << slot)) || !controls.period_us) return false;
        uint32_t elapsed = now - last_control_us_[slot];
        if (elapsed >= controls.period_us) return false;
        uint32_t remaining = controls.period_us - elapsed;
        if (remaining < slack) slack = remaining;
    }
    if (uint64_t(active) * control_cost * 1000u > uint64_t(cfg::can_budget_us_per_ms) * controls.period_us)
        return false;
    return slack >= DCU_CAN_TX_EOF_GAP_US + cfg::process_period_us + query_cost &&
           credit_us_ >= query_cost + active * control_cost;
}
int BusScheduler::next_control(ControlSet controls, uint32_t now, unsigned cost, bool ready) const
{
    if (!ready || !can_afford(cost)) return -1;
    for (unsigned n = 0; n < DCU_MOTOR_DEVICES_PER_BUS; ++n) {
        unsigned slot = (control_slot_ + n) % DCU_MOTOR_DEVICES_PER_BUS;
        if (!(controls.active_mask & (1u << slot))) continue;
        if ((controls.first_send_mask & (1u << slot)) || uint32_t(now - last_control_us_[slot]) >= controls.period_us)
            return static_cast<int>(slot);
    }
    return -1;
}
void BusScheduler::control_submitted(unsigned slot, uint32_t now, uint32_t period, bool first, unsigned cost)
{
    charge(cost);
    uint32_t elapsed = now - last_control_us_[slot];
    // Preserve the requested phase, but do not burst to catch up after a stall.
    if (first || elapsed >= 2u * period) last_control_us_[slot] = now;
    else last_control_us_[slot] += period;
    control_slot_ = (slot + 1) % DCU_MOTOR_DEVICES_PER_BUS;
}
bool BusScheduler::stalled(uint32_t now, uint32_t progress, bool pending, uint16_t timeout)
{
    if (progress != last_progress_ || !pending) {
        last_progress_ = progress;
        last_progress_ms_ = now;
        return false;
    }
    return uint32_t(now - last_progress_ms_) >= timeout;
}

}
