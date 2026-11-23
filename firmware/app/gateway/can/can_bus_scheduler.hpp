#ifndef DCU_CAN_BUS_SCHEDULER_HPP
#define DCU_CAN_BUS_SCHEDULER_HPP

#include "device_config.h"
#include <cstdint>

namespace dcu::app {

struct ControlSet {
    uint8_t active_mask{}, first_send_mask{};
    uint32_t period_us{};
};

// One gateway-owned scheduler per bus. It selects work, but never sends a frame.
class BusScheduler {
public:
    static unsigned frame_cost(bool fd, unsigned bytes);
    void reset_budget(uint32_t now_ms);
    void refill(uint32_t now_ms);
    bool can_afford(unsigned cost) const { return credit_us_ >= cost; }
    bool query_window(ControlSet controls, uint32_t now_us,
                      unsigned query_cost, unsigned control_cost) const;
    int next_control(ControlSet controls, uint32_t now_us,
                     unsigned cost, bool transmitter_ready) const;
    void control_submitted(unsigned slot, uint32_t now_us, uint32_t period_us,
                           bool first, unsigned cost);
    // Charge only after can_afford() and a successful transmitter submission.
    void charge(unsigned cost) { credit_us_ -= cost; }
    bool stalled(uint32_t now_ms, uint32_t progress, bool pending, uint16_t timeout_ms);

private:
    uint32_t last_control_us_[DCU_MOTOR_DEVICES_PER_BUS]{};
    uint32_t last_budget_ms_{}, last_progress_{}, last_progress_ms_{};
    uint16_t credit_us_{};
    uint8_t control_slot_{};
};

}
#endif
