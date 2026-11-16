#ifndef DCU_RAW_CAN_GATEWAY_HPP
#define DCU_RAW_CAN_GATEWAY_HPP

#include "axis_controller.hpp"
#include "can_bus_scheduler.hpp"
#include "cfg/motor_policy.hpp"
#include "device_config.h"
#include <cstdint>

namespace dcu::app {

struct RawCanCycle {
    unsigned profile;
    uint8_t enabled_channels;
    // Caller has checked OP/session, BoardControl and command lease.
    bool permitted;
    // Borrowed RxPDO image; required for profile 2 during process().
    const uint8_t *pdo;
};

struct RawCanDrops {
    uint32_t total{};
    uint32_t channel[ENCOS_CHANNEL_COUNT]{};
};

// Owns raw FIFO, batch deduplication and rate limits; borrows bus budgets per call.
class RawCanGateway {
public:
    // Discard pending work and sequence tracking; retain RX FIFO and rate history.
    void reset_session();
    void cancel() { pending_ = false; }
    void stage_full(bool enabled) { pending_ = enabled; }
    void stage_batch(uint32_t sequence, bool enabled);
    void receive(unsigned bus, const dcu_can_frame_t &frame);
    bool take(unsigned bus, dcu_can_frame_t &frame);
    uint32_t rx_drops(unsigned bus) const { return bus < ENCOS_CHANNEL_COUNT ? rx_[bus].drops : 0; }
    RawCanDrops process(const RawCanCycle &cycle, uint32_t now_ms,
                        const mc_channel_config_t (&channels)[ENCOS_CHANNEL_COUNT],
                        const AxisController (&axes)[ENCOS_AXIS_COUNT],
                        BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT]);

private:
    bool send(unsigned bus, const dcu_can_frame_t &frame, const RawCanCycle &cycle, uint32_t now_ms,
              const mc_channel_config_t (&channels)[ENCOS_CHANNEL_COUNT],
              const AxisController (&axes)[ENCOS_AXIS_COUNT],
              BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT]);
    struct Queue {
        dcu_can_frame_t frames[config::motor_control::raw_queue_capacity]{};
        uint8_t read{}, write{};
        uint32_t drops{};
    } rx_[ENCOS_CHANNEL_COUNT]{};
    bool pending_{}, seen_{};
    uint32_t last_batch_{};
    uint32_t last_full_cycle_[ENCOS_CHANNEL_COUNT]{}, budget_ms_[ENCOS_CHANNEL_COUNT]{};
    uint8_t budget_[ENCOS_CHANNEL_COUNT]{};
    bool full_seen_[ENCOS_CHANNEL_COUNT]{};
};

}
#endif
