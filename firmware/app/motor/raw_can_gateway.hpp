#ifndef DCU_RAW_CAN_GATEWAY_HPP
#define DCU_RAW_CAN_GATEWAY_HPP

#include "can_frame.h"
#include "can_bus_scheduler.hpp"
#include "cfg/motor_policy.hpp"
#include "device_config.h"
#include <cstdint>

namespace dcu::app {

struct RawCanBatch {
    enum class Kind : uint8_t { Sideband, FullBus };
    struct Slot {
        dcu_can_frame_t frame{};
        uint8_t bus{};
    } slots[config::motor_control::raw_batch_capacity]{};
    uint32_t sequence{};
    Kind kind{Kind::Sideband};
    uint8_t bus{}, count{};
    bool valid{};
};

struct RawCanPolicy {
    struct Bus {
        uint32_t protected_ids[ENCOS_MOTORS_PER_CHANNEL]{};
        uint8_t protected_count{}, rate_limit{};
        bool can_send{}, fd_response{};
    } buses[ENCOS_CHANNEL_COUNT]{};
    // Caller checks OP/session, BoardControl and lease, then supplies current
    // channel/service admission and IDs from applied (not pending) configuration.
    bool permitted{};
};

struct RawCanDrops {
    uint32_t total{};
    uint32_t channel[ENCOS_CHANNEL_COUNT]{};
};

// Owns raw FIFO, batch deduplication and rate limits; borrows bus budgets per call.
class RawCanGateway {
public:
    // Discard sequence tracking; retain RX FIFO and rate history.
    void reset_session();
    void receive(unsigned bus, const dcu_can_frame_t &frame);
    bool take(unsigned bus, dcu_can_frame_t &frame);
    uint32_t rx_drops(unsigned bus) const { return bus < ENCOS_CHANNEL_COUNT ? rx_[bus].drops : 0; }
    // Call only for accepted input. Sideband consumes new 32-bit sequences even
    // when denied/invalid; FullBus consumes changed 16-bit sequences only when permitted.
    RawCanDrops process(const RawCanBatch &batch, const RawCanPolicy &policy, uint32_t now_ms,
                        BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT]);

private:
    bool send(unsigned bus, const dcu_can_frame_t &frame, bool protect_ids, uint32_t now_ms,
              const RawCanPolicy &policy,
              BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT]);
    struct Queue {
        dcu_can_frame_t frames[config::motor_control::raw_queue_capacity]{};
        uint8_t read{}, write{};
        uint32_t drops{};
    } rx_[ENCOS_CHANNEL_COUNT]{};
    bool seen_{};
    uint32_t last_batch_{};
    uint32_t last_full_cycle_[ENCOS_CHANNEL_COUNT]{}, budget_ms_[ENCOS_CHANNEL_COUNT]{};
    uint8_t budget_[ENCOS_CHANNEL_COUNT]{};
    bool full_seen_[ENCOS_CHANNEL_COUNT]{};
};

}
#endif
