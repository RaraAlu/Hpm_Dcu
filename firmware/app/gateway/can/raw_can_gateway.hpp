#ifndef DCU_RAW_CAN_GATEWAY_HPP
#define DCU_RAW_CAN_GATEWAY_HPP

#include "can_frame.h"
#include "cfg/raw_can_gateway_config.hpp"
#include <cstdint>

namespace dcu::app {

struct RawCanBatch {
    enum class Kind : uint8_t { Sideband, FullBus, SingleFd };
    struct Slot {
        dcu_can_frame_t frame{};
        uint8_t bus{};
    } slots[config::raw_can::batch_capacity]{};
    uint32_t sequence{};
    Kind kind{Kind::Sideband};
    uint8_t bus{}, count{};
    bool valid{};
    unsigned capacity() const {
        return kind == Kind::SingleFd ? config::raw_can::single_fd_capacity : config::raw_can::batch_capacity;
    }
};

struct RawCanPolicy {
    struct Bus {
        uint32_t protected_ids[config::raw_can::protected_id_capacity]{};
        uint8_t protected_count{}, rate_limit{};
        bool can_send{}, fd_response{}, protect_full_bus{};
    } buses[config::raw_can::bus_capacity]{};
    // Caller resolves session and channel admission from its active configuration.
    bool permitted{};
};

struct RawCanDrops {
    uint32_t total{};
    uint32_t channel[config::raw_can::bus_capacity]{};
};

struct RawCanSubmit {
    void *context{};
    // Returns true only after the caller's budget check and successful submission.
    bool (*frame)(void *context, unsigned bus, const dcu_can_frame_t &frame, bool fd_response){};
};

// Owns raw FIFO, batch deduplication and rate limits; borrows submission per call.
class RawCanGateway {
public:
    // Discard sequence tracking; retain RX FIFO and rate history.
    void reset_session();
    void discard_frames();
    unsigned reject(const RawCanBatch &batch);
    void receive(unsigned bus, const dcu_can_frame_t &frame);
    bool take(unsigned bus, dcu_can_frame_t &frame);
    uint32_t rx_drops(unsigned bus) const { return bus < config::raw_can::bus_capacity ? rx_[bus].drops : 0; }
    // Call only for accepted input. Sideband consumes new 32-bit sequences even
    // when denied/invalid; FullBus consumes changed 16-bit sequences only when permitted.
    RawCanDrops process(const RawCanBatch &batch, const RawCanPolicy &policy, uint32_t now_ms,
                        RawCanSubmit submit);

private:
    bool accept_sequence(const RawCanBatch &batch);
    bool send(unsigned bus, const dcu_can_frame_t &frame, bool protect_ids, uint32_t now_ms,
              const RawCanPolicy &policy, RawCanSubmit submit);
    struct Queue {
        dcu_can_frame_t frames[config::raw_can::queue_capacity]{};
        uint8_t read{}, write{};
        uint32_t drops{};
    } rx_[config::raw_can::bus_capacity]{};
    bool seen_{};
    uint32_t last_batch_{};
    uint32_t last_full_cycle_[config::raw_can::bus_capacity]{}, budget_ms_[config::raw_can::bus_capacity]{};
    uint8_t budget_[config::raw_can::bus_capacity]{};
    bool full_seen_[config::raw_can::bus_capacity]{};
};

}
#endif
