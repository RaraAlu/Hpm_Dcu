#include "raw_can_gateway.hpp"
#include "ethercat_canfd_gateway.h"
#include <cstring>

namespace dcu::app {
namespace motor_cfg = config::motor_control;

namespace {
bool newer(uint32_t a, uint32_t b)
{
    uint32_t d = a - b;
    return d && d < 0x80000000u;
}
}

void RawCanGateway::reset_session()
{
    seen_ = false;
    std::memset(full_seen_, 0, sizeof(full_seen_));
}
void RawCanGateway::receive(unsigned c, const dcu_can_frame_t &frame)
{
    if (c >= ENCOS_CHANNEL_COUNT) return;
    const auto *f = &frame;
    auto *q = &rx_[c];
    uint8_t next = (q->write + 1) & (motor_cfg::raw_queue_capacity - 1);
    if (next == q->read) {
        ++q->drops;
        return;
    }
    q->frames[q->write] = *f;
    q->write = next;
}
bool RawCanGateway::take(unsigned c, dcu_can_frame_t &frame)
{
    if (c >= ENCOS_CHANNEL_COUNT) return false;
    auto *q = &rx_[c];
    if (q->read == q->write)
        return false;
    frame = q->frames[q->read];
    q->read = (q->read + 1) & (motor_cfg::raw_queue_capacity - 1);
    return true;
}
bool RawCanGateway::send(unsigned c, const dcu_can_frame_t &frame, bool protect_ids, uint32_t now,
                         const RawCanPolicy &policy,
                         BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT])
{
    const auto *f = &frame;
    if (c >= ENCOS_CHANNEL_COUNT || !policy.buses[c].can_send)
        return false;
    const auto &bus = policy.buses[c];
    if (protect_ids) {
        if (f->id == 0x7FF || bus.protected_count > ENCOS_MOTORS_PER_CHANNEL)
            return false;
        for (unsigned i = 0; i < bus.protected_count; ++i)
            if (bus.protected_ids[i] == f->id)
                return false;
    }
    if (now - budget_ms_[c] >= 2) {
        budget_ms_[c] = now;
        budget_[c] = 0;
    }
    if (budget_[c] >= bus.rate_limit) {
        return false;
    }
    unsigned cost = BusScheduler::frame_cost((f->flags & 8u) != 0, f->dlc) +
                    BusScheduler::frame_cost(bus.fd_response, 8);
    if (!schedule[c].can_afford(cost) || !ethercat_canfd_send(c, f, ENCOS_AXIS_COUNT))
        return false;
    schedule[c].charge(cost);
    ++budget_[c];
    return true;
}
RawCanDrops RawCanGateway::process(const RawCanBatch &batch, const RawCanPolicy &policy, uint32_t now,
                                   BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT])
{
    RawCanDrops drops{};
    const bool sideband = batch.kind == RawCanBatch::Kind::Sideband;
    if (sideband) {
        if (seen_ && !newer(batch.sequence, last_batch_))
            return drops;
        last_batch_ = batch.sequence;
        seen_ = true;
    } else if (batch.kind == RawCanBatch::Kind::FullBus) {
        const unsigned c = batch.bus;
        if (!policy.permitted || c >= ENCOS_CHANNEL_COUNT)
            return drops;
        if (full_seen_[c] && static_cast<uint16_t>(batch.sequence - last_full_cycle_[c]) == 0)
            return drops;
        last_full_cycle_[c] = batch.sequence;
        full_seen_[c] = true;
    } else {
        return drops;
    }
    if (!policy.permitted || !batch.valid || batch.count > motor_cfg::raw_batch_capacity)
        return drops;
    for (unsigned i = 0; i < batch.count; ++i) {
        const auto &slot = batch.slots[i];
        const unsigned c = sideband ? slot.bus : batch.bus;
        if (!send(c, slot.frame, sideband, now, policy, schedule)) {
            ++drops.total;
            if (c < ENCOS_CHANNEL_COUNT)
                ++drops.channel[c];
        }
    }
    return drops;
}

}
