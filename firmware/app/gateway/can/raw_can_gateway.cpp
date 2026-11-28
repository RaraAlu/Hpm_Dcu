#include "raw_can_gateway.hpp"
#include <cstring>

namespace dcu::app {
namespace raw_cfg = config::raw_can;

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

void RawCanGateway::discard_frames()
{
    for (auto &queue : rx_) queue.read = queue.write;
}


void RawCanGateway::receive(unsigned c, const dcu_can_frame_t &frame)
{
    if (c >= raw_cfg::bus_capacity) return;
    const auto *f = &frame;
    auto *q = &rx_[c];
    uint8_t next = (q->write + 1) & (raw_cfg::queue_capacity - 1);
    if (next == q->read) {
        ++q->drops;
        return;
    }
    q->frames[q->write] = *f;
    q->write = next;
}


bool RawCanGateway::take(unsigned c, dcu_can_frame_t &frame)
{
    if (c >= raw_cfg::bus_capacity) return false;
    auto *q = &rx_[c];
    if (q->read == q->write)
        return false;
    frame = q->frames[q->read];
    q->read = (q->read + 1) & (raw_cfg::queue_capacity - 1);
    return true;
}


bool RawCanGateway::send(unsigned c, const dcu_can_frame_t &frame, RawCanBatch::Kind kind, uint32_t now,
                         const RawCanPolicy &policy, RawCanSubmit submit)
{
    if (c >= raw_cfg::bus_capacity || !policy.buses[c].can_send || !submit.frame)
        return false;
    if (kind == RawCanBatch::Kind::SingleFd)
        return submit.frame(submit.context, c, frame, false);
    const auto &bus = policy.buses[c];
    if (kind == RawCanBatch::Kind::Sideband || bus.protect_full_bus) {
        if (bus.protected_count > raw_cfg::protected_id_capacity)
            return false;
        for (unsigned i = 0; i < bus.protected_count; ++i)
            if (bus.protected_ids[i] == frame.id)
                return false;
    }
    if (now - budget_ms_[c] >= raw_cfg::rate_window_ms) {
        budget_ms_[c] = now;
        budget_[c] = 0;
    }
    if (budget_[c] >= bus.rate_limit) {
        return false;
    }
    if (!submit.frame(submit.context, c, frame, bus.fd_response))
        return false;
    ++budget_[c];
    return true;
}
bool RawCanGateway::accept_sequence(const RawCanBatch &batch)
{
    const bool sideband = batch.kind == RawCanBatch::Kind::Sideband || batch.kind == RawCanBatch::Kind::SingleFd;
    if (sideband) {
        if (seen_ && !newer(batch.sequence, last_batch_))
            return false;
        last_batch_ = batch.sequence;
        seen_ = true;
    } else if (batch.kind == RawCanBatch::Kind::FullBus) {
        const unsigned c = batch.bus;
        if (c >= raw_cfg::bus_capacity) return false;
        if (full_seen_[c] && static_cast<uint16_t>(batch.sequence - last_full_cycle_[c]) == 0)
            return false;
        last_full_cycle_[c] = batch.sequence;
        full_seen_[c] = true;
    } else {
        return false;
    }
    return true;
}

unsigned RawCanGateway::reject(const RawCanBatch &batch)
{
    return accept_sequence(batch) && batch.valid && batch.count <= batch.capacity() ? batch.count : 0;
}

RawCanDrops RawCanGateway::process(const RawCanBatch &batch, const RawCanPolicy &policy, uint32_t now,
                                   RawCanSubmit submit)
{
    RawCanDrops drops{};
    const bool sideband = batch.kind != RawCanBatch::Kind::FullBus;
    if ((!sideband && !policy.permitted) || !accept_sequence(batch) ||
        !policy.permitted || !batch.valid || batch.count > batch.capacity())
        return drops;
    for (unsigned i = 0; i < batch.count; ++i) {
        const auto &slot = batch.slots[i];
        const unsigned c = sideband ? slot.bus : batch.bus;
        if (!send(c, slot.frame, batch.kind, now, policy, submit)) {
            ++drops.total;
            if (c < raw_cfg::bus_capacity)
                ++drops.channel[c];
        }
    }
    return drops;
}

}
