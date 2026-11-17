#include "raw_can_gateway.hpp"
#include "encos_parameter_transactions.hpp"
#include "ethercat_canfd_gateway.h"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include <cstring>

namespace dcu::app {
namespace motor_cfg = config::motor_control;
namespace motor_pdo = dcu::ethercat::motor_pdo;
using namespace dcu::ethercat::motor_dictionary;

namespace {
bool newer(uint32_t a, uint32_t b)
{
    uint32_t d = a - b;
    return d && d < 0x80000000u;
}
}

void RawCanGateway::reset_session()
{
    pending_ = false;
    seen_ = false;
    std::memset(full_seen_, 0, sizeof(full_seen_));
}
void RawCanGateway::stage_batch(uint32_t sequence, bool enabled)
{
    if (!seen_ || newer(sequence, last_batch_)) {
        last_batch_ = sequence;
        seen_ = true;
        pending_ = enabled;
    }
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
bool RawCanGateway::send(unsigned c, const dcu_can_frame_t &frame, const RawCanCycle &cycle, uint32_t now,
                         const mc_channel_config_t (&channels)[ENCOS_CHANNEL_COUNT],
                         const AxisController (&axes)[ENCOS_AXIS_COUNT],
                         BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT])
{
    const auto *f = &frame;
    if (c >= ENCOS_CHANNEL_COUNT || !(cycle.enabled_channels & (1u << c)) ||
        (channels[c].Role != 2 && channels[c].Role != 3) || !parameters::available(c, now))
        return false;
    if (cycle.profile != 2) {
        if (f->id == 0x7FF)
            return false;
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            if (axes[a].matches(f->id))
                return false;
    }
    if (now - budget_ms_[c] >= 2) {
        budget_ms_[c] = now;
        budget_[c] = 0;
    }
    if (budget_[c] >= channels[c].RawRateLimit) {
        return false;
    }
    unsigned cost = BusScheduler::frame_cost((f->flags & 8u) != 0, f->dlc) +
                    BusScheduler::frame_cost(channels[c].FrameFormat != 0, 8);
    if (!schedule[c].can_afford(cost) || !ethercat_canfd_send(c, f, ENCOS_AXIS_COUNT))
        return false;
    schedule[c].charge(cost);
    ++budget_[c];
    return true;
}
RawCanDrops RawCanGateway::process(const RawCanCycle &cycle, uint32_t now,
                                   const mc_channel_config_t (&channels)[ENCOS_CHANNEL_COUNT],
                                   const AxisController (&axes)[ENCOS_AXIS_COUNT],
                                   BusScheduler (&schedule)[ENCOS_CHANNEL_COUNT])
{
    RawCanDrops drops{};
    if (!pending_) {
        return drops;
    }
    pending_ = false;
    if (!cycle.permitted)
        return drops;
    if (cycle.profile == 1) {
        mc_raw_t *r = object<mc_raw_t>(0x7300);
        if ((r->ValidMask & ~3u) || r->SlotCount > 2 || r->Flags || r->DropCount || r->Reserved || r->Reserved1 || r->Reserved2)
            return drops;
        for (unsigned i = 0; i < 2; i++)
            if (r->ValidMask & (1u << i)) {
                auto slot = motor_pdo::raw_slot(*r, i);
                dcu_can_frame_t f{};
                f.id = slot.id;
                f.flags = slot.flags;
                f.dlc = slot.dlc;
                memcpy(f.data, slot.data, sizeof(f.data));
                if (!send(slot.channel, f, cycle, now, channels, axes, schedule)) {
                    ++drops.total;
                    if (slot.channel < ENCOS_CHANNEL_COUNT)
                        ++drops.channel[slot.channel];
                }
            }
    } else if (cycle.profile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
            auto raw = motor_pdo::raw_bus(cycle.pdo, c);
            const auto *p = &raw;
            *object<canfd_bus_pdo_t>(0x7400 + 16 * c) = raw;
            if (full_seen_[c] && (uint16_t)(p->cycle_seq - last_full_cycle_[c]) == 0)
                continue;
            last_full_cycle_[c] = p->cycle_seq;
            full_seen_[c] = true;
            if (p->valid_mask & 0x80u)
                continue;
            for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS; i++)
                if ((p->valid_mask & (1u << i)) && (p->slot[i].flags & 1)) {
                    const canfd_slot_pdo_t *s = &p->slot[i];
                    dcu_can_frame_t f{};
                    f.id = s->can_id;
                    f.dlc = s->dlc;
                    f.flags = static_cast<uint8_t>(s->flags & ~33u);
                    memcpy(f.data, s->data, 8);
                    if (!send(c, f, cycle, now, channels, axes, schedule)) {
                        ++drops.total;
                        ++drops.channel[c];
                    }
                }
        }
    }
    return drops;
}

}
