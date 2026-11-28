#include "../detail/gateway_application_impl.hpp"
#include "realtime_port.h"
#include <cstring>

namespace dcu::app::gateway_application::detail {
bool GatewayApplication::select_mode(OperatingMode mode)
{
    if (mode > OperatingMode::SingleFd) return false;
    // 上电默认值只选择一次；后续换档只能由 PREOP 映射提交。
    if (layout_.selected || layout_.started || layout_.switching() || raw_state() == RawState::Faulted)
        return mode == state_.mode;
    if (state_.operational) return false;
    if (device_count_ && raw_only(mode)) clear_layout_business();
    state_.mode = mode;
    layout_.selected = true;
    return true;
}

uint32_t GatewayApplication::pure_raw_limits() const
{
    uint32_t packed = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c)
        packed |= uint32_t{layout_.limits[c]} << (8 * c);
    return packed;
}

bool GatewayApplication::set_pure_raw_limits(uint32_t packed)
{
    if (!state_.config_ready || layout_.topology || !preop_access_allowed()) return false;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        const auto limit = static_cast<uint8_t>(packed >> (8 * c));
        if (limit > 8 || (limit && !state_.channels[c].enabled) ||
            (state_.channels[c].enabled && dcu_can_pending(c, DCU_CAN_DEVICE_RAW))) return false;
    }
    checkpoint_raw_losses();
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c)
        layout_.limits[c] = static_cast<uint8_t>(packed >> (8 * c));
    return true;
}

bool GatewayApplication::raw_channel(unsigned bus) const
{
    return raw_only(state_.mode) ? layout_.limits[bus] != 0 : state_.channels[bus].role >= 2;
}

bool GatewayApplication::raw_motor_bus_idle(uint8_t buses) const
{
    if (buses >> DCU_DEVICE_CAN_BUS_COUNT) return false;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        if (!(buses & (1U << c))) continue;
        if (!state_.channels[c].enabled || !layout_.limits[c] || !bus_device_count_[c] ||
            dcu_can_pending(c, DCU_CAN_DEVICE_ANY)) return false;
    }
    return true;
}

bool GatewayApplication::stage_raw_motor_owner(uint8_t buses)
{
    if (!state_.config_ready || !preop_access_allowed() ||
        layout_.target != OperatingMode::SingleFd || raw_state() != RawState::Requested ||
        !raw_motor_bus_idle(buses)) return false;
    layout_.pending_motor_owner = buses;
    return true;
}

void GatewayApplication::clear_layout_business()
{
    revoke_business();
    for (auto &command : state_.channel_command) command = {};
    raw_.discard_frames();
    // 回切后不能用纯透传的本地就绪或旧电机反馈放行。
    startup_.invalidate(DCU_GATEWAY_REASON_MASTER_TIMEOUT, dcu_time_now_ms());
}

bool GatewayApplication::drain_raw()
{
    bool empty = true;
    for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus)
        if (state_.channels[bus].enabled && !dcu_can_drain_raw(bus)) empty = false;
    return empty;
}

bool GatewayApplication::request_layout(OperatingMode target, uint32_t profile, uint32_t revision,
                                       const uint8_t (&digest)[DCU_GATEWAY_DIGEST_BYTES])
{
    return begin_layout(target, profile, revision, digest, false);
}

bool GatewayApplication::request_topology()
{
    if (!preop_access_allowed()) return false;
    const auto active = readiness();
    return begin_layout(OperatingMode::MotorWithRaw, active.profile_id, active.revision, active.digest, true);
}

bool GatewayApplication::drain_layout()
{
    if (!layout_.topology) return drain_raw();
    bool idle = true;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        if (!catalogue_channels_[c].enabled) continue;
        collect_tx_events(c);
        idle &= !dcu_can_pending(c, DCU_CAN_DEVICE_ANY);
        dcu_can_frame_t frame;
        dcu_can_rx_stamp_t stamp;
        for (unsigned i = 0; dcu_can_receive(c, &frame, &stamp); ++i)
            if (i == 31) { idle = false; break; }
    }
    return idle;
}

bool GatewayApplication::begin_layout(OperatingMode target, uint32_t profile, uint32_t revision,
                                      const uint8_t (&digest)[DCU_GATEWAY_DIGEST_BYTES], bool topology)
{
    const auto active = readiness();
    const auto next = next_layout(topology ? PDO_SDO_2103_01_SELECT : PDO_SDO_2101_01_REQUEST);
    if (target > OperatingMode::SingleFd || (raw_only(target) && device_count_ && state_.operational) ||
        next == PDO_REFUSED ||
        !profile || profile != active.profile_id || revision != active.revision ||
        std::memcmp(active.digest, digest, sizeof(digest))) return false;
    layout_.target = target;
    if (observer_) observer_->stop();
    layout_.pending_motor_owner = 0;
    layout_.phase = next;
    layout_.revision = revision;
    layout_.drain_ms = dcu_time_now_ms();
    layout_.baseline = raw_drops();
    raw_.discard_frames();
    layout_.topology = topology;
    if (topology) {
        for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c)
            layout_.topology_request_rx[c] = channel_losses(c);
        clear_layout_business();
    }
    if (drain_layout()) layout_.phase = next_layout(PDO_CAN_DRAINED);
    return true;
}

bool GatewayApplication::reset_layout()
{
    const auto next = next_layout(PDO_SDO_2101_02_CONFIRM);
    if (next == PDO_REFUSED || !drain_layout()) return false;
    if (raw_state() == RawState::Requested) layout_.baseline = raw_drops();
    layout_.phase = next;
    return true;
}

bool GatewayApplication::map_layout(OperatingMode mode)
{
    return finish_layout(mode, false);
}

bool GatewayApplication::finish_layout(OperatingMode mode, bool topology)
{
    if (mode > OperatingMode::SingleFd) return false;
    const auto next = next_layout(topology ? PDO_SDO_2103_06_COMMIT : PDO_PREOP_MAPPING, mode);
    if (next == PDO_REFUSED) return false;
    if (layout_.switching()) {
        if (mode != layout_.target ||
            readiness().revision != layout_.revision || !drain_layout() ||
            !raw_motor_bus_idle(layout_.pending_motor_owner)) return false;
    } else {
        if (mode != state_.mode) return false;
        // 故障不妨碍电机用原布局重新上线，也不解除透传闭锁。
        if (raw_state() == RawState::Faulted) return true;
    }
    if (mode != state_.mode) {
        checkpoint_raw_losses();
        if (raw_only(mode) || raw_only(state_.mode)) clear_layout_business();
    }
    state_.mode = mode;
    if (layout_.switching()) layout_.motor_owner = layout_.pending_motor_owner;
    layout_.pending_motor_owner = 0;
    layout_.selected = true;
    layout_.topology = false;
    layout_.phase = next;
    return true;
}

uint32_t GatewayApplication::raw_drops() const
{
    uint32_t total = layout_.drops;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c)
        if (state_.channels[c].role >= 2) total += channel_losses(c);
        else if (raw_channel(c)) total += channel_losses(c) - layout_.loss_origin[c];
    return total;
}

uint32_t GatewayApplication::channel_losses(unsigned c) const
{
    return raw_.rx_drops(c) + state_.legacy_rx_drop[c] + dcu_can_rx_drops(c);
}

void GatewayApplication::checkpoint_raw_losses()
{
    // 临时授权电机通道时仅累计授权期间的丢帧，回切时计数不倒退。
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        if (!state_.channels[c].enabled || state_.channels[c].role >= 2) continue;
        const auto losses = channel_losses(c);
        if (raw_channel(c)) layout_.drops += losses - layout_.loss_origin[c];
        layout_.loss_origin[c] = losses;
    }
}

void GatewayApplication::reject_raw(const CycleCommand &candidate)
{
    const auto mode = state_.mode;
    if (mode == OperatingMode::Motor) return;
    const unsigned count = mode == OperatingMode::Raw ? DCU_DEVICE_CAN_BUS_COUNT : 1;
    for (unsigned i = 0; i < count; ++i)
        layout_.drops += raw_.reject(candidate.raw[i]);
}

void GatewayApplication::advance_layout(uint32_t now)
{
    if (raw_state() == RawState::Draining) {
        if (now - layout_.drain_ms >= state_.config.feedback_timeout_ms) layout_fault(PDO_DRAIN_TIMEOUT);
        else if (drain_layout()) layout_.phase = next_layout(PDO_CAN_DRAINED);
    } else if (layout_.topology && layout_.switching()) {
        // The host may spend many mailbox cycles assigning the new PDOs.
        // Passive CAN producers keep sending while all business is suspended.
        (void)drain_layout();
    }
    if (layout_.topology && raw_state() == RawState::Requested && now - layout_.drain_ms >= 60000U)
        layout_fault(PDO_TOPOLOGY_TIMEOUT);
    if ((layout_.phase == PDO_ACTIVE || layout_.switching()) &&
        raw_drops() - layout_.baseline > config::raw_can::drop_limit)
        layout_fault(PDO_DROP_LIMIT);
}

void GatewayApplication::layout_fault(dcu_layout_event_t cause)
{
    const auto next = next_layout(cause, state_.mode);
    if (next != PDO_FAULTED && next != PDO_DISABLED) return;
    if (observer_) observer_->stop();
    layout_.motor_owner = layout_.pending_motor_owner = 0;
    const bool discard = raw_state() != RawState::Disabled && raw_state() != RawState::Faulted;
    layout_.phase = next;
    if (!discard) return;
    raw_.discard_frames();
    drain_raw();
}

bool GatewayApplication::take_raw_frame(unsigned bus, dcu_can_frame_t &frame)
{ return layout_.phase == PDO_ACTIVE && raw_.take(bus, frame); }
}
