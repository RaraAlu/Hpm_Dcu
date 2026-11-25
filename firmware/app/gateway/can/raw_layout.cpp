#include "../detail/gateway_application_impl.hpp"
#include "realtime_port.h"
#include <cstring>

namespace dcu::app::gateway_application::detail {
bool GatewayApplication::select_mode(OperatingMode mode)
{
    if (mode > OperatingMode::SingleFd) return false;
    // 上电默认值只选择一次；后续换档只能由 PREOP 映射提交。
    if (layout_.selected || layout_.started || layout_.switching || layout_.state == RawState::Faulted)
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
    if (!state_.config_ready || !preop_access_allowed() || !layout_.switching ||
        layout_.target != OperatingMode::SingleFd || layout_.state != RawState::Requested ||
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
    const bool recovering = layout_.state == RawState::Faulted;
    if (target > OperatingMode::SingleFd || (raw_only(target) && device_count_ && state_.operational) ||
        layout_.switching || (layout_.topology && !topology) ||
        (recovering && (!layout_.reset || state_.operational)) || !active.boot_id ||
        !profile || profile != active.profile_id || revision != active.revision ||
        std::memcmp(active.digest, digest, sizeof(digest))) return false;
    layout_.target = target;
    layout_.pending_motor_owner = 0;
    layout_.switching = true;
    layout_.reset = recovering;
    layout_.session = active.session_id;
    layout_.revision = revision;
    layout_.drain_ms = dcu_time_now_ms();
    layout_.baseline = raw_drops();
    raw_.discard_frames();
    layout_.topology = topology;
    if (topology) clear_layout_business();
    layout_.state = drain_layout() ? RawState::Requested : RawState::Draining;
    return true;
}

bool GatewayApplication::reset_layout()
{
    if (state_.operational || (layout_.state != RawState::Requested && layout_.state != RawState::Faulted) ||
        !drain_layout()) return false;
    if (layout_.state == RawState::Faulted) {
        // 复位不恢复旧授权；主站还要提交当前 Profile 的摘要及目标布局。
        layout_.reset = true;
        return true;
    }
    if (!layout_.switching || layout_.state != RawState::Requested) return false;
    layout_.reset = true;
    layout_.baseline = raw_drops();
    return true;
}

bool GatewayApplication::map_layout(OperatingMode mode)
{
    return finish_layout(mode, false);
}

bool GatewayApplication::finish_layout(OperatingMode mode, bool topology)
{
    if (state_.operational || mode > OperatingMode::SingleFd || topology != layout_.topology)
        return false;
    if (layout_.switching) {
        if (layout_.state != RawState::Requested || !layout_.reset || mode != layout_.target ||
            readiness().revision != layout_.revision || !drain_layout() ||
            !raw_motor_bus_idle(layout_.pending_motor_owner)) return false;
    } else {
        if (mode != state_.mode) return false;
        // 故障不妨碍电机用原布局重新上线，也不解除透传闭锁。
        if (layout_.state == RawState::Faulted) return true;
    }
    if (mode != state_.mode) {
        checkpoint_raw_losses();
        if (raw_only(mode) || raw_only(state_.mode)) clear_layout_business();
    }
    state_.mode = mode;
    if (layout_.switching) layout_.motor_owner = layout_.pending_motor_owner;
    layout_.pending_motor_owner = 0;
    layout_.selected = true;
    layout_.switching = false;
    layout_.topology = false;
    layout_.state = mode == OperatingMode::Motor ? RawState::Disabled : RawState::Armed;
    return true;
}

void GatewayApplication::activate_layout()
{
    layout_.state = RawState::Active;
    layout_.session = readiness().session_id;
    layout_.baseline = raw_drops();
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
    if (layout_.state == RawState::Draining) {
        if (now - layout_.drain_ms >= state_.config.feedback_timeout_ms) layout_fault();
        else if (drain_layout()) layout_.state = RawState::Requested;
    }
    if (layout_.topology && layout_.state == RawState::Requested && now - layout_.drain_ms >= 60000U)
        layout_fault();
    if ((layout_.state == RawState::Active || layout_.switching) &&
        raw_drops() - layout_.baseline > config::raw_can::drop_limit)
        layout_fault();
}

void GatewayApplication::layout_fault()
{
    layout_.reset = false;
    layout_.motor_owner = layout_.pending_motor_owner = 0;
    if (layout_.state == RawState::Disabled || layout_.state == RawState::Faulted) return;
    layout_.state = RawState::Faulted;
    layout_.switching = false;
    raw_.discard_frames();
    drain_raw();
}

bool GatewayApplication::take_raw_frame(unsigned bus, dcu_can_frame_t &frame)
{ return layout_.state == RawState::Active && raw_.take(bus, frame); }
}
