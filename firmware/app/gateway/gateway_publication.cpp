#include "detail/gateway_application_impl.hpp"
#include "cycle_config.h"
#include "realtime_port.h"

namespace dcu::app::gateway_application::detail {

bool GatewayApplication::neutral_feedback_pending() const
{
    return neutral_run_.awaiting || (neutral_run_.active & ~neutral_run_.sent_this_cycle);
}

DCU_FAST_CODE void GatewayApplication::prepare_publication(Publication &snapshot)
{
    const uint32_t now = dcu_time_now_ms();
    uint32_t active = 0;
    snapshot.online_devices = snapshot.fault_devices = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        for (auto mask = routes_[c].members; mask; mask &= mask - 1U) {
            const unsigned a = __builtin_ctz(mask);
            const auto bit = 1U << a;
            if (devices_[a].active()) active |= bit;
            if (raw_only(state_.mode)) continue;
            auto &device = snapshot.devices[a];
            const auto status = devices_[a].read_feedback(now, state_.config.feedback_timeout_ms,
                                                          device.feedback, device.diagnostics);
            devices_[a].acknowledge_publication();
            if (status.online) snapshot.online_devices |= bit;
            if (status.fault) snapshot.fault_devices |= bit;
        }
    }
    prepare_header(snapshot, now, active);
}

DCU_FAST_CODE bool GatewayApplication::stage_device(unsigned a, uint32_t now)
{
    Publication::Device sample;
    const auto status = devices_[a].read_feedback(now, state_.config.feedback_timeout_ms,
                                                  sample.feedback, sample.diagnostics);
    if (!feedback_writer_ || !feedback_writer_(a, sample, status.online, status.fault)) return false;
    devices_[a].acknowledge_publication();
    const auto bit = 1U << a;
    cyclic_.active = (cyclic_.active & ~bit) | (devices_[a].active() ? bit : 0);
    cyclic_.online = (cyclic_.online & ~bit) | (status.online ? bit : 0);
    cyclic_.faults = (cyclic_.faults & ~bit) | (status.fault ? bit : 0);
    return true;
}

DCU_FAST_CODE bool GatewayApplication::finish_cyclic_publication(Publication &snapshot)
{
    if (!cyclic_.required || cyclic_feedback_missing() || !cyclic_.image_valid || raw_only(state_.mode)) return false;
    snapshot.online_devices = cyclic_.online;
    snapshot.fault_devices = cyclic_.faults;
    prepare_header(snapshot, dcu_time_now_ms(), cyclic_.active);
    return true;
}

DCU_FAST_CODE void GatewayApplication::prepare_header(Publication &snapshot, uint32_t now, uint32_t active)
{
    snapshot.mode = state_.mode;
    snapshot.sequence = ++state_.publication_sequence;
    snapshot.accepted_cycle_sequence = state_.last_cycle;
    snapshot.cycle_sequence = state_.board_command.sequence;
    snapshot.board_status = state_.board_fault | (state_.config_ready ? 1 : 0) | (state_.operational ? 2 : 0);
    snapshot.online_channels = snapshot.fault_channels = 0;
    uint32_t rx_drops_total = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; c++) {
        auto *ch = &snapshot.channels[c];
        *ch = {};
        const uint32_t compatibility_drops = raw_.rx_drops(c) + state_.legacy_rx_drop[c];
        uint32_t drops = dcu_can_rx_drops(c) + compatibility_drops;
        rx_drops_total += drops;
        if (state_.channels[c].enabled) {
            snapshot.board_status |= state_.bus_fault[c];
            if (drops != state_.rx_fault_origin[c]) snapshot.board_status |= 16;
        }
        const auto members = routes_[c].members;
        ch->status = (state_.config_ready ? 1 : 0) | ((members & active) ? 2 : 0) | (dcu_can_is_bus_off_fast(c) ? 4 : 0) | (drops ? 8 : 0) | (state_.channel_tx_drop[c] ? 16 : 0);
        if (observer_ && observer_->bus == c) ch->status |= observer_->channel_status(now);
        ch->rx_drops = drops;
        ch->tx_drops = state_.channel_tx_drop[c];
        ch->role = state_.channels[c].role;
        ch->raw_drops = compatibility_drops;
        if (members & snapshot.online_devices) snapshot.online_channels |= 1U << c;
        if (members & snapshot.fault_devices) snapshot.fault_channels |= 1U << c;
        if (ch->status & 4) snapshot.fault_channels |= 1u << c;
    }
    snapshot.rx_drops = rx_drops_total;
    snapshot.tx_drops = state_.tx_total;
}

}
