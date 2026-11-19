#include "detail/gateway_application_impl.hpp"
#include "realtime_port.h"

namespace dcu::app::gateway_application::detail {

void GatewayApplication::prepare_publication(Publication &snapshot)
{
    const uint32_t now = dcu_time_now_ms();
    snapshot.mode = state_.mode;
    snapshot.sequence = ++state_.publication_sequence;
    snapshot.accepted_cycle_sequence = state_.last_cycle;
    snapshot.cycle_sequence = state_.board_command.sequence;
    snapshot.board_status = state_.board_fault | (state_.config_ready ? 1 : 0) | (state_.operational ? 2 : 0) | (state_.owner == OWNER_SDO ? 128 : 0);
    snapshot.online_devices = snapshot.fault_devices = 0;
    snapshot.online_channels = snapshot.fault_channels = 0;
    uint32_t rx_drops_total = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; c++) {
        auto *ch = &snapshot.channels[c];
        *ch = {};
        const uint32_t compatibility_drops = raw_.rx_drops(c) + state_.legacy_rx_drop[c];
        uint32_t drops = dcu_can_rx_drops(c) + compatibility_drops;
        rx_drops_total += drops;
        ch->status = (state_.config_ready ? 1 : 0) | (channel_active(c) ? 2 : 0) | (dcu_can_bus_state(c) == 3 ? 4 : 0) | (drops ? 8 : 0) | (state_.channel_tx_drop[c] ? 16 : 0);
        ch->rx_drops = drops;
        ch->tx_drops = state_.channel_tx_drop[c];
        ch->role = state_.channels[c].role;
        ch->raw_drops = compatibility_drops;
        for (unsigned j = 0; j < bus_device_count_[c]; ++j) {
            const unsigned a = devices_on_bus_[c][j];
            auto &device = snapshot.devices[a];
            const auto status = devices_[a].read_feedback(now, state_.config.feedback_timeout_ms,
                                                          device.feedback, device.diagnostics);
            device.cyclic_feedback_count = devices_[a].cyclic_feedback_count();
            devices_[a].acknowledge_publication();
            if (status.online) {
                snapshot.online_devices |= 1u << a;
                snapshot.online_channels |= 1u << c;
            }
            if (status.fault) {
                snapshot.fault_devices |= 1u << a;
                snapshot.fault_channels |= 1u << c;
            }
        }
        if (ch->status & 4) snapshot.fault_channels |= 1u << c;
    }
    snapshot.rx_drops = rx_drops_total;
    snapshot.tx_drops = state_.tx_total;
    if (rx_drops_total) {
        state_.board_fault |= 16;
        snapshot.board_status |= 16;
    }
}

}
