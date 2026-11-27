#ifndef DCU_MOTOR_PDO_LAYOUT_HPP
#define DCU_MOTOR_PDO_LAYOUT_HPP

#include <cstdint>
#include <type_traits>
#include <dcu_profile_pdo.hpp>
#include "encos_interface.h"
#include "raw_can_pdo.h"
#include "device_config.h"

namespace dcu::ethercat::pdo_layout {

inline constexpr unsigned profile_count = 4, assignment_capacity = 40;
inline constexpr uint16_t absent = UINT16_MAX;
using MappedSideband = std::conditional_t<(dcu_profile_pdo::sideband_bus >= 0), mc_classic_slot_t, mc_sideband_t>;
static_assert(sizeof(MappedSideband) == dcu_profile_pdo::rx1_blocks[dcu_profile_pdo::modes[1].rx.count - 1].bytes);
static_assert(sizeof(MappedSideband) == dcu_profile_pdo::tx1_blocks[dcu_profile_pdo::modes[1].tx.count - 1].bytes);

// Bits and CoE indices retain their physical identities; only the wire is dense.
struct Layout {
    uint32_t buses{}, command_axes{}, feedback_axes{};

    static unsigned count(uint32_t bits) { return __builtin_popcount(bits); }
    static uint16_t offset(uint32_t mask, unsigned id, unsigned base, unsigned stride)
    {
        if (id >= 32) return absent;
        const auto bit = uint32_t{1} << id;
        return mask & bit ? static_cast<uint16_t>(base + count(mask & (bit - 1U)) * stride) : absent;
    }
    uint16_t channel(bool tx, unsigned c) const
    {
        return offset(buses, c, tx ? sizeof(mc_board_tx_t) : sizeof(mc_board_rx_t),
                      tx ? sizeof(mc_channel_tx_t) : sizeof(mc_channel_rx_t));
    }
    uint16_t axes_begin(bool tx) const
    {
        return tx ? sizeof(mc_board_tx_t) + count(buses) * sizeof(mc_channel_tx_t) :
                    sizeof(mc_board_rx_t) + count(buses) * sizeof(mc_channel_rx_t);
    }
    uint16_t axis(bool tx, unsigned a) const
    {
        return offset(tx ? feedback_axes : command_axes, a, axes_begin(tx),
                      tx ? sizeof(mc_axis_tx_t) : sizeof(mc_axis_rx_t));
    }
    uint16_t sideband(bool tx) const
    {
        return axes_begin(tx) + (tx ? count(feedback_axes) * sizeof(mc_axis_tx_t) :
                                      count(command_axes) * sizeof(mc_axis_rx_t));
    }
    uint16_t raw_bus(bool tx, unsigned c) const
    {
        return offset(buses, c, tx ? sizeof(mc_board_tx_t) : sizeof(mc_board_rx_t), sizeof(canfd_bus_pdo_t));
    }
    uint16_t bytes(unsigned mode, bool tx) const
    {
        if (mode == 3) return sizeof(mc_raw_t);
        if (mode == 2) return (tx ? sizeof(mc_board_tx_t) : sizeof(mc_board_rx_t)) + count(buses) * sizeof(canfd_bus_pdo_t);
        return sideband(tx) + (mode == 1 ? sizeof(MappedSideband) : 0U);
    }
};

static_assert(CANFD_GATEWAY_BUS_COUNT == ENCOS_CHANNEL_COUNT);
// Local SSC/staging capacities, never a mapping or an amount to copy.
inline constexpr unsigned max_output_bytes = sizeof(mc_board_rx_t) + ENCOS_CHANNEL_COUNT * sizeof(mc_channel_rx_t)
    + ENCOS_AXIS_COUNT * sizeof(mc_axis_rx_t) + sizeof(mc_sideband_t);
inline constexpr unsigned max_input_bytes = sizeof(mc_board_tx_t) + ENCOS_CHANNEL_COUNT * sizeof(mc_channel_tx_t)
    + ENCOS_AXIS_COUNT * sizeof(mc_axis_tx_t) + sizeof(mc_sideband_t);
static_assert(sizeof(mc_board_rx_t) + sizeof(ethercat_canfd_process_image_t) <= max_output_bytes);
static_assert(sizeof(mc_board_tx_t) + sizeof(ethercat_canfd_process_image_t) <= max_input_bytes);

const Layout &active();
}
#endif
