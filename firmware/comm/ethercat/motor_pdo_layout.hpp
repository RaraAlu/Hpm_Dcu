#ifndef DCU_MOTOR_PDO_LAYOUT_HPP
#define DCU_MOTOR_PDO_LAYOUT_HPP

#include <cstdint>
#include <type_traits>
#include <dcu_profile_pdo.hpp>
#include <dcu_can_event.hpp>
#include "encos_interface.h"
#include "raw_can_pdo.h"
#include "device_config.h"

// Fixed motor PDO/CoE slot numbering, not a property of every CAN peripheral.
#define ENCOS_CHANNEL_COUNT DCU_DEVICE_CAN_BUS_COUNT
#define ENCOS_MOTORS_PER_CHANNEL DCU_MOTOR_DEVICES_PER_BUS
#define ENCOS_AXIS_COUNT DCU_MOTOR_DEVICE_CAPACITY

namespace dcu::ethercat::pdo_layout {

inline constexpr unsigned profile_count = 4, assignment_capacity = 44;
inline constexpr uint16_t absent = UINT16_MAX;
using MappedSideband = std::conditional_t<(dcu_profile_pdo::sideband_bus >= 0), mc_classic_slot_t, mc_sideband_t>;
constexpr unsigned mapped_size(const dcu_profile_pdo::Image &image, unsigned object)
{
    for (unsigned i = 0; i < image.count; ++i)
        if (image.blocks[i].object == object) return image.blocks[i].bytes;
    return 0;
}
static_assert(sizeof(MappedSideband) == mapped_size(dcu_profile_pdo::modes[1].rx, 0x7300));
static_assert(sizeof(MappedSideband) == mapped_size(dcu_profile_pdo::modes[1].tx, 0x6400));

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
    template<class Visit>
    constexpr unsigned visit(unsigned mode, bool tx, Visit visit_block) const
    {
        const auto &image = tx ? dcu_profile_pdo::modes[mode].tx : dcu_profile_pdo::modes[mode].rx;
        unsigned pos = 0;
        for (unsigned i = 0; i < image.count; ++i) {
            const auto &block = image.blocks[i];
            const unsigned n = block.mapping - (tx ? 0x1a00 : 0x1600);
            if (n >= 1 && n <= ENCOS_CHANNEL_COUNT && !(buses & (1U << (n - 1)))) continue;
            if (n >= 0x20 && n < 0x20 + ENCOS_AXIS_COUNT && !((tx ? feedback_axes : command_axes) & (1U << (n - 0x20)))) continue;
            if (n >= 0x50 && n < 0x50 + ENCOS_CHANNEL_COUNT && !(buses & (1U << (n - 0x50)))) continue;
            if (n >= 0x60 && n < 0x60 + ENCOS_CHANNEL_COUNT && !(buses & (1U << (n - 0x60)))) continue;
            visit_block(block, pos);
            pos += block.bytes;
        }
        return pos;
    }
    uint16_t raw_bus(bool tx, unsigned c) const
    {
        return offset(buses, c, tx ? sizeof(mc_board_tx_t) : sizeof(mc_board_rx_t), sizeof(canfd_bus_pdo_t));
    }
    uint16_t bytes(unsigned mode, bool tx) const
    {
        if (mode == 3) return dcu_can_event::bytes(buses);
        if (mode == 2) return (tx ? sizeof(mc_board_tx_t) : sizeof(mc_board_rx_t)) + count(buses) * sizeof(canfd_bus_pdo_t);
        return (tx ? sizeof(mc_board_tx_t) + count(buses) * sizeof(mc_channel_tx_t) + count(feedback_axes) * sizeof(mc_axis_tx_t) :
                     sizeof(mc_board_rx_t) + count(buses) * sizeof(mc_channel_rx_t) + count(command_axes) * sizeof(mc_axis_rx_t)) +
               (mode == 1 ? sizeof(MappedSideband) : 0U);
    }
};

// Bound when the PREOP assignment changes, never scanned during a control cycle.
struct MotorOffsets {
    uint16_t channels[ENCOS_CHANNEL_COUNT], motors[ENCOS_AXIS_COUNT], sideband;
    void bind(const Layout &layout, unsigned mode, bool tx);
};
const MotorOffsets &input_offsets();

static_assert(CANFD_GATEWAY_BUS_COUNT == ENCOS_CHANNEL_COUNT);
// Local SSC/staging capacities, never a mapping or an amount to copy.
inline constexpr unsigned max_output_bytes = sizeof(mc_board_rx_t) + ENCOS_CHANNEL_COUNT * sizeof(mc_channel_rx_t)
    + ENCOS_AXIS_COUNT * sizeof(mc_axis_rx_t) + sizeof(mc_sideband_t);
inline constexpr unsigned max_input_bytes = sizeof(mc_board_tx_t) + ENCOS_CHANNEL_COUNT * sizeof(mc_channel_tx_t)
    + ENCOS_AXIS_COUNT * sizeof(mc_axis_tx_t) + sizeof(mc_sideband_t);
static_assert(sizeof(mc_board_rx_t) + sizeof(ethercat_canfd_process_image_t) <= max_output_bytes);
static_assert(sizeof(mc_board_tx_t) + sizeof(ethercat_canfd_process_image_t) <= max_input_bytes);

const Layout &active();
const Layout &event_layout();
}
#endif
