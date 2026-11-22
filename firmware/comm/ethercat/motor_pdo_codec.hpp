#ifndef DCU_MOTOR_PDO_CODEC_HPP
#define DCU_MOTOR_PDO_CODEC_HPP

#include <cstring>
#include "motor_pdo_layout.hpp"
#include "can/raw_can_gateway.hpp"
#include "gateway_application.hpp"

namespace dcu::ethercat::motor_pdo {
namespace layout = dcu::ethercat::pdo_layout;

struct SidebandSlot {
    uint32_t id;
    uint8_t channel, flags, dlc, reserved;
    uint8_t data[8];
};
static_assert(sizeof(SidebandSlot) == offsetof(mc_sideband_t, CanId1) - offsetof(mc_sideband_t, CanId0));

struct RawSlot {
    uint32_t id;
    uint8_t channel, flags, dlc, slave_match;
    uint8_t data[DCU_CAN_MAX_PAYLOAD];
};
static_assert(sizeof(RawSlot) == 72 && sizeof(RawSlot) == sizeof(mc_raw_t) - offsetof(mc_raw_t, CanId));
static_assert(offsetof(RawSlot, channel) == 4 && offsetof(RawSlot, flags) == 5 &&
              offsetof(RawSlot, dlc) == 6 && offsetof(RawSlot, slave_match) == 7 &&
              offsetof(RawSlot, data) == 8);

template <typename T> T load(const void *image, size_t offset)
{
    T value;
    std::memcpy(&value, static_cast<const uint8_t *>(image) + offset, sizeof(value));
    return value;
}
template <typename T> void store(void *image, size_t offset, const T &value)
{
    std::memcpy(static_cast<uint8_t *>(image) + offset, &value, sizeof(value));
}

inline mc_board_rx_t board(const void *image) { return load<mc_board_rx_t>(image, 0); }
inline RawSlot raw_slot(const mc_raw_t &wire) { return load<RawSlot>(&wire, offsetof(mc_raw_t, CanId)); }
inline void write_raw_slot(mc_raw_t &wire, const RawSlot &value) { store(&wire, offsetof(mc_raw_t, CanId), value); }
inline mc_channel_rx_t channel(const void *image, unsigned bus)
{ return load<mc_channel_rx_t>(image, layout::output_channels_offset + bus * sizeof(mc_channel_rx_t)); }
inline mc_axis_rx_t axis(const void *image, unsigned index)
{ return load<mc_axis_rx_t>(image, layout::output_axes_offset + index * sizeof(mc_axis_rx_t)); }
inline mc_sideband_t raw(const void *image) { return load<mc_sideband_t>(image, layout::motor_output_bytes); }
inline canfd_bus_pdo_t raw_bus(const void *image, unsigned bus)
{ return load<canfd_bus_pdo_t>(image, sizeof(mc_board_rx_t) + bus * sizeof(canfd_bus_pdo_t)); }
inline SidebandSlot sideband_slot(const mc_sideband_t &raw, unsigned slot)
{ return load<SidebandSlot>(&raw, offsetof(mc_sideband_t, CanId0) + slot * sizeof(SidebandSlot)); }
inline void write_sideband_slot(mc_sideband_t &raw, unsigned slot, const SidebandSlot &value)
{ store(&raw, offsetof(mc_sideband_t, CanId0) + slot * sizeof(SidebandSlot), value); }

inline void write_board(void *image, const mc_board_tx_t &value) { store(image, 0, value); }
inline void write_channel(void *image, unsigned bus, const mc_channel_tx_t &value)
{ store(image, layout::input_channels_offset + bus * sizeof(value), value); }
inline void write_axis(void *image, unsigned index, const mc_axis_tx_t &value)
{ store(image, layout::input_axes_offset + index * sizeof(value), value); }
inline void write_imu(void *image, const mc_imu_t &value) { store(image, layout::input_imu_offset, value); }
inline void write_raw(void *image, const mc_sideband_t &value) { store(image, layout::motor_input_bytes, value); }
inline void write_raw_bus(void *image, unsigned bus, const canfd_bus_pdo_t &value)
{ store(image, sizeof(mc_board_tx_t) + bus * sizeof(value), value); }

unsigned mapping_indices(unsigned profile, bool input, uint16_t *indices);
bool bind_publication();
void decode_raw_sideband(const mc_sideband_t &raw, app::RawCanBatch &batch);
void decode_raw_bus(const canfd_bus_pdo_t &raw, unsigned bus, app::RawCanBatch &batch);
void decode_cycle(const void *image, app::gateway_application::OperatingMode mode,
                  app::gateway_application::CycleCommand &command);

}
#endif
