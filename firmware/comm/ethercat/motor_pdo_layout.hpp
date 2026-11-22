#ifndef DCU_MOTOR_PDO_LAYOUT_HPP
#define DCU_MOTOR_PDO_LAYOUT_HPP

#include <cstddef>
#include <cstdint>
#include "encos_interface.h"
#include "raw_can_pdo.h"
#include "device_config.h"

namespace dcu::ethercat::pdo_layout {

struct ProfileSize {
    uint16_t input;
    uint16_t output;
};

inline constexpr uint16_t motor_input_bytes = sizeof(mc_board_tx_t) +
    ENCOS_CHANNEL_COUNT * sizeof(mc_channel_tx_t) + ENCOS_AXIS_COUNT * sizeof(mc_axis_tx_t) + sizeof(mc_imu_t);
inline constexpr uint16_t motor_output_bytes = sizeof(mc_board_rx_t) +
    ENCOS_CHANNEL_COUNT * sizeof(mc_channel_rx_t) + ENCOS_AXIS_COUNT * sizeof(mc_axis_rx_t);
inline constexpr ProfileSize profiles[] = {
    {motor_input_bytes, motor_output_bytes},
    {motor_input_bytes + sizeof(mc_sideband_t), motor_output_bytes + sizeof(mc_sideband_t)},
    {sizeof(mc_board_tx_t) + sizeof(ethercat_canfd_process_image_t),
     sizeof(mc_board_rx_t) + sizeof(ethercat_canfd_process_image_t)},
    {sizeof(mc_raw_t), sizeof(mc_raw_t)},
};
inline constexpr unsigned profile_count = sizeof(profiles) / sizeof(profiles[0]);
constexpr ProfileSize maximum_profile_size()
{
    ProfileSize maximum{};
    for (const auto &profile : profiles) {
        if (profile.input > maximum.input) maximum.input = profile.input;
        if (profile.output > maximum.output) maximum.output = profile.output;
    }
    return maximum;
}
inline constexpr uint16_t max_input_bytes = maximum_profile_size().input;
inline constexpr uint16_t max_output_bytes = maximum_profile_size().output;

inline constexpr size_t output_channels_offset = sizeof(mc_board_rx_t);
inline constexpr size_t output_axes_offset = output_channels_offset + ENCOS_CHANNEL_COUNT * sizeof(mc_channel_rx_t);
inline constexpr size_t input_channels_offset = sizeof(mc_board_tx_t);
inline constexpr size_t input_axes_offset = input_channels_offset + ENCOS_CHANNEL_COUNT * sizeof(mc_channel_tx_t);
inline constexpr size_t input_imu_offset = input_axes_offset + ENCOS_AXIS_COUNT * sizeof(mc_axis_tx_t);
inline constexpr unsigned assignment_capacity = 40;

// These are the frozen on-wire layouts, not scheduling configuration.
static_assert(CANFD_GATEWAY_BUS_COUNT == ENCOS_CHANNEL_COUNT);
static_assert(profiles[0].input == 1024 && profiles[0].output == 736);
static_assert(profiles[1].input == 1072 && profiles[1].output == 784);
static_assert(profiles[2].input == 544 && profiles[2].output == 544);
static_assert(profiles[3].input == 88 && profiles[3].output == 88);

}
#endif
