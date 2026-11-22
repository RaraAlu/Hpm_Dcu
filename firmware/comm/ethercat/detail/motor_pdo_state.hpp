#ifndef DCU_MOTOR_PDO_STATE_HPP
#define DCU_MOTOR_PDO_STATE_HPP

#include <cstdint>
#include "motor_pdo_layout.hpp"
#include "gateway_application.hpp"

namespace dcu::ethercat::motor_pdo::detail {

struct MotorSlot {
    mc_axis_tx_t *feedback;
    mc_diag_t *diagnostics;
    uint8_t *tx;
    uint16_t rx_offset;
    uint8_t slot, device;
};

struct ChannelSlot {
    mc_channel_tx_t *feedback;
    canfd_bus_pdo_t *raw;
    uint8_t *tx, *raw_tx;
};

struct PublicationState {
    uint8_t *image;
    uint8_t *imu_tx, *raw_tx;
    uint32_t raw_batch;
    uint32_t configured_slots;
    uint8_t round_robin, diagnostic_page, slot_count, layout;
    bool motor_layout, diagnostic_layout;
    mc_board_tx_t *board;
    ChannelSlot channels[ENCOS_CHANNEL_COUNT];
    MotorSlot slots[ENCOS_AXIS_COUNT];
    mc_imu_t *imu;
    mc_sideband_t *raw;
    mc_raw_t *single_fd;
    uint8_t slave_match;
    app::gateway_application::Publication snapshot;
};

}
#endif
