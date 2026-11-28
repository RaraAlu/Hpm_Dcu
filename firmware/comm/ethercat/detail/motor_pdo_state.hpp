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
    uint8_t *raw_tx;
    pdo_layout::Layout wire;
    pdo_layout::MotorOffsets motor_rx;
    uint32_t raw_batch;
    uint32_t online_bits, fault_bits;
    uint8_t binding_for_device[DCU_MOTOR_DEVICE_CAPACITY]; // One-based; zero is unmapped.
    uint8_t round_robin, slot_count, layout;
    bool motor_layout;
    mc_board_tx_t *board;
    ChannelSlot channels[ENCOS_CHANNEL_COUNT];
    MotorSlot slots[ENCOS_AXIS_COUNT];
    pdo_layout::MappedSideband *raw;
    mc_event_data_t *single_fd[ENCOS_CHANNEL_COUNT];
    mc_event_header_t *event_header;
    pdo_layout::Layout event;
    uint8_t slave_match;
    app::gateway_application::Publication snapshot;
};

}
#endif
