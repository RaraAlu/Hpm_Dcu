#ifndef DCU_MOTOR_APPLICATION_HPP
#define DCU_MOTOR_APPLICATION_HPP

#include <cstdint>
#include "encos_interface.h"
#include "device_config.h"
#include "can_frame.h"

namespace dcu::app::motor_application {

// Stable bridge result/owner codes, shared with the existing EtherCAT contract.
enum ResultCode : uint16_t {
    RESULT_OK, RESULT_BUSY, RESULT_INVALID, RESULT_STATE,
    RESULT_TIMEOUT, RESULT_UNSUPPORTED, RESULT_MISMATCH, RESULT_IO
};
enum Owner : uint8_t { OWNER_NONE, OWNER_PDO, OWNER_SDO };

// Task-only application operations used by the EtherCAT adapter. No SSC types.
bool configuration_writable();
bool owner_change_allowed();
bool transaction_busy(unsigned axis);
bool apply_configuration();
bool select_profile(unsigned profile);
void owner_changed();
void commit_sdo_command(unsigned axis, mc_sdo_command_t &command, uint32_t now_ms);
bool commit_parameter(unsigned axis, bool maintenance, uint32_t now_ms, bool preop);
// Refresh the read-only EtherCAT mirror without consuming PDO event flags.
void refresh_axis_feedback(unsigned axis);

struct PdoPublication {
    unsigned profile;
    uint32_t cycle_sequence;
    uint8_t roles[ENCOS_CHANNEL_COUNT];
    uint32_t raw_drops[ENCOS_CHANNEL_COUNT];
    uint32_t type1_count[ENCOS_AXIS_COUNT];
};
PdoPublication prepare_publication();
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

}
#endif
