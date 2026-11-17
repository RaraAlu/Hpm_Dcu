#ifndef DCU_MOTOR_APPLICATION_HPP
#define DCU_MOTOR_APPLICATION_HPP

#include <cstdint>
#include "axis_contract.hpp"
#include "device_config.h"
#include "can_frame.h"
#include "motor_result.hpp"
#include "encos_parameter_transactions.hpp"
#include "raw_can_gateway.hpp"

namespace dcu::app::motor_application {

enum class OperatingMode : uint8_t { Motor, MotorWithRaw, Raw };

struct BoardConfiguration {
    uint32_t configured_axes{};
    uint16_t max_command_lease_ms{}, feedback_timeout_ms{};
};
struct ChannelConfiguration {
    uint8_t role{}, frame_format{};
    uint32_t nominal_bitrate{}, data_bitrate{}, schedule_us{};
    uint16_t raw_rate_limit{};
    uint8_t configured_motors{};
};
struct Configuration {
    BoardConfiguration board{};
    ChannelConfiguration channels[ENCOS_CHANNEL_COUNT]{};
    AxisConfiguration axes[ENCOS_AXIS_COUNT]{};
};
struct BoardCommand {
    uint32_t sequence{}, enabled_axes{};
    uint16_t control_bits{}, lease_ms{};
    uint8_t enabled_channels{};
    // The adapter checks version/reserved fields; stop intent is still honored when false.
    bool layout_valid{};
};
struct ChannelCommand {
    uint16_t control_bits{};
    uint8_t active_motors{}, expected_role{};
    uint32_t period_us{};
};
struct CycleCommand {
    BoardCommand board{};
    ChannelCommand channels[ENCOS_CHANNEL_COUNT]{};
    motor::Command axes[ENCOS_AXIS_COUNT]{};
    RawCanBatch raw[ENCOS_CHANNEL_COUNT]{};
};
struct ProcessResult {
    bool consumed{}, accepted{}, raw_mirrored{};
    uint32_t mirrored_axes{};
};
struct SdoCommand {
    motor::Command command{};
    uint32_t sequence{};
    uint16_t lease_ms{};
};
struct CommandResult {
    ResultCode result{RESULT_INVALID};
    uint32_t accepted_sequence{};
    bool mirror_command{};
    bool changed{true};
};
struct Publication {
    OperatingMode mode{};
    uint32_t sequence{}, accepted_cycle_sequence{}, cycle_sequence{};
    uint16_t board_status{};
    uint8_t online_channels{}, fault_channels{};
    uint32_t online_axes{}, fault_axes{}, rx_drops{}, tx_drops{};
    struct Channel {
        uint16_t status{};
        uint8_t role{}, online_motors{}, fault_motors{};
        uint32_t rx_drops{}, tx_drops{}, raw_drops{};
    } channels[ENCOS_CHANNEL_COUNT]{};
    struct Axis {
        AxisFeedback feedback{};
        AxisDiagnostics diagnostics{};
        uint32_t type1_count{};
    } axes[ENCOS_AXIS_COUNT]{};
};

// One task owns all operations. Inputs are borrowed only for the duration of a call.
void initialize(const Configuration &defaults);
ProcessResult process(const CycleCommand *candidate);
void operational(bool operational);
void cycle_fault();
bool configuration_writable();
bool owner_change_allowed();
bool transaction_busy(unsigned axis);
bool apply_configuration(const Configuration &configuration);
bool select_mode(OperatingMode mode);
OperatingMode mode();
bool set_owner(Owner owner);
Owner owner();
CommandResult commit_sdo_command(unsigned axis, const SdoCommand &command, uint32_t now_ms);
parameters::CommitOutcome commit_parameter(const parameters::Request &request, uint32_t now_ms, bool preop);
// A read does not acknowledge events; only publication does.
bool read_axis(unsigned axis, AxisFeedback &feedback, AxisDiagnostics &diagnostics);
void prepare_publication(Publication &snapshot);
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

}
#endif
