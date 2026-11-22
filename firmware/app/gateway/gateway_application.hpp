#ifndef DCU_GATEWAY_APPLICATION_HPP
#define DCU_GATEWAY_APPLICATION_HPP

#include <cstdint>
#include "motor_feedback.hpp"
#include "device_config.h"
#include "can_frame.h"
#include "motor_result.hpp"
#include "can/raw_can_gateway.hpp"
#include "production_profile.h"
#include "startup/gateway_readiness.h"
#include "policy/profile_update.hpp"

namespace dcu::app::gateway_application {

enum class OperatingMode : uint8_t { Motor, MotorWithRaw, Raw };
enum Owner : uint8_t { OWNER_NONE, OWNER_PDO, OWNER_SDO };

struct ChannelConfiguration {
    uint8_t role{}, frame_format{};
    uint16_t raw_rate_limit{};
    bool enabled{};
};
struct BoardCommand {
    uint32_t sequence{}, enabled_devices{};
    uint16_t control_bits{}, lease_ms{};
    uint8_t enabled_channels{};
    // The adapter checks version/reserved fields; stop intent is still honored when false.
    bool layout_valid{}, unmapped_device_requested{};
};
struct ChannelCommand {
    uint16_t control_bits{};
    uint8_t expected_role{};
    bool active_mask_valid{true};
    uint32_t period_us{};
};
struct CycleCommand {
    BoardCommand board{};
    ChannelCommand channels[DCU_DEVICE_CAN_BUS_COUNT]{};
    motor::Command devices[DCU_MOTOR_DEVICE_CAPACITY]{};
    uint32_t active_devices{};
    RawCanBatch raw[DCU_DEVICE_CAN_BUS_COUNT]{};
};
struct ProcessResult {
    bool consumed{}, accepted{}, raw_mirrored{};
    uint32_t mirrored_devices{};
};
struct SdoCommand {
    motor::Command command{};
    uint32_t sequence{};
    uint16_t lease_ms{};
};
struct CommandResult {
    motor::ResultCode result{motor::RESULT_INVALID};
    uint32_t accepted_sequence{};
    bool mirror_command{};
    bool changed{true};
};
struct ParameterRead {
    uint32_t sequence{}, done_sequence{};
    uint16_t parameter{};
};
struct ParameterReadResult {
    bool accepted{}, changed{};
    uint16_t result{};
    uint32_t done_sequence{};
};
struct Publication {
    OperatingMode mode{};
    uint32_t sequence{}, accepted_cycle_sequence{}, cycle_sequence{};
    uint16_t board_status{};
    uint8_t online_channels{}, fault_channels{};
    uint32_t online_devices{}, fault_devices{}, rx_drops{}, tx_drops{};
    struct Channel {
        uint16_t status{};
        uint8_t role{};
        uint32_t rx_drops{}, tx_drops{}, raw_drops{};
    } channels[DCU_DEVICE_CAN_BUS_COUNT]{};
    struct Device {
        motor::MotorFeedback feedback{};
        motor::MotorDiagnostics diagnostics{};
        uint32_t cyclic_feedback_count{};
    } devices[DCU_MOTOR_DEVICE_CAPACITY]{};
};

// One task owns all operations. Inputs are borrowed only for the duration of a call.
bool initialize_profile(const dcu_production_profile_t &profile);
ChannelConfiguration channel_configuration(unsigned bus);
bool boot_startup(const dcu_gateway_profile_t &, uint64_t boot_id);
bool startup_enabled();
dcu::app::profile_update::Policy profile_policy();
bool apply_profile_policy(uint32_t profile, uint32_t base, uint32_t revision, const profile_update::Policy &);
dcu_gateway_readiness_t readiness();
ProcessResult process(const CycleCommand *candidate);
void operational(bool operational);
void cycle_fault();
bool preop_access_allowed();
bool owner_change_allowed();
bool select_mode(OperatingMode mode);
OperatingMode mode();
bool set_owner(Owner owner);
Owner owner();
CommandResult commit_sdo_command(unsigned device, const SdoCommand &command, uint32_t now_ms);
bool parameter_service_busy(unsigned bus);
ParameterReadResult read_parameter(unsigned device, const ParameterRead &, uint32_t now_ms, bool preop);
// A read does not acknowledge events; only publication does.
bool read_device(unsigned device, motor::MotorFeedback &feedback, motor::MotorDiagnostics &diagnostics);
void prepare_publication(Publication &snapshot);
bool neutral_feedback_pending();
void set_neutral_window(uint32_t period_ns, uint32_t cutoff_us);
void neutral_feedback_deadline();
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame);

}
#endif
