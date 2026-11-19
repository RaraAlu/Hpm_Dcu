#include "motor_adapter.hpp"
#include "detail/motor_adapter_state.hpp"
#include "motor_ethercat.h"
#include "motor_axis_codec.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_services.hpp"
#include "can_transport.h"
#include <cstring>

namespace application = dcu::app::gateway_application;
namespace adapter = dcu::ethercat::motor_adapter;
namespace axis_codec = dcu::ethercat::motor_axis_codec;
namespace pdo = dcu::ethercat::motor_pdo;
namespace layout = dcu::ethercat::pdo_layout;
using namespace dcu::ethercat::motor_dictionary;

namespace {
// Task-owned scratch keeps full configuration/publication values off the 8 KiB task stack.
application::Configuration configuration;
const dcu_motor_device_profile_t *slot_profiles[ENCOS_AXIS_COUNT];
const dcu_motor_device_profile_t *profile_devices;
uint8_t device_slots[DCU_MOTOR_DEVICE_CAPACITY];
unsigned device_count;
uint32_t configured_slots;
uint8_t configured_motors[ENCOS_CHANNEL_COUNT];
adapter::detail::InputState input;
bool bind_profile(const dcu_production_profile_t &profile)
{
    if (profile.device_count > ENCOS_AXIS_COUNT || (profile.device_count && !profile.devices)) return false;
    configuration.board = {profile.max_command_lease_ms, profile.feedback_timeout_ms};
    configuration.device_count = profile.device_count;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        const auto *active = dcu_can_channel_config(c);
        configuration.channels[c] = {};
        configuration.channels[c].enabled = active != nullptr;
        configuration.channels[c].frame_format = active && active->fd;
    }
    uint8_t counts[ENCOS_CHANNEL_COUNT]{};
    uint8_t slots[DCU_MOTOR_DEVICE_CAPACITY]{};
    for (size_t i = 0; i < profile.device_count; ++i) {
        const auto &device = profile.devices[i];
        const unsigned bus = device.bus;
        if (bus >= ENCOS_CHANNEL_COUNT || !configuration.channels[bus].enabled ||
            counts[bus] >= ENCOS_MOTORS_PER_CHANNEL) return false;
        slots[i] = bus * ENCOS_MOTORS_PER_CHANNEL + counts[bus]++;
        configuration.channels[bus].role = 1;
        auto &motor = configuration.devices[i];
        motor.route = {device.can_id, device.position_min_urad, device.position_max_urad,
                       device.velocity_limit_urad_s, device.torque_limit_mNm, device.current_limit_dA};
        motor.model_profile_id = device.model_profile_id;
        motor.power_query_period_ms = device.power_query_period_ms;
        motor.encos = device.encos;
        motor.bus = bus;
        motor.status_poll_period_ms = device.status_poll_period_ms;
        motor.status_reply_timeout_ms = device.status_reply_timeout_ms;
    }
    if (!application::initialize(configuration)) return false;
    profile_devices = profile.devices;
    device_count = profile.device_count;
    for (size_t i = 0; i < device_count; ++i) {
        const unsigned slot = slots[i];
        device_slots[i] = slot;
        slot_profiles[slot] = &profile_devices[i];
        configured_slots |= 1U << slot;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
        configured_motors[c] = (1U << counts[c]) - 1U;
    return true;
}

void publish_profile()
{
    auto &board = *object<mc_board_config_t>(0x2001);
    board.AxisConfiguredMask = configured_slots;
    board.MaxCommandLeaseMs = configuration.board.max_command_lease_ms;
    board.FeedbackTimeoutMs = configuration.board.feedback_timeout_ms;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &channel = *object<mc_channel_config_t>(0x2010 + c);
        const auto *active = dcu_can_channel_config(c);
        channel.Role = configuration.channels[c].role;
        channel.MotorConfiguredMask = configured_motors[c];
        channel.FrameFormat = active && active->fd;
        channel.NominalBitrate = active ? active->nominal_bitrate_bps : 0;
        channel.DataBitrate = active ? active->data_bitrate_bps : 0;
    }
}

void publish_owner()
{
    auto &owner = *object<mc_owner_t>(0x2003);
    owner.Owner = application::owner();
    owner.RequestedOwner = owner.Owner;
    owner.Result = dcu::motor::RESULT_OK;
}
}

bool motor_control_init(void)
{
    if (!bind_profile(*dcu_production_profile()) || !pdo::bind_publication()) return false;
    dcu::ethercat::motor_services::initialize();
    publish_profile();
    if (!application::select_mode(application::OperatingMode::MotorWithRaw)) return false;
    object<mc_board_info_t>(0x2000)->Capabilities = 7;
    return true;
}

bool motor_control_boot_startup(uint64_t id, uint64_t (*issue_session)(void))
{
    return application::boot_startup(dcu_production_profile()->readiness, id, issue_session);
}

void motor_control_output(const void *data)
{
    input.mode = application::mode();
    const size_t bytes = layout::profiles[static_cast<unsigned>(input.mode)].output;
    std::memcpy(input.wire, data, bytes);
    pdo::decode_cycle(input.wire, input.mode, input.command);
    input.pending = true;
}

bool motor_control_process(void)
{
    // A PREOP remapping must not reinterpret a pending image from the previous mode.
    if (input.pending && input.mode != application::mode()) input.pending = false;
    const auto previous_owner = application::owner();
    const auto result = application::process(input.pending ? &input.command : nullptr);
    if (previous_owner != application::owner()) publish_owner();
    if (result.consumed) input.pending = false;
    if (!result.accepted) return !input.pending;
    *object<mc_board_rx_t>(0x7000) = pdo::board(input.wire);
    object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq = input.command.board.sequence;
    if (input.mode == application::OperatingMode::Raw) {
        if (result.raw_mirrored)
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
                *object<canfd_bus_pdo_t>(0x7400 + 16 * c) = pdo::raw_bus(input.wire, c);
        return true;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
        *object<mc_channel_rx_t>(0x7100 + c) = pdo::channel(input.wire, c);
    const uint32_t mirror = adapter::slots_for_devices(result.mirrored_devices);
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
        if (mirror & (1u << a))
            *object<mc_axis_rx_t>(0x7200 + a) = pdo::axis(input.wire, a);
    if (input.mode == application::OperatingMode::MotorWithRaw)
        *object<mc_raw_t>(0x7300) = pdo::raw(input.wire);
    return true;
}

void motor_control_operational(bool operational)
{
    input.pending = false;
    const auto previous_owner = application::owner();
    application::operational(operational);
    if (!operational || previous_owner != application::owner()) publish_owner();
}

void motor_control_cycle_fault(void)
{
    input.pending = false;
    application::cycle_fault();
    publish_owner();
}

namespace dcu::ethercat::motor_adapter {

const dcu_motor_device_profile_t *configured_device(unsigned slot)
{
    return slot < ENCOS_AXIS_COUNT ? slot_profiles[slot] : nullptr;
}

unsigned device_for_slot(unsigned slot)
{
    const auto *profile = configured_device(slot);
    return profile ? static_cast<unsigned>(profile - profile_devices) : DCU_MOTOR_DEVICE_CAPACITY;
}

unsigned slot_for_device(unsigned device)
{
    return device < device_count ? device_slots[device] : ENCOS_AXIS_COUNT;
}

uint32_t slots_for_devices(uint32_t mask)
{
    uint32_t result = 0;
    for (unsigned i = 0; i < device_count; ++i)
        if (mask & (1U << i)) result |= 1U << device_slots[i];
    return result;
}

void refresh_axis_feedback(unsigned axis)
{
    motor::MotorFeedback feedback{};
    motor::MotorDiagnostics diagnostics{};
    if (!application::read_device(device_for_slot(axis), feedback, diagnostics)) return;
    *axis_feedback(axis) = axis_codec::feedback(feedback);
    axis_codec::diagnostics(diagnostics, *axis_diagnostics(axis));
}

void commit_sdo_command(unsigned axis, uint32_t now)
{
    if (axis >= ENCOS_AXIS_COUNT) return;
    auto &wire = *object<mc_sdo_command_t>(0x2200 + 16 * axis);
    const application::SdoCommand command{axis_codec::command(wire), wire.CommitSeq, wire.LeaseMs};
    const auto result = application::commit_sdo_command(device_for_slot(axis), command, now);
    wire.AcceptedSeq = result.accepted_sequence;
    if (result.changed) wire.CommandResult = result.result;
    if (result.mirror_command) *object<mc_axis_rx_t>(0x7200 + axis) = axis_codec::command(command.command);
}
}
