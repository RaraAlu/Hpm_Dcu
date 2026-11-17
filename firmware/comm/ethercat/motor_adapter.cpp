#include "motor_adapter.hpp"
#include "detail/motor_adapter_state.hpp"
#include "motor_ethercat.h"
#include "motor_axis_codec.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_services.hpp"
#include <cstring>

namespace application = dcu::app::motor_application;
namespace adapter = dcu::ethercat::motor_adapter;
namespace axis_codec = dcu::ethercat::motor_axis_codec;
namespace pdo = dcu::ethercat::motor_pdo;
namespace layout = dcu::ethercat::pdo_layout;
using namespace dcu::ethercat::motor_dictionary;

namespace {
// Task-owned scratch keeps full configuration/publication values off the 8 KiB task stack.
application::Configuration configuration;
adapter::detail::InputState input;

void read_configuration()
{
    const auto &board = *object<mc_board_config_t>(0x2001);
    configuration.board = {board.AxisConfiguredMask, board.MaxCommandLeaseMs, board.FeedbackTimeoutMs};
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        const auto &channel = *object<mc_channel_config_t>(0x2010 + c);
        configuration.channels[c] = {channel.Role, channel.FrameFormat, channel.NominalBitrate,
            channel.DataBitrate, channel.ScheduleUs, channel.RawRateLimit, channel.MotorConfiguredMask};
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a) {
        const auto base = axis_object_base(a);
        configuration.axes[a] = axis_codec::configuration(*object<mc_routing_t>(base),
            *object<mc_basic_t>(base + 1), *object<mc_range_t>(base + 2));
    }
}

void publish_owner()
{
    auto &owner = *object<mc_owner_t>(0x2003);
    owner.Owner = application::owner();
    owner.RequestedOwner = owner.Owner;
    owner.Result = application::RESULT_OK;
}
}

void motor_control_init(void)
{
    dcu::ethercat::motor_services::initialize();
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a) {
        object<mc_routing_t>(axis_object_base(a))->CanId = a % ENCOS_MOTORS_PER_CHANNEL + 1;
        axis_feedback(a)->FeedbackAgeMs = axis_feedback(a)->PowerAgeMs = UINT16_MAX;
        axis_diagnostics(a)->PositionQueryAgeUs = axis_diagnostics(a)->VelocityQueryAgeUs = UINT32_MAX;
    }
    read_configuration();
    application::initialize(configuration);
    const auto mode = static_cast<application::OperatingMode>(object<mc_board_config_t>(0x2001)->ActivePdoProfile);
    if (!application::select_mode(mode)) application::cycle_fault();
    object<mc_board_info_t>(0x2000)->Capabilities = 7;
}

void motor_control_output(const void *data)
{
    input.mode = application::mode();
    std::memcpy(input.wire, data, layout::profiles[static_cast<unsigned>(input.mode)].output);
    pdo::decode_cycle(input.wire, input.mode, input.command);
    input.pending = true;
}

void motor_control_process(void)
{
    // A PREOP remapping must not reinterpret a pending image from the previous mode.
    if (input.pending && input.mode != application::mode()) input.pending = false;
    const auto result = application::process(input.pending ? &input.command : nullptr);
    if (result.consumed) input.pending = false;
    if (!result.accepted) return;
    *object<mc_board_rx_t>(0x7000) = pdo::board(input.wire);
    object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq = input.command.board.sequence;
    if (input.mode == application::OperatingMode::Raw) {
        if (result.raw_mirrored)
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
                *object<canfd_bus_pdo_t>(0x7400 + 16 * c) = pdo::raw_bus(input.wire, c);
        return;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
        *object<mc_channel_rx_t>(0x7100 + c) = pdo::channel(input.wire, c);
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
        if (result.mirrored_axes & (1u << a))
            *object<mc_axis_rx_t>(0x7200 + a) = pdo::axis(input.wire, a);
    if (input.mode == application::OperatingMode::MotorWithRaw)
        *object<mc_raw_t>(0x7300) = pdo::raw(input.wire);
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
bool apply_configuration()
{
    read_configuration();
    return application::apply_configuration(configuration);
}

void refresh_axis_feedback(unsigned axis)
{
    app::AxisFeedback feedback{};
    app::AxisDiagnostics diagnostics{};
    if (!application::read_axis(axis, feedback, diagnostics)) return;
    *axis_feedback(axis) = axis_codec::feedback(feedback);
    axis_codec::diagnostics(diagnostics, *axis_diagnostics(axis));
}

void commit_sdo_command(unsigned axis, uint32_t now)
{
    if (axis >= ENCOS_AXIS_COUNT) return;
    auto &wire = *object<mc_sdo_command_t>(0x2200 + 16 * axis);
    const application::SdoCommand command{axis_codec::command(wire), wire.CommitSeq, wire.LeaseMs};
    const auto result = application::commit_sdo_command(axis, command, now);
    wire.AcceptedSeq = result.accepted_sequence;
    if (result.changed) wire.CommandResult = result.result;
    if (result.mirror_command) *object<mc_axis_rx_t>(0x7200 + axis) = axis_codec::command(command.command);
}
}
