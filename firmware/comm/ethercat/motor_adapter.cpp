#include "motor_adapter.hpp"
#include "cycle_config.h"
#include "detail/motor_adapter_state.hpp"
#include "motor_ethercat.h"
#include "motor_axis_codec.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_services.hpp"
#include "can_transport.h"
#include <cstring>
#include <new>

namespace application = dcu::app::gateway_application;
namespace adapter = dcu::ethercat::motor_adapter;
namespace axis_codec = dcu::ethercat::motor_axis_codec;
namespace pdo = dcu::ethercat::motor_pdo;
namespace layout = dcu::ethercat::pdo_layout;
using namespace dcu::ethercat::motor_dictionary;

namespace {
const dcu_motor_device_profile_t *slot_profiles[ENCOS_AXIS_COUNT];
const dcu_production_profile_t *bound_profile;
uint8_t device_slots[DCU_MOTOR_DEVICE_CAPACITY];
mc_axis_rx_t *command_mirrors[DCU_MOTOR_DEVICE_CAPACITY];
unsigned device_count;
uint32_t configured_slots;
layout::Layout catalogue_wire, requested_wire;
uint32_t generation;
adapter::detail::InputState input;
bool bind_slots(const dcu_production_profile_t &profile)
{
    if (bound_profile || profile.device_count > DCU_MOTOR_DEVICE_CAPACITY || profile.device_count > ENCOS_AXIS_COUNT ||
        (profile.device_count && !profile.devices)) return false;
    uint8_t counts[ENCOS_CHANNEL_COUNT]{};
    uint8_t slots[DCU_MOTOR_DEVICE_CAPACITY]{};
    for (size_t i = 0; i < profile.device_count; ++i) {
        const unsigned bus = profile.devices[i].bus;
        if (bus >= ENCOS_CHANNEL_COUNT || counts[bus] >= ENCOS_MOTORS_PER_CHANNEL) return false;
        slots[i] = bus * ENCOS_MOTORS_PER_CHANNEL + counts[bus]++;
        command_mirrors[i] = object<mc_axis_rx_t>(0x7200 + slots[i]);
        if (!command_mirrors[i]) return false;
    }
    bound_profile = &profile;
    device_count = profile.device_count;
    for (size_t i = 0; i < device_count; ++i) {
        const unsigned slot = slots[i];
        device_slots[i] = slot;
        slot_profiles[slot] = &profile.devices[i];
        configured_slots |= 1U << slot;
    }
    return true;
}

void publish_profile()
{
    auto &board = *object<mc_board_config_t>(0x2001);
    const auto policy = application::profile_policy();
    board.AxisConfiguredMask = configured_slots;
    board.MaxCommandLeaseMs = policy.max_command_lease_ms;
    board.FeedbackTimeoutMs = policy.feedback_timeout_ms;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &channel = *object<mc_channel_config_t>(0x2010 + c);
        const auto config = application::channel_configuration(c);
        const auto *active = config.enabled ? dcu_can_channel_config(c) : nullptr;
        channel.Role = config.role;
        channel.MotorConfiguredMask = (configured_slots >> (ENCOS_MOTORS_PER_CHANNEL * c)) & 0x7f;
        channel.FrameFormat = config.frame_format;
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

bool adapter::initialize(const dcu_production_profile_t &profile)
{
    if (!bind_slots(profile) || !pdo::bind_publication(profile)) return false;
    catalogue_wire = layout::active();
    dcu::ethercat::motor_services::initialize();
    publish_profile();
    object<mc_board_info_t>(0x2000)->Capabilities = 7;
    return true;
}

bool motor_control_event_pdo(void)
{
    static_assert(sizeof(mc_event_header_t) == dcu_can_event::header_bytes && sizeof(mc_event_data_t) == dcu_can_event::data_bytes);
    return application::mode() == application::OperatingMode::SingleFd;
}

uint32_t motor_control_event_lease_us(void)
{ return uint32_t{application::profile_policy().max_command_lease_ms} * 1000U; }
uint16_t motor_control_event_bytes(void) { return layout::event_layout().bytes(3, false); }

bool motor_control_preview_supported(void)
{
    const auto mode = application::mode();
    return mode == application::OperatingMode::Motor || mode == application::OperatingMode::MotorWithRaw;
}

void motor_control_preview(const void *data)
{
    const auto board = pdo::board(data);
    application::reset_preview(board.CycleSeq);
    if ((board.BoardControl & 3U) != 1U) return;
    uint32_t slots = board.AxisEnableMask & layout::active().command_axes;
    while (slots) {
        const unsigned slot = __builtin_ctz(slots);
        slots &= slots - 1;
        application::preview_control(adapter::device_for_slot(slot), axis_codec::command(pdo::axis(data, slot)));
    }
}

void motor_control_output(const void *data)
{
    input.mode = application::mode();
    const size_t bytes = motor_control_event_pdo() ? motor_control_event_bytes() :
        layout::active().bytes(static_cast<unsigned>(input.mode), false);
    std::memcpy(input.wire, data, bytes);
    pdo::decode_cycle(input.wire, input.mode, input.command);
    input.pending = true;
}

DCU_FAST_CODE bool motor_control_process(void)
{
    // A PREOP remapping must not reinterpret a pending image from the previous mode.
    if (input.pending && input.mode != application::mode()) input.pending = false;
    const auto previous_owner = application::owner();
    const auto previous_raw = application::raw_state();
    const auto result = application::process(input.pending ? &input.command : nullptr);
    if (previous_raw == application::RawState::Active && application::raw_state() != previous_raw)
        motor_control_clear_raw_input();
    if (previous_owner != application::owner()) publish_owner();
    if (result.consumed) input.pending = false;
    if (!result.accepted) return !input.pending;
    if (input.mode == application::OperatingMode::SingleFd) {
        if (result.raw_mirrored) {
            *object<mc_event_header_t>(0x7305) = pdo::load<mc_event_header_t>(input.wire, 0);
            const auto mask = layout::event_layout().buses;
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
                if (mask & (1U << c)) *object<mc_event_data_t>(0x7301 + c) =
                    pdo::load<mc_event_data_t>(input.wire, dcu_can_event::offset(mask, c));
        }
        return true;
    }
    *object<mc_board_rx_t>(0x7000) = pdo::board(input.wire);
    object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq = input.command.board.sequence;
    if (input.mode == application::OperatingMode::Raw) {
        if (result.raw_mirrored)
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
                if (layout::active().buses & (1U << c))
                    *object<canfd_bus_pdo_t>(0x7400 + 16 * c) = pdo::raw_bus(input.wire, c);
        return true;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
        if (layout::active().buses & (1U << c))
            *object<mc_channel_rx_t>(0x7100 + c) = pdo::channel(input.wire, c);
    for (auto mask = result.mirrored_devices; mask; mask &= mask - 1U) {
        const unsigned device = __builtin_ctz(mask);
        const unsigned binding = device_slots[device];
        if (layout::active().command_axes & (1U << binding))
            *command_mirrors[device] = pdo::axis(input.wire, binding);
    }
    if (input.mode == application::OperatingMode::MotorWithRaw && result.raw_mirrored)
        *object<pdo::layout::MappedSideband>(0x7300) = pdo::raw(input.wire);
    return true;
}

void motor_control_set_cyclic_feedback_window(uint32_t period_us, uint32_t deadline_us)
{ application::set_cyclic_feedback_window(period_us, deadline_us); }

uint32_t motor_control_cyclic_feedback_required(void) { return application::cyclic_feedback_required(); }
uint32_t motor_control_cyclic_feedback_missing(void) { return application::cyclic_feedback_missing(); }
void motor_control_fault_cyclic_feedback(uint32_t missing) { application::fault_cyclic_feedback(missing); }

void motor_control_operational(bool operational)
{
    // 在原缓冲区重建，避免 Debug 固件在任务栈上生成整幅 PDO 临时副本。
    if (!operational) new (&input) adapter::detail::InputState{};
    else input.pending = false;
    const auto previous_owner = application::owner();
    application::operational(operational);
    if (!operational) motor_control_clear_raw_input();
    if (!operational || previous_owner != application::owner()) publish_owner();
}

void motor_control_cycle_fault(void)
{
    new (&input) adapter::detail::InputState{};
    application::cycle_fault();
    motor_control_clear_raw_input();
    publish_owner();
}

void motor_control_raw_link_lost(void)
{
    application::layout_fault();
    motor_control_clear_raw_input();
}

void motor_control_mailbox_stopped(void)
{
    application::layout_fault(dcu_profile_pdo::sideband_bus >= 0 ? PDO_FIXED_MAILBOX_STOP : PDO_ABORT);
    motor_control_clear_raw_input();
}

namespace dcu::ethercat::motor_adapter {

uint32_t topology_generation() { return generation; }

const layout::Layout &requested_layout()
{ return application::topology_pending() ? requested_wire : layout::active(); }

bool stage_selection(uint32_t buses, uint32_t axes, uint32_t base)
{
    if (!bound_profile || !buses || base != generation || base == UINT32_MAX ||
        (buses & ~catalogue_wire.buses) || (axes & ~catalogue_wire.feedback_axes)) return false;
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
        if ((axes & (1U << a)) && !(buses & (1U << (a / ENCOS_MOTORS_PER_CHANNEL)))) return false;
    if (!application::request_topology()) return false;
    requested_wire = {buses, axes & catalogue_wire.command_axes, axes};
    clear_layout_commands();
    return true;
}

bool commit_selection()
{
    uint32_t devices = 0;
    for (unsigned i = 0; i < device_count; ++i)
        if (requested_wire.feedback_axes & (1U << device_slots[i])) devices |= 1U << i;
    if (!application::commit_topology(*bound_profile, requested_wire.buses, devices)) return false;
    configured_slots = requested_wire.feedback_axes;
    clear_layout_commands();
    for (unsigned i = 0; i < device_count; ++i) {
        const unsigned a = device_slots[i];
        *object<mc_sdo_command_t>(0x2200 + 16 * a) = {};
        *object<mc_query_t>(axis_object_base(a) + 5) = {};
        *object<mc_identity_t>(axis_object_base(a) + 6) = {};
        *object<mc_transaction_t>(axis_object_base(a) + 7) = {};
    }
    pdo::select_publication(requested_wire);
    publish_profile();
    publish_owner();
    ++generation;
    return true;
}

void clear_layout_commands()
{
    new (&input) detail::InputState{};
    *object<mc_board_rx_t>(0x7000) = {};
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        *object<mc_channel_rx_t>(0x7100 + c) = {};
        *object<mc_event_data_t>(0x7301 + c) = {};
        *object<canfd_bus_pdo_t>(0x7400 + 16 * c) = {};
    }
    for (unsigned i = 0; i < device_count; ++i)
        *object<mc_axis_rx_t>(0x7200 + device_slots[i]) = {};
    *object<pdo::layout::MappedSideband>(0x7300) = {};
    *object<mc_event_header_t>(0x7305) = {};
}

const dcu_motor_device_profile_t *configured_device(unsigned slot)
{
    return slot < ENCOS_AXIS_COUNT && (configured_slots & (1U << slot)) ? slot_profiles[slot] : nullptr;
}

unsigned device_for_slot(unsigned slot)
{
    const auto *profile = configured_device(slot);
    return profile ? static_cast<unsigned>(profile - bound_profile->devices) : DCU_MOTOR_DEVICE_CAPACITY;
}

unsigned slot_for_device(unsigned device)
{
    return device < device_count && (configured_slots & (1U << device_slots[device])) ?
        device_slots[device] : static_cast<unsigned>(ENCOS_AXIS_COUNT);
}

uint32_t slots_for_devices(uint32_t mask)
{
    uint32_t result = 0;
    for (unsigned i = 0; i < device_count; ++i)
        if (mask & (1U << i)) result |= 1U << device_slots[i];
    return result & configured_slots;
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
    const auto result = application::commit_sdo_command(device_for_slot(axis), wire.CommitSeq, now);
    wire.AcceptedSeq = 0; // Production never grants SDO motion ownership.
    if (result.changed) wire.CommandResult = result.result;
}
}
