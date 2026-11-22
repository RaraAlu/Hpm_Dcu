#include "motor_ethercat.h"
#include "detail/motor_pdo_state.hpp"
#include "gateway_application.hpp"
#include "motor_dictionary.hpp"
#include "motor_adapter.hpp"
#include "motor_pdo_codec.hpp"
#include "motor_axis_codec.hpp"
#include "pdo_pipeline.h"
#include "can_transport.h"
extern "C" {
#include "digital_io.h"
#include "ecatslv.h"
}
#include <cstring>

namespace pdo_layout = dcu::ethercat::pdo_layout;
namespace motor_pdo = dcu::ethercat::motor_pdo;
using namespace dcu::ethercat::motor_dictionary;
static_assert(pdo_layout::max_output_bytes <= PDO_PIPELINE_RX_MAX, "PDO staging capacity is too small");

namespace {
dcu::ethercat::motor_pdo::detail::PublicationState publication;

constexpr mc_axis_tx_t empty_feedback()
{
    mc_axis_tx_t feedback{};
    feedback.StatusWord = 128U | 2048U;
    feedback.FeedbackAgeMs = feedback.PowerAgeMs = UINT16_MAX;
    return feedback;
}

void latch_layout(unsigned profile)
{
    if (profile == publication.layout) return;
    const bool motor = profile != 2;
    if (publication.layout == pdo_layout::profile_count || motor != publication.motor_layout) {
        // Raw and motor mappings share the SSC image; restore the motor template on reentry.
        memset(publication.image + sizeof(mc_board_tx_t), 0,
               pdo_layout::profiles[profile].input - sizeof(mc_board_tx_t));
        if (motor) {
            constexpr auto empty = empty_feedback();
            for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
                motor_pdo::write_axis(publication.image, a, empty);
        }
    } else if (profile == 1) {
        memset(publication.raw_tx, 0, sizeof(mc_raw_t));
    }
    publication.layout = profile;
    publication.motor_layout = motor;
    publication.diagnostic_layout = false;
}

bool publish_board_and_channels()
{
    const auto &snapshot = publication.snapshot;
    const auto online = dcu::ethercat::motor_adapter::slots_for_devices(snapshot.online_devices);
    const auto faults = dcu::ethercat::motor_adapter::slots_for_devices(snapshot.fault_devices);
    auto &board = *publication.board;
    board = {snapshot.sequence, snapshot.accepted_cycle_sequence, snapshot.board_status,
        snapshot.online_channels, snapshot.fault_channels, online, faults,
        snapshot.rx_drops, snapshot.tx_drops, 0};
    motor_pdo::write_board(publication.image, board);
    bool motor_only = true;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        const auto &channel = snapshot.channels[c];
        const auto &slot = publication.channels[c];
        auto &mirror = *slot.feedback;
        mirror = {channel.status,
            static_cast<uint8_t>((online >> (c * ENCOS_MOTORS_PER_CHANNEL)) & 0x7f),
            static_cast<uint8_t>((faults >> (c * ENCOS_MOTORS_PER_CHANNEL)) & 0x7f),
            static_cast<uint16_t>(channel.rx_drops > UINT16_MAX ? UINT16_MAX : channel.rx_drops),
            static_cast<uint16_t>(channel.tx_drops > UINT16_MAX ? UINT16_MAX : channel.tx_drops)};
        if (publication.motor_layout) motor_pdo::store(slot.tx, 0, mirror);
        motor_only &= channel.role < 2;
    }
    return motor_only;
}

void publish_axes(uint8_t *diagnostic)
{
    for (unsigned i = 0; i < publication.slot_count; ++i) {
        const auto &slot = publication.slots[i];
        const auto &entry = publication.snapshot.devices[slot.device];
        auto &mirror = *slot.feedback;
        mirror = dcu::ethercat::motor_axis_codec::feedback(entry.feedback);
        dcu::ethercat::motor_axis_codec::diagnostics(entry.diagnostics, *slot.diagnostics);
        if (publication.motor_layout) motor_pdo::store(slot.tx, 0, mirror);
        if (diagnostic) {
            const auto count = static_cast<uint8_t>(diagnostic[4] ? entry.cyclic_feedback_count :
                                                                   entry.diagnostics.tx_complete_count);
            diagnostic[5 + slot.slot] = publication.raw_tx[5 + slot.slot] = count;
        }
    }
}
}

namespace dcu::ethercat::motor_pdo {
static_assert(app::config::raw_can::batch_capacity >= CANFD_GATEWAY_SLOTS_PER_BUS);
static_assert(static_cast<unsigned>(app::gateway_application::OperatingMode::Motor) == 0 &&
              static_cast<unsigned>(app::gateway_application::OperatingMode::MotorWithRaw) == 1 &&
              static_cast<unsigned>(app::gateway_application::OperatingMode::Raw) == 2 &&
              pdo_layout::profile_count == 3);

bool bind_publication()
{
    uint16_t capacity = 0;
    publication.image = static_cast<uint8_t *>(dcu_ssc_txpdo_buffer(&capacity));
    if (!publication.image || capacity < pdo_layout::max_input_bytes) return false;
    publication.board = object<mc_board_tx_t>(0x6000);
    publication.imu = object<mc_imu_t>(0x6300);
    publication.raw = object<mc_raw_t>(0x6400);
    if (!publication.board || !publication.imu || !publication.raw) return false;
    publication.imu_tx = publication.image + pdo_layout::input_imu_offset;
    publication.raw_tx = publication.image + pdo_layout::motor_input_bytes;
    publication.slot_count = 0;
    publication.configured_slots = 0;
    publication.layout = pdo_layout::profile_count;
    publication.motor_layout = publication.diagnostic_layout = false;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &slot = publication.channels[c];
        slot = {object<mc_channel_tx_t>(0x6100 + c), object<canfd_bus_pdo_t>(0x6500 + c * 16),
                publication.image + pdo_layout::input_channels_offset + c * sizeof(mc_channel_tx_t),
                publication.image + sizeof(mc_board_tx_t) + c * sizeof(canfd_bus_pdo_t)};
        if (!slot.feedback || !slot.raw) return false;
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a) {
        auto *feedback = axis_feedback(a);
        auto *diagnostics = axis_diagnostics(a);
        if (!feedback || !diagnostics) return false;
        diagnostics->PositionQueryAgeUs = diagnostics->VelocityQueryAgeUs = UINT32_MAX;
        const auto device = motor_adapter::device_for_slot(a);
        if (device >= DCU_MOTOR_DEVICE_CAPACITY) {
            *feedback = empty_feedback();
            continue;
        }
        feedback->FeedbackAgeMs = feedback->PowerAgeMs = UINT16_MAX;
        publication.configured_slots |= 1U << a;
        publication.slots[publication.slot_count++] = {
            feedback, diagnostics, publication.image + pdo_layout::input_axes_offset + a * sizeof(mc_axis_tx_t),
            static_cast<uint16_t>(pdo_layout::output_axes_offset + a * sizeof(mc_axis_rx_t)),
            static_cast<uint8_t>(a), static_cast<uint8_t>(device)};
    }
    return true;
}

void decode_cycle(const void *image, app::gateway_application::OperatingMode mode,
                  app::gateway_application::CycleCommand &command)
{
    const auto b = board(image);
    command.board = {b.CycleSeq, 0, b.BoardControl, b.CommandLeaseMs, b.ChannelEnableMask,
        b.InterfaceVersion == MC_INTERFACE_VERSION && !b.Reserved0 && !b.Reserved1 &&
        !b.Reserved2 && !b.Reserved3 && !b.Reserved4 && !(b.AxisEnableMask >> ENCOS_AXIS_COUNT)};
    command.active_devices = 0;
    command.board.unmapped_device_requested = b.AxisEnableMask != 0;
    if (mode == app::gateway_application::OperatingMode::Raw) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            decode_raw_bus(raw_bus(image, c), c, command.raw[c]);
    } else {
        uint32_t requested_slots = 0;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            const auto ch = channel(image, c);
            command.channels[c] = {ch.ChannelControl, ch.ExpectedRole, !(ch.ActiveMask & 0x80), ch.Reserved};
            requested_slots |= static_cast<uint32_t>(ch.ActiveMask & 0x7fu) << (c * ENCOS_MOTORS_PER_CHANNEL);
        }
        for (unsigned i = 0; i < publication.slot_count; ++i) {
            const auto &slot = publication.slots[i];
            const uint32_t bit = 1U << slot.slot;
            if (b.AxisEnableMask & bit) command.board.enabled_devices |= 1U << slot.device;
            if (requested_slots & bit) command.active_devices |= 1U << slot.device;
            command.devices[slot.device] = motor_axis_codec::command(load<mc_axis_rx_t>(image, slot.rx_offset));
        }
        command.board.unmapped_device_requested = (b.AxisEnableMask & ~publication.configured_slots) != 0;
        if (mode == app::gateway_application::OperatingMode::MotorWithRaw)
            decode_raw_sideband(raw(image), command.raw[0]);
    }
}

void decode_raw_sideband(const mc_raw_t &raw, app::RawCanBatch &batch)
{
    batch = {};
    batch.sequence = raw.BatchSeq;
    if ((raw.ValidMask & ~3u) || raw.SlotCount > 2 || raw.Flags || raw.DropCount ||
        raw.Reserved || raw.Reserved1 || raw.Reserved2)
        return;
    batch.valid = true;
    for (unsigned i = 0; i < 2; ++i) {
        if (!(raw.ValidMask & (1u << i))) continue;
        const auto wire = raw_slot(raw, i);
        auto &slot = batch.slots[batch.count++];
        slot.bus = wire.channel;
        slot.frame.id = wire.id;
        slot.frame.flags = wire.flags;
        slot.frame.dlc = wire.dlc;
        std::memcpy(slot.frame.data, wire.data, sizeof(wire.data));
    }
}

void decode_raw_bus(const canfd_bus_pdo_t &raw, unsigned bus, app::RawCanBatch &batch)
{
    batch = {};
    batch.kind = app::RawCanBatch::Kind::FullBus;
    batch.bus = bus < ENCOS_CHANNEL_COUNT ? static_cast<uint8_t>(bus) : ENCOS_CHANNEL_COUNT;
    batch.sequence = raw.cycle_seq;
    if (bus >= ENCOS_CHANNEL_COUNT || (raw.valid_mask & 0x80u))
        return;
    batch.valid = true;
    for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS; ++i) {
        if (!(raw.valid_mask & (1u << i)) || !(raw.slot[i].flags & CANFD_FLAG_VALID)) continue;
        const auto &wire = raw.slot[i];
        auto &slot = batch.slots[batch.count++];
        slot.bus = bus;
        slot.frame.id = wire.can_id;
        slot.frame.flags = static_cast<uint8_t>(wire.flags & ~(CANFD_FLAG_VALID | CANFD_FLAG_ON_CHANGE));
        slot.frame.dlc = wire.dlc;
        std::memcpy(slot.frame.data, wire.data, sizeof(wire.data));
    }
}

unsigned mapping_indices(unsigned profile, bool tx, uint16_t *indices)
{
    unsigned n = 0;
    indices[n++] = tx ? 0x1A00 : 0x1600;
    if (profile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
            indices[n++] = (tx ? 0x1A50 : 0x1650) + c;
        return n;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        indices[n++] = (tx ? 0x1A01 : 0x1601) + c;
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        indices[n++] = (tx ? 0x1A20 : 0x1620) + a;
    if (tx) {
        indices[n++] = 0x1A40;
    }
    if (profile == 1) {
        indices[n++] = tx ? 0x1A41 : 0x1640;
    }
    return n;
}
}

using motor_pdo::mapping_indices;
uint16_t motor_control_mapping(uint16_t *in, uint16_t *out)
{
    for (unsigned p = 0; p < pdo_layout::profile_count; p++) {
        uint16_t rx[pdo_layout::assignment_capacity], tx[pdo_layout::assignment_capacity];
        unsigned nr = mapping_indices(p, false, rx), nt = mapping_indices(p, true, tx);
        if (gRxAssign.u16SubIndex0 == nr && gTxAssign.u16SubIndex0 == nt && !memcmp(rx, gRxAssign.aEntries, nr * 2) && !memcmp(tx, gTxAssign.aEntries, nt * 2)) {
            if (!dcu::app::gateway_application::select_mode(static_cast<dcu::app::gateway_application::OperatingMode>(p)))
                return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            latch_layout(p);
            object<mc_board_config_t>(0x2001)->ActivePdoProfile = p;
            *in = pdo_layout::profiles[p].input;
            *out = pdo_layout::profiles[p].output;
            return 0;
        }
    }
    *in = *out = 0;
    return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
}
bool motor_control_neutral_feedback_pending(void)
{
    return dcu::app::gateway_application::neutral_feedback_pending();
}

void motor_control_set_neutral_window(uint32_t period_ns, uint32_t cutoff_us)
{
    dcu::app::gateway_application::set_neutral_window(period_ns, cutoff_us);
}

void motor_control_neutral_feedback_deadline(void)
{
    dcu::app::gateway_application::neutral_feedback_deadline();
}

void motor_control_prepare_input(void)
{
    auto &snapshot = publication.snapshot;
    dcu::app::gateway_application::prepare_publication(snapshot);
    const unsigned profile = static_cast<unsigned>(snapshot.mode);
    latch_layout(profile);
    const bool motor_only = publish_board_and_channels();
    auto *raw = publication.raw;
    uint8_t *diagnostic = nullptr;
    if (profile == 1) {
        if (motor_only) {
            diagnostic = reinterpret_cast<uint8_t *>(raw);
            if (!publication.diagnostic_layout) {
                memset(raw, 0, sizeof(*raw));
                memcpy(diagnostic, "T1R2", 4);
                motor_pdo::store(publication.raw_tx, 0, *raw);
                publication.diagnostic_layout = true;
            }
            diagnostic[4] = publication.raw_tx[4] = publication.diagnostic_page++ & 1u;
        } else {
            memset(raw, 0, sizeof(*raw));
            publication.diagnostic_layout = false;
        }
    }
    publish_axes(diagnostic);
    if (profile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
            const auto &slot = publication.channels[c];
            auto *raw = slot.raw;
            memset(raw, 0, sizeof(*raw));
            raw->cycle_seq = snapshot.cycle_sequence;
            raw->bus_state = dcu_can_bus_state(c);
            raw->counter0 = dcu_can_rx_drops(c) + snapshot.channels[c].raw_drops;
            dcu_can_frame_t f;
            for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS && dcu::app::gateway_application::take_raw_frame(c, f); i++) {
                raw->valid_mask |= 1u << i;
                raw->mode++;
                raw->slot[i].can_id = f.id;
                raw->slot[i].flags = f.flags | 1;
                raw->slot[i].dlc = f.dlc;
                raw->slot[i].seq = ++publication.raw_batch;
                memcpy(raw->slot[i].data, f.data, sizeof(raw->slot[i].data));
            }
            motor_pdo::store(slot.raw_tx, 0, *raw);
        }
    } else {
        motor_pdo::store(publication.imu_tx, 0, *publication.imu);
        if (profile == 1 && !motor_only) {
            raw->BatchSeq = ++publication.raw_batch;
            raw->DropCount = publication.board->RxDropCount;
            for (unsigned i = 0; i < 2; i++)
                for (unsigned n = 0; n < ENCOS_CHANNEL_COUNT; n++) {
                    unsigned c = publication.round_robin;
                    publication.round_robin = (publication.round_robin + 1) % ENCOS_CHANNEL_COUNT;
                    dcu_can_frame_t f;
                    if (!dcu::app::gateway_application::take_raw_frame(c, f))
                        continue;
                    motor_pdo::RawSlot slot{};
                    slot.id = f.id;
                    slot.channel = c;
                    slot.flags = f.flags;
                    slot.dlc = f.dlc;
                    memcpy(slot.data, f.data, sizeof(slot.data));
                    motor_pdo::write_raw_slot(*raw, i, slot);
                    raw->ValidMask |= 1u << i;
                    raw->SlotCount++;
                    break;
                }
            motor_pdo::store(publication.raw_tx, 0, *raw);
        }
    }
}
