#include "motor_ethercat.h"
#include "detail/motor_pdo_state.hpp"
#include "gateway_application.hpp"
#include "motor_dictionary.hpp"
#include "motor_adapter.hpp"
#include "motor_pdo_codec.hpp"
#include <dcu_sideband.hpp>
#include "motor_axis_codec.hpp"
#include "pdo_pipeline.h"
#include "can_transport.h"
#include "cycle_config.h"
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

void clear_event_image()
{
    *publication.event_header = {};
    for (auto *slot : publication.single_fd) *slot = {};
    memset(publication.image, 0, publication.event.bytes(3, true));
}

void latch_layout(unsigned profile)
{
    if (profile == publication.layout) return;
    const bool motor = profile < 2;
    if (motor) {
        auto &rx = publication.motor_rx;
        rx.bind(publication.wire, profile, false);
        pdo_layout::MotorOffsets tx;
        tx.bind(publication.wire, profile, true);
        publication.raw_tx = tx.sideband == pdo_layout::absent ? nullptr : publication.image + tx.sideband;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            publication.channels[c].tx = tx.channels[c] == pdo_layout::absent ? nullptr : publication.image + tx.channels[c];
        for (unsigned i = 0; i < publication.slot_count; ++i) {
            auto &binding = publication.slots[i];
            const auto offset = tx.motors[binding.slot];
            binding.tx = offset == pdo_layout::absent ? nullptr : publication.image + offset;
            binding.rx_offset = rx.motors[binding.slot];
        }
    }
    if (publication.layout < pdo_layout::profile_count && (!motor || !publication.motor_layout)) {
        dcu::ethercat::motor_adapter::clear_layout_commands();
        *publication.board = {};
        motor_pdo::store(publication.image, 0, *publication.board);
    }
    if (profile == 3) {
        clear_event_image();
    } else {
        // Raw and motor mappings share the SSC image; restore the motor template on reentry.
        memset(publication.image + sizeof(mc_board_tx_t), 0,
               publication.wire.bytes(profile, true) - sizeof(mc_board_tx_t));
        if (motor) {
            constexpr auto empty = empty_feedback();
            for (unsigned i = 0; i < publication.slot_count; ++i)
                if (publication.slots[i].tx) motor_pdo::store(publication.slots[i].tx, 0, empty);
        }
    }
    publication.layout = profile;
    publication.motor_layout = motor;
}

void publish_board_and_channels(uint32_t online, uint32_t faults)
{
    const auto &snapshot = publication.snapshot;
    auto &board = *publication.board;
    board = {snapshot.sequence, snapshot.accepted_cycle_sequence, snapshot.board_status,
        snapshot.online_channels, snapshot.fault_channels, online, faults,
        snapshot.rx_drops, snapshot.tx_drops, 0};
    motor_pdo::store(publication.image, 0, board);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        const auto &channel = snapshot.channels[c];
        const auto &slot = publication.channels[c];
        auto &mirror = *slot.feedback;
        mirror = {channel.status,
            static_cast<uint8_t>((online >> (c * ENCOS_MOTORS_PER_CHANNEL)) & 0x7f),
            static_cast<uint8_t>((faults >> (c * ENCOS_MOTORS_PER_CHANNEL)) & 0x7f),
            static_cast<uint16_t>(channel.rx_drops > UINT16_MAX ? UINT16_MAX : channel.rx_drops),
            static_cast<uint16_t>(channel.tx_drops > UINT16_MAX ? UINT16_MAX : channel.tx_drops)};
        if (publication.motor_layout && slot.tx) motor_pdo::store(slot.tx, 0, mirror);
    }
}

using dcu::app::gateway_application::Publication;
using dcu::ethercat::motor_pdo::detail::MotorSlot;

DCU_FAST_CODE void write_feedback(const MotorSlot &binding, const Publication::Device &sample)
{
    auto &mirror = *binding.feedback;
    mirror = dcu::ethercat::motor_axis_codec::feedback(sample.feedback);
    dcu::ethercat::motor_axis_codec::diagnostics(sample.diagnostics, *binding.diagnostics);
    if (publication.motor_layout && binding.tx) motor_pdo::store(binding.tx, 0, mirror);
}

DCU_FAST_CODE bool stage_feedback(unsigned device, const Publication::Device &sample, bool online, bool fault)
{
    if (!publication.motor_layout || device >= DCU_MOTOR_DEVICE_CAPACITY) return false;
    const unsigned index = publication.binding_for_device[device];
    if (!index) return false;
    const auto &binding = publication.slots[index - 1];
    if (!binding.tx) return false;
    write_feedback(binding, sample);
    const auto bit = 1U << binding.slot;
    publication.online_bits = (publication.online_bits & ~bit) | (online ? bit : 0);
    publication.fault_bits = (publication.fault_bits & ~bit) | (fault ? bit : 0);
    return true;
}

void publish_axes()
{
    for (unsigned i = 0; i < publication.slot_count; ++i) {
        const auto &slot = publication.slots[i];
        const auto &entry = publication.snapshot.devices[slot.device];
        write_feedback(slot, entry);
    }
}
}

const pdo_layout::Layout &dcu::ethercat::pdo_layout::active() { return publication.wire; }
const pdo_layout::MotorOffsets &dcu::ethercat::pdo_layout::input_offsets() { return publication.motor_rx; }
void pdo_layout::MotorOffsets::bind(const Layout &layout, unsigned mode, bool tx)
{
    for (auto &v : channels) v = absent;
    for (auto &v : motors) v = absent;
    sideband = absent;
    layout.visit(mode, tx, [&](const auto &block, unsigned pos) {
        const unsigned n = block.mapping - (tx ? 0x1a00 : 0x1600);
        if (n >= 1 && n <= ENCOS_CHANNEL_COUNT) channels[n - 1] = pos;
        else if (n >= 0x20 && n < 0x20 + ENCOS_AXIS_COUNT) motors[n - 0x20] = pos;
        else if (block.object == (tx ? 0x6400 : 0x7300)) sideband = pos;
    });
}
const pdo_layout::Layout &dcu::ethercat::pdo_layout::event_layout() { return publication.event; }

namespace dcu::ethercat::motor_pdo {
static_assert(app::config::raw_can::batch_capacity >= CANFD_GATEWAY_SLOTS_PER_BUS);
static_assert(static_cast<unsigned>(app::gateway_application::OperatingMode::Motor) == 0 &&
              static_cast<unsigned>(app::gateway_application::OperatingMode::MotorWithRaw) == 1 &&
              static_cast<unsigned>(app::gateway_application::OperatingMode::Raw) == 2 &&
              static_cast<unsigned>(app::gateway_application::OperatingMode::SingleFd) == 3 &&
              pdo_layout::profile_count == 4);

bool bind_publication(const dcu_production_profile_t &profile)
{
    uint16_t capacity = 0;
    publication.image = static_cast<uint8_t *>(dcu_ssc_txpdo_buffer(&capacity));
    if (!publication.image || capacity < pdo_layout::max_input_bytes) return false;
    publication.board = object<mc_board_tx_t>(0x6000);
    publication.raw = object<layout::MappedSideband>(0x6400);
    publication.event_header = object<mc_event_header_t>(0x6405);
    if (!publication.board || !publication.raw || !publication.event_header) return false;
    layout::Layout wire{};
    for (size_t i = 0; i < profile.channel_count; ++i) {
        if (profile.channels[i].bus >= ENCOS_CHANNEL_COUNT) return false;
        wire.buses |= 1U << profile.channels[i].bus;
    }
    for (size_t i = 0; i < profile.device_count; ++i) {
        const auto &device = profile.devices[i];
        const unsigned slot = motor_adapter::slot_for_device(i);
        if (slot >= ENCOS_AXIS_COUNT || device.bus >= ENCOS_CHANNEL_COUNT || !(wire.buses & (1U << device.bus))) return false;
        const auto bit = 1U << slot;
        wire.feedback_axes |= bit;
        if (device.protocol == DCU_MOTOR_PROTOCOL_ENCOS || device.model_profile_id ||
            device.neutral_mit_only || device.motion_parameters_pending)
            wire.command_axes |= bit;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &slot = publication.channels[c];
        slot.feedback = object<mc_channel_tx_t>(0x6100 + c);
        publication.single_fd[c] = object<mc_event_data_t>(0x6401 + c);
        if (!publication.single_fd[c]) return false;
        slot.raw = object<canfd_bus_pdo_t>(0x6500 + c * 16);
        if (!slot.feedback || !slot.raw) return false;
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
        if (!axis_feedback(a) || !axis_diagnostics(a)) return false;
    select_publication(wire);
    app::gateway_application::bind_feedback_writer(stage_feedback);
    return true;
}

void select_publication(const layout::Layout &wire)
{
    publication.wire = wire;
    std::memset(publication.image, 0, pdo_layout::max_input_bytes);
    *publication.board = {};
    publication.raw_tx = nullptr;
    publication.slot_count = 0;
    publication.online_bits = publication.fault_bits = 0;
    std::memset(publication.binding_for_device, 0, sizeof(publication.binding_for_device));
    publication.layout = pdo_layout::profile_count;
    publication.motor_layout = false;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &slot = publication.channels[c];
        const bool selected = (wire.buses & (1U << c)) != 0;
        slot.tx = nullptr;
        slot.raw_tx = selected ? publication.image + wire.raw_bus(true, c) : nullptr;
        *slot.feedback = {};
        *slot.raw = {};
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a) {
        auto *feedback = axis_feedback(a);
        auto *diagnostics = axis_diagnostics(a);
        *diagnostics = {};
        *feedback = empty_feedback();
        diagnostics->PositionQueryAgeUs = diagnostics->VelocityQueryAgeUs = UINT32_MAX;
        const auto device = motor_adapter::device_for_slot(a);
        if (device >= DCU_MOTOR_DEVICE_CAPACITY) continue;
        publication.slots[publication.slot_count++] = {
            feedback, diagnostics, nullptr, layout::absent,
            static_cast<uint8_t>(a), static_cast<uint8_t>(device)};
        publication.binding_for_device[device] = publication.slot_count;
    }
}

static inline void decode_sideband(const mc_classic_slot_t &wire, uint32_t cycle, app::RawCanBatch &batch)
{
    batch = {};
    batch.sequence = cycle;
    if (!wire.Valid) { batch.valid = true; return; }
    if (wire.Valid != 1 || !dcu_profile_pdo::fits_classic_slot(wire.CanId, wire.Dlc)) return;
    auto &slot = batch.slots[0];
    slot.bus = static_cast<uint8_t>(dcu_profile_pdo::sideband_bus);
    slot.frame.id = wire.CanId;
    slot.frame.dlc = wire.Dlc;
    std::memcpy(slot.frame.data, wire.Data, wire.Dlc);
    batch.count = 1;
    batch.valid = true;
}

static inline void decode_sideband(const mc_sideband_t &raw, uint32_t, app::RawCanBatch &batch)
{
    batch = {};
    batch.sequence = raw.BatchSeq;
    if ((raw.ValidMask & ~3u) || raw.SlotCount > 2 || raw.Flags || raw.DropCount ||
        raw.Reserved || raw.Reserved1 || raw.Reserved2)
        return;
    batch.valid = true;
    for (unsigned i = 0; i < 2; ++i) {
        if (!(raw.ValidMask & (1u << i))) continue;
        const auto wire = sideband_slot(raw, i);
        auto &slot = batch.slots[batch.count++];
        slot.bus = wire.channel;
        slot.frame.id = wire.id;
        slot.frame.flags = wire.flags;
        slot.frame.dlc = wire.dlc;
        std::memcpy(slot.frame.data, wire.data, sizeof(wire.data));
    }
}

void decode_cycle(const void *image, app::gateway_application::OperatingMode mode,
                  app::gateway_application::CycleCommand &command)
{
    if (mode == app::gateway_application::OperatingMode::SingleFd) {
        auto &batch = command.raw[0];
        batch = {};
        batch.kind = app::RawCanBatch::Kind::SingleFd;
        const auto *data = static_cast<const uint8_t *>(image);
        batch.sequence = dcu_can_event::read32(data);
        if (!batch.sequence) return;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            dcu_can_event::Route route{};
            const auto entry = dcu_can_event::decode(data, c, route);
            if (entry == dcu_can_event::Entry::Empty) continue;
            if (entry != dcu_can_event::Entry::Frame || !(publication.event.buses & (1U << c))) return;
            auto &out = batch.slots[batch.count];
            out.bus = c;
            out.frame.id = route.id;
            out.frame.flags = route.flags;
            out.frame.dlc = route.dlc;
            std::memcpy(out.frame.data, data + dcu_can_event::offset(publication.event.buses, c), sizeof(out.frame.data));
            ++batch.count;
        }
        batch.valid = true;
        return;
    }
    const auto b = board(image);
    command.board = {b.CycleSeq, 0, b.BoardControl, b.CommandLeaseMs, b.ChannelEnableMask,
        b.InterfaceVersion == MC_INTERFACE_VERSION && !b.Reserved0 && !b.Reserved1 &&
        !b.Reserved2 && !b.Reserved3 && !b.Reserved4 && !(b.AxisEnableMask >> ENCOS_AXIS_COUNT)};
    command.active_devices = 0;
    command.board.unmapped_device_requested = b.AxisEnableMask != 0;
    if (mode == app::gateway_application::OperatingMode::Raw) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
            if (publication.wire.buses & (1U << c)) decode_raw_bus(raw_bus(image, c), c, command.raw[c]);
            else command.raw[c] = {};
    } else {
        uint32_t requested_slots = 0;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            if (!(publication.wire.buses & (1U << c))) { command.channels[c] = {}; continue; }
            const auto ch = channel(image, c);
            command.channels[c] = {ch.ChannelControl, ch.ExpectedRole, !(ch.ActiveMask & 0x80), ch.Reserved};
            requested_slots |= static_cast<uint32_t>(ch.ActiveMask & 0x7fu) << (c * ENCOS_MOTORS_PER_CHANNEL);
        }
        for (unsigned i = 0; i < publication.slot_count; ++i) {
            const auto &slot = publication.slots[i];
            if (slot.rx_offset == pdo_layout::absent) { command.devices[slot.device] = {}; continue; }
            const uint32_t bit = 1U << slot.slot;
            if (b.AxisEnableMask & bit) command.board.enabled_devices |= 1U << slot.device;
            if (requested_slots & bit) command.active_devices |= 1U << slot.device;
            command.devices[slot.device] = motor_axis_codec::command(load<mc_axis_rx_t>(image, slot.rx_offset));
        }
        command.board.unmapped_device_requested = (b.AxisEnableMask & ~publication.wire.command_axes) != 0;
        if (mode == app::gateway_application::OperatingMode::MotorWithRaw)
            decode_sideband(raw(image), b.CycleSeq, command.raw[0]);
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

static unsigned indices_for(const layout::Layout &wire, unsigned profile, bool tx, uint16_t *indices)
{
    unsigned n = 0;
    wire.visit(profile, tx, [&](const auto &block, unsigned) { indices[n++] = block.mapping; });
    return n;
}
}

bool motor_control_verify_contract(void)
{
    static_assert(MC_INTERFACE_VERSION == dcu_profile_pdo::interface_version);
    const auto &boot = dcu_production_profile()->readiness;
    static_assert(sizeof(boot.digest) == sizeof(dcu_profile_pdo::profile_digest));
    if (boot.profile_id != dcu_profile_pdo::profile_id || boot.revision != dcu_profile_pdo::profile_revision ||
        memcmp(boot.digest, dcu_profile_pdo::profile_digest, sizeof(boot.digest))) return false;
    const auto *identity = object<mc_layout_identity_t>(0x2102);
    if (!identity || memcmp(identity->Sha256, dcu_profile_pdo::digest, sizeof(identity->Sha256))) return false;
    for (unsigned p = 0; p < pdo_layout::profile_count; ++p) {
        for (unsigned tx = 0; tx < 2; ++tx) {
            const auto &image = tx ? dcu_profile_pdo::modes[p].tx : dcu_profile_pdo::modes[p].rx;
            uint16_t indices[pdo_layout::assignment_capacity];
            if (motor_pdo::indices_for(publication.wire, p, tx, indices) != image.count ||
                publication.wire.bytes(p, tx) != image.bytes)
                return false;
            for (unsigned i = 0; i < image.count; ++i)
                if (indices[i] != image.blocks[i].mapping) return false;
            if (p == static_cast<unsigned>(dcu::app::gateway_application::OperatingMode::MotorWithRaw)) {
                const auto &assigned = tx ? gTxAssign : gRxAssign;
                if (assigned.u16SubIndex0 != image.count || memcmp(assigned.aEntries, indices, image.count * sizeof(*indices)))
                    return false;
            }
        }
    }
    return true;
}
static uint16_t checked_mapping(uint16_t *in, uint16_t *out, bool commit)
{
    *in = *out = 0;
    if ((nAlStatus & STATE_MASK) != STATE_PREOP) return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
    namespace gateway = dcu::app::gateway_application;
    namespace adapter = dcu::ethercat::motor_adapter;
    const bool hot = gateway::topology_pending();
    if (hot != commit) return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
    for (unsigned p = 0; p < pdo_layout::profile_count; p++) {
        if (hot && p != static_cast<unsigned>(gateway::OperatingMode::MotorWithRaw)) continue;
        uint16_t rx[pdo_layout::assignment_capacity], tx[pdo_layout::assignment_capacity];
        auto wire = adapter::requested_layout();
        if (p == 3) {
            const auto count = gRxAssign.u16SubIndex0;
            if (count < 2 || count > ENCOS_CHANNEL_COUNT + 1 || gRxAssign.aEntries[0] != 0x165F)
                continue;
            uint32_t buses = 0;
            for (unsigned i = 1; i < count; ++i) {
                const auto index = gRxAssign.aEntries[i];
                if (index < 0x1660 || index >= 0x1660 + ENCOS_CHANNEL_COUNT) {
                    buses = 0;
                    break;
                }
                buses |= 1U << (index - 0x1660);
            }
            if (!buses || (buses & ~wire.buses))
                continue;
            wire.buses = buses;
        }
        unsigned nr = motor_pdo::indices_for(wire, p, false, rx);
        unsigned nt = motor_pdo::indices_for(wire, p, true, tx);
        if (gRxAssign.u16SubIndex0 == nr && gTxAssign.u16SubIndex0 == nt && !memcmp(rx, gRxAssign.aEntries, nr * 2) && !memcmp(tx, gTxAssign.aEntries, nt * 2)) {
            const bool mapped = hot ? adapter::commit_selection() :
                                      gateway::map_layout(static_cast<gateway::OperatingMode>(p), p == 3 ? wire.buses : 0);
            if (!mapped)
                return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            if (p == 3) publication.event = wire;
            latch_layout(p);
            if (p == 3) publication.slave_match = static_cast<uint8_t>(dcu_ssc_station_address());
            object<mc_board_config_t>(0x2001)->ActivePdoProfile = p;
            *in = wire.bytes(p, true);
            *out = wire.bytes(p, false);
            return 0;
        }
    }
    return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
}
uint16_t motor_control_mapping(uint16_t *in, uint16_t *out)
{ return checked_mapping(in, out, false); }

bool motor_control_commit_topology(uint32_t generation)
{
    uint16_t in, out;
    return generation && generation - 1 == dcu::ethercat::motor_adapter::topology_generation() &&
        checked_mapping(&in, &out, true) == 0;
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

void motor_control_clear_raw_input(void)
{
    // 各档共用 SSC 的输入镜像，只擦当前映射的透传区，不碰电机字节。
    if (publication.layout == 3) {
        clear_event_image();
    } else if (publication.layout == 1) {
        *publication.raw = {};
        motor_pdo::store(publication.raw_tx, 0, *publication.raw);
    } else if (publication.layout == 2) {
        for (const auto &slot : publication.channels) {
            if (!slot.raw_tx) continue;
            *slot.raw = {};
            motor_pdo::store(slot.raw_tx, 0, *slot.raw);
        }
    }
}

namespace {
inline void publish_mixed(mc_classic_slot_t &slot, bool active)
{
    namespace gateway = dcu::app::gateway_application;
    slot = {};
    const auto bus = static_cast<unsigned>(dcu_profile_pdo::sideband_bus);
    dcu_can_frame_t frame;
    if (active && bus < ENCOS_CHANNEL_COUNT && (publication.wire.buses & (1U << bus)) &&
        gateway::take_raw_frame(bus, frame) &&
        dcu_profile_pdo::fits_classic_slot(frame.id, frame.dlc, frame.flags)) {
        slot.CanId = static_cast<uint16_t>(frame.id);
        slot.Dlc = frame.dlc;
        slot.Valid = 1;
        memcpy(slot.Data, frame.data, slot.Dlc);
    }
    motor_pdo::store(publication.raw_tx, 0, slot);
}

inline void publish_mixed(mc_sideband_t &raw, bool active)
{
    raw = {};
    if (active) {
        raw.BatchSeq = ++publication.raw_batch;
        raw.DropCount = publication.board->RxDropCount;
        for (unsigned i = 0; i < 2; i++)
            for (unsigned n = 0; n < ENCOS_CHANNEL_COUNT; n++) {
                unsigned c = publication.round_robin;
                publication.round_robin = (c + 1) % ENCOS_CHANNEL_COUNT;
                dcu_can_frame_t f;
                if (!dcu::app::gateway_application::take_raw_frame(c, f)) continue;
                motor_pdo::SidebandSlot slot{};
                slot.id = f.id;
                slot.channel = c;
                slot.flags = f.flags;
                slot.dlc = f.dlc;
                memcpy(slot.data, f.data, sizeof(slot.data));
                motor_pdo::write_sideband_slot(raw, i, slot);
                raw.ValidMask |= 1u << i;
                raw.SlotCount++;
                break;
            }
    }
    motor_pdo::store(publication.raw_tx, 0, raw);
}
}

bool motor_control_prepare_event_input(bool allow_empty, bool *has_frame)
{
    namespace gateway = dcu::app::gateway_application;
    bool any = false;
    const auto mask = publication.event.buses;
    if (gateway::raw_state() == gateway::RawState::Active) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
            dcu_can_frame_t frame;
            if (!(mask & (1U << c)) || !gateway::take_raw_frame(c, frame) || !dcu_can_frame_valid(&frame)) continue;
            if (!any) clear_event_image();
            any = true;
            dcu_can_event::encode(publication.image, c, {frame.id, frame.flags, frame.dlc});
            const auto offset = dcu_can_event::offset(mask, c);
            const auto size = dcu_can_dlc_to_bytes(frame.dlc, frame.flags & DCU_CAN_FLAG_FD);
            memcpy(publication.image + offset, frame.data, size);
            memcpy(publication.single_fd[c], publication.image + offset, dcu_can_event::data_bytes);
        }
    }
    if (!any && !allow_empty) return false;
    if (!any) clear_event_image();
    if (++publication.raw_batch == 0) ++publication.raw_batch;
    dcu_can_event::write32(publication.image, publication.raw_batch);
    memcpy(publication.event_header, publication.image, dcu_can_event::header_bytes);
    *has_frame = any;
    return true;
}

DCU_FAST_CODE bool motor_control_prepare_cyclic_input(void)
{
    namespace gateway = dcu::app::gateway_application;
    if (!publication.motor_layout || publication.layout != static_cast<unsigned>(gateway::mode()) ||
        !gateway::finish_cyclic_publication(publication.snapshot)) return false;
    publish_board_and_channels(publication.online_bits, publication.fault_bits);
    if (publication.layout == static_cast<unsigned>(gateway::OperatingMode::MotorWithRaw))
        publish_mixed(*publication.raw, gateway::raw_state() == gateway::RawState::Active);
    return true;
}

DCU_FAST_CODE void motor_control_prepare_input(void)
{
    namespace gateway = dcu::app::gateway_application;
    if (gateway::mode() == gateway::OperatingMode::SingleFd) {
        bool data;
        motor_control_prepare_event_input(true, &data);
        return;
    }
    auto &snapshot = publication.snapshot;
    dcu::app::gateway_application::prepare_publication(snapshot);
    const unsigned profile = static_cast<unsigned>(snapshot.mode);
    latch_layout(profile);
    publish_board_and_channels(dcu::ethercat::motor_adapter::slots_for_devices(snapshot.online_devices),
                               dcu::ethercat::motor_adapter::slots_for_devices(snapshot.fault_devices));
    if (publication.motor_layout) publish_axes();
    const bool raw_active = dcu::app::gateway_application::raw_state() == dcu::app::gateway_application::RawState::Active;
    if (profile == 1) {
        publish_mixed(*publication.raw, raw_active);
        return;
    }
    if (!raw_active) motor_control_clear_raw_input();
    if (profile == 2) {
        if (!raw_active) return;
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
            const auto &slot = publication.channels[c];
            if (!slot.raw_tx) continue;
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
    }
}
