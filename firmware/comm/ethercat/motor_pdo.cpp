#include "motor_ethercat.h"
#include "detail/motor_pdo_state.hpp"
#include "motor_application.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include "pdo_pipeline.h"
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
}

namespace dcu::ethercat::motor_pdo {
static_assert(app::config::motor_control::raw_batch_capacity >= CANFD_GATEWAY_SLOTS_PER_BUS);

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
        std::memcpy(slot.frame.data, wire.data, sizeof(slot.frame.data));
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
        std::memcpy(slot.frame.data, wire.data, sizeof(slot.frame.data));
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
            if (!dcu::app::motor_application::select_profile(p))
                return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            object<mc_board_config_t>(0x2001)->ActivePdoProfile = p;
            *in = publication.input_size = pdo_layout::profiles[p].input;
            *out = pdo_layout::profiles[p].output;
            return 0;
        }
    }
    *in = *out = 0;
    return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
}
void motor_control_prepare_input(void)
{
    auto snapshot = dcu::app::motor_application::prepare_publication();
    publication.input_size = pdo_layout::profiles[snapshot.profile].input;
    uint8_t *p = publication.image;
    motor_pdo::write_board(p, *object<mc_board_tx_t>(0x6000));
    if (snapshot.profile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
            auto *raw = static_cast<canfd_bus_pdo_t *>(mc_object(0x6500 + c * 16));
            memset(raw, 0, sizeof(*raw));
            raw->cycle_seq = snapshot.cycle_sequence;
            raw->bus_state = ethercat_canfd_bus_state(c);
            raw->counter0 = ethercat_canfd_rx_drops(c) + snapshot.raw_drops[c];
            dcu_can_frame_t f;
            for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS && dcu::app::motor_application::take_raw_frame(c, f); i++) {
                raw->valid_mask |= 1u << i;
                raw->mode++;
                raw->slot[i].can_id = f.id;
                raw->slot[i].flags = f.flags | 1;
                raw->slot[i].dlc = f.dlc;
                raw->slot[i].seq = ++publication.raw_batch;
                memcpy(raw->slot[i].data, f.data, 8);
            }
            motor_pdo::write_raw_bus(p, c, *raw);
        }
    } else {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
            motor_pdo::write_channel(p, c, *object<mc_channel_tx_t>(0x6100 + c));
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
            motor_pdo::write_axis(p, a, *axis_feedback(a));
        motor_pdo::write_imu(p, *object<mc_imu_t>(0x6300));
        if (snapshot.profile == 1) {
            mc_raw_t *raw = object<mc_raw_t>(0x6400);
            memset(raw, 0, sizeof(*raw));
            bool motor_only = true;
            for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
                if (snapshot.roles[c] >= 2)
                    motor_only = false;
            if (motor_only) {
                uint8_t *d = (uint8_t *)raw;
                uint8_t page = publication.diagnostic_page++ & 1u;
                d[0] = 'T';
                d[1] = '1';
                d[2] = 'R';
                d[3] = '2';
                d[4] = page;
                for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
                    d[5 + a] = (uint8_t)(page ? snapshot.type1_count[a] : ethercat_canfd_completions(a));
            } else {
                raw->BatchSeq = ++publication.raw_batch;
                raw->DropCount = object<mc_board_tx_t>(0x6000)->RxDropCount;
                for (unsigned i = 0; i < 2; i++)
                    for (unsigned n = 0; n < ENCOS_CHANNEL_COUNT; n++) {
                        unsigned c = publication.round_robin;
                        publication.round_robin = (publication.round_robin + 1) % ENCOS_CHANNEL_COUNT;
                        dcu_can_frame_t f;
                        if (!dcu::app::motor_application::take_raw_frame(c, f))
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
            }
            motor_pdo::write_raw(p, *raw);
        }
    }
}
void motor_control_input(void *data)
{
    memcpy(data, publication.image, publication.input_size);
}
