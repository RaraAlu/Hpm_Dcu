#include "motor_ethercat.h"
#include "head_health.hpp"
#include "motor_dictionary.hpp"
#include "motor_pdo_layout.hpp"
#include "gateway_application.hpp"
#include "motor_services.hpp"
#include "motor_adapter.hpp"
#include "realtime_port.h"
extern "C" {
#include "digital_io.h"
#include "ecatslv.h"
#include "sdoserv.h"
}
#include <cstring>

using namespace dcu::ethercat::motor_dictionary;
using namespace dcu::app::gateway_application;
namespace profile_update = dcu::app::profile_update;
namespace pdo_layout = dcu::ethercat::pdo_layout;
namespace adapter = dcu::ethercat::motor_adapter;

namespace {
static_assert(pdo_layout::assignment_capacity == sizeof(TMC_ASSIGN::aEntries) / sizeof(uint16_t));
constexpr uint16_t profile_update_index = 0x2100;
constexpr unsigned candidate_bytes = sizeof(mc_profile_update_t::Candidate);

uint32_t load_le(const uint8_t *p, unsigned n)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < n; ++i) value |= uint32_t{p[i]} << (8U * i);
    return value;
}

uint32_t crc32(const uint8_t *p, unsigned n)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0; i < n; ++i) {
        crc ^= p[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
    return ~crc;
}

bool stage_profile_candidate(const void *data, size_t bytes)
{
    const auto *p = static_cast<const uint8_t *>(data);
    if (!p || bytes != candidate_bytes || load_le(p, 2) != 1 || load_le(p + 2, 2) != bytes ||
        crc32(p, candidate_bytes - 4) != load_le(p + 32, 4))
        return profile_update::stage(nullptr);
    // Bytes 16..23 are the legacy Boot ID; still covered by the wire CRC.
    const profile_update::Candidate next{
        load_le(p + 4, 4), load_le(p + 8, 4), load_le(p + 12, 4),
        {static_cast<uint16_t>(load_le(p + 24, 2)),
         static_cast<uint16_t>(load_le(p + 26, 2)), load_le(p + 28, 4)}};
    return profile_update::stage(&next);
}
}

static bool preop()
{
    return (nAlStatus & STATE_MASK) == STATE_PREOP && preop_access_allowed();
}

static bool reserved_configuration(UINT16 index)
{
    if (index == 0x2002) return true;
    if (index < axis_object_base(0) || index >= axis_object_base(ENCOS_AXIS_COUNT)) return false;
    const unsigned part = (index - axis_object_base(0)) % 16;
    return part <= 4 || part == 8;
}

UINT8 MC_AssignWrite(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    if ((nAlStatus & STATE_MASK) != STATE_PREOP)
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    TMC_ASSIGN *v = index == 0x1C12 ? &gRxAssign : &gTxAssign;
    if (sub > pdo_layout::assignment_capacity)
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (bytes != (sub ? 2u : 1u))
        return ABORTIDX_PARAM_LENGTH_ERROR;
    uint16_t value = 0;
    memcpy(&value, data, bytes);
    if (!sub) {
        if (value > pdo_layout::assignment_capacity)
            return ABORTIDX_VALUE_EXCEEDED;
        v->u16SubIndex0 = value;
    } else {
        if (v->u16SubIndex0)
            return ABORTIDX_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0;
        v->aEntries[sub - 1] = value;
    }
    return 0;
}
UINT8 MC_OD_Read(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    mc_object_t *o = find(index);
    if (!o)
        return ABORTIDX_OBJECT_NOT_EXISTING;
    if (!sub) {
        if (bytes != 1)
            return ABORTIDX_PARAM_LENGTH_ERROR;
        *((uint8_t *)data) = (uint8_t)o->count;
        return 0;
    }
    // Existing reserved board record: same indices, sizes and read-only descriptors.
    if (index == 0x2002) {
        if (sub > 4) return ABORTIDX_SUBINDEX_NOT_EXISTING;
        if (bytes != (sub == 3 ? 2U : 4U)) return ABORTIDX_PARAM_LENGTH_ERROR;
        const auto status = readiness();
        const uint32_t values[] = {static_cast<uint32_t>(status.boot_id >> 32), status.ready_generation,
            static_cast<uint32_t>(status.state) | (static_cast<uint32_t>(status.reason) << 8),
            0U};
        std::memcpy(data, &values[sub - 1], bytes);
        return 0;
    }
    if (index == profile_update_index) {
        if (sub < 3) return ABORTIDX_WRITE_ONLY_ENTRY;
        if (sub > 7) return ABORTIDX_SUBINDEX_NOT_EXISTING;
        if (bytes != (sub == 3 ? 2U : 4U)) return ABORTIDX_PARAM_LENGTH_ERROR;
        const auto active = readiness();
        const uint32_t values[] = {static_cast<uint16_t>(profile_update::status()), active.profile_id,
            active.revision, profile_update::staged_revision(), profile_policy().readiness_ttl_ms};
        memcpy(data, &values[sub - 3], bytes);
        return 0;
    }
    if (index == DCU_SDO_2101_PDO_LAYOUT) {
        if (sub < 3) return ABORTIDX_WRITE_ONLY_ENTRY;
        if (sub > 8) return ABORTIDX_SUBINDEX_NOT_EXISTING;
        if (bytes != (sub == 3 || sub == 6 ? 1u : 4u)) return ABORTIDX_PARAM_LENGTH_ERROR;
        uint32_t value = 0;
        switch (sub) {
        case 3: value = static_cast<uint8_t>(raw_state()); break;
        case 4: value = raw_drops(); break;
        case 5: value = pure_raw_limits(); break;
        case 6: value = raw_motor_owner(); break;
        default:
            for (unsigned bus = 0; bus < DCU_CAN_BUS_COUNT; ++bus)
                value += sub == 7 ? dcu_can_tx_progress(bus) : dcu_can_raw_tx_cancelled(bus);
            break;
        }
        memcpy(data, &value, bytes);
        return 0;
    }
    if (index == DCU_SDO_2103_PDO_TOPOLOGY) {
        if (sub == 1 || sub == 6) return ABORTIDX_WRITE_ONLY_ENTRY;
        if (sub > 5) return ABORTIDX_SUBINDEX_NOT_EXISTING;
        if (bytes != (sub == 5 ? 8U : 4U)) return ABORTIDX_PARAM_LENGTH_ERROR;
        const uint64_t value = sub == 2 ? adapter::topology_generation() : sub == 3 ?
            pdo_layout::active().buses : sub == 4 ? pdo_layout::active().feedback_axes : readiness().boot_id;
        memcpy(data, &value, bytes);
        return 0;
    }
    // Project the immutable active Profile through existing read-only descriptors.
    if (index == 0x2104) {
        const auto &h = dcu::head::health(dcu_time_now_ms());
        *object<mc_head_health_t>(index) = {static_cast<uint8_t>(h.monitoring), h.heartbeat_seen, h.offline,
            h.feedback_seen, h.stale, h.device_errors, h.last_error[0], h.last_error[1], h.last_error[2], h.last_error[3]};
    }
    if (index >= axis_object_base(0) && index < axis_object_base(ENCOS_AXIS_COUNT)) {
        const unsigned offset = index - axis_object_base(0);
        const auto *device = adapter::configured_device(offset / 16);
        if (device && offset % 16 == 0 && sub <= 7) {
            const uint32_t fields[] = {device->can_id, device->model_profile_id,
                static_cast<uint32_t>(device->position_min_urad), static_cast<uint32_t>(device->position_max_urad),
                device->velocity_limit_urad_s, device->current_limit_dA, device->torque_limit_mNm};
            if (bytes != (sub <= 2 ? 2U : 4U)) return ABORTIDX_PARAM_LENGTH_ERROR;
            memcpy(data, &fields[sub - 1], bytes);
            return 0;
        }
        if (device && device->protocol == DCU_MOTOR_PROTOCOL_ENCOS && offset % 16 == 2 && sub <= 4) {
            const auto &profile = device->encos.profile;
            const double fields[] = {profile.kp[0], profile.kp[1], profile.kd[0], profile.kd[1]};
            if (bytes != 2U) return ABORTIDX_PARAM_LENGTH_ERROR;
            const auto value = static_cast<uint16_t>(fields[sub - 1]);
            memcpy(data, &value, bytes);
            return 0;
        }
    }
    if (reserved_configuration(index))
        return ABORTIDX_UNSUPPORTED_ACCESS;
    for (unsigned i = 0; i < mc_field_count; i++) {
        const mc_field_t *f = &mc_fields[i];
        if (f->index != index || f->sub != sub)
            continue;
        if (!(f->access & 1))
            return ABORTIDX_WRITE_ONLY_ENTRY;
        if (bytes != f->bytes)
            return ABORTIDX_PARAM_LENGTH_ERROR;
        if (index >= 0x6200 && index < 0x6200 + ENCOS_AXIS_COUNT)
            adapter::refresh_axis_feedback(index - 0x6200);
        else if (index >= 0x2400 && index < 0x2400 + 16 * ENCOS_AXIS_COUNT &&
                 (index - 0x2400) % 16 == 0)
            adapter::refresh_axis_feedback((index - 0x2400) / 16);
        if ((index == 0x6400 && (mode() != OperatingMode::MotorWithRaw || raw_state() != RawState::Active)) ||
            (index >= 0x6401 && index <= 0x6405 && (mode() != OperatingMode::SingleFd || raw_state() != RawState::Active)) ||
            (index >= 0x6500 && index <= 0x6530 && !(index & 15) &&
             (mode() != OperatingMode::Raw || raw_state() != RawState::Active)))
            memset(data, 0, bytes);
        else memcpy(data, o->data + f->offset, bytes);
        return 0;
    }
    return ABORTIDX_SUBINDEX_NOT_EXISTING;
}
namespace {
struct WriteEntry {
    mc_object_t *object;
    const mc_field_t *field;
};

UINT8 validate_write(UINT16 index, UINT8 sub, UINT32 bytes, UINT8 complete, WriteEntry *entry = nullptr)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    mc_object_t *o = find(index);
    if (!o)
        return ABORTIDX_OBJECT_NOT_EXISTING;
    if (!sub)
        return ABORTIDX_READ_ONLY_ENTRY;
    if (reserved_configuration(index))
        return ABORTIDX_READ_ONLY_ENTRY;
    const mc_field_t *field = NULL;
    for (unsigned i = 0; i < mc_field_count; i++)
        if (mc_fields[i].index == index && mc_fields[i].sub == sub) {
            field = &mc_fields[i];
            break;
        }
    if (!field)
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (!(field->access & 2))
        return ABORTIDX_READ_ONLY_ENTRY;
    bool command = index >= 0x2200 && index <= 0x23B0, owner = index == 0x2003;
    if (index != DCU_SDO_2101_PDO_LAYOUT && !command && !owner && !preop())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (bytes != field->bytes) {
        if (index == profile_update_index && sub == 1)
            profile_update::discard();
        return ABORTIDX_PARAM_LENGTH_ERROR;
    }
    if (entry) *entry = {o, field};
    return 0;
}
}

UINT8 dcu_pdo_topology_sdo_write(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (index != DCU_SDO_2103_PDO_TOPOLOGY) return ABORTIDX_UNSUPPORTED_ACCESS;
    if (const auto error = validate_write(index, sub, bytes, complete)) return error;
    static_assert(sizeof(mc_topology_t::Selection) == 88);
    const auto *p = reinterpret_cast<const uint8_t *>(data);
    if (sub == 6) return motor_control_commit_topology(load_le(p, 4)) ? 0 : ABORTIDX_VALUE_EXCEEDED;
    if (sub != 1) return ABORTIDX_SUBINDEX_NOT_EXISTING;
    const auto active = readiness();
    const auto *identity = object<mc_layout_identity_t>(0x2102);
    // The first eight bytes are the legacy Boot ID, not a selection precondition.
    if (load_le(p + 8, 4) != active.revision ||
        memcmp(p + 24, identity->Sha256, sizeof(identity->Sha256)) || memcmp(p + 56, active.digest, sizeof(active.digest)) ||
        !adapter::stage_selection(load_le(p + 16, 4), load_le(p + 20, 4), load_le(p + 12, 4)))
        return ABORTIDX_VALUE_EXCEEDED;
    profile_update::discard();
    return 0;
}

UINT8 dcu_pdo_layout_sdo_write(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (index != DCU_SDO_2101_PDO_LAYOUT) return ABORTIDX_UNSUPPORTED_ACCESS;
    if (const auto error = validate_write(index, sub, bytes, complete)) return error;
    const auto *wire = reinterpret_cast<const uint8_t *>(data);
    if (sub == 5) {
        if (!preop()) return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
        return set_pure_raw_limits(load_le(wire, 4)) ? 0 : ABORTIDX_VALUE_EXCEEDED;
    }
    if (sub == 6) {
        if (!preop()) return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
        return stage_raw_motor_owner(*wire) ? 0 : ABORTIDX_VALUE_EXCEEDED;
    }
    if (sub == 1) {
        const auto al = nAlStatus & STATE_MASK;
        if (al != STATE_OP && al != STATE_PREOP) return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
        static_assert(sizeof(mc_raw_switch_t::LayoutRequest) == 9 + DCU_GATEWAY_DIGEST_BYTES);
        uint8_t digest[DCU_GATEWAY_DIGEST_BYTES];
        memcpy(digest, wire + 9, sizeof(digest));
        if (!request_layout(static_cast<OperatingMode>(wire[0]), load_le(wire + 1, 4),
                            load_le(wire + 5, 4), digest)) return ABORTIDX_VALUE_EXCEEDED;
        motor_control_clear_raw_input();
        return 0;
    }
    if (sub != 2) return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (*wire == 2) { // 主站报告 WKC 或会话故障。
        motor_control_raw_link_lost();
        return 0;
    }
    if (*wire != 1) return ABORTIDX_VALUE_EXCEEDED;
    return (nAlStatus & STATE_MASK) == STATE_PREOP && reset_layout() ? 0 :
        ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
}

UINT8 MC_OD_Write(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (index == DCU_SDO_2101_PDO_LAYOUT || index == DCU_SDO_2103_PDO_TOPOLOGY)
        return ABORTIDX_UNSUPPORTED_ACCESS; // Reload records must never fall through to a dictionary write.
    WriteEntry entry;
    if (const auto error = validate_write(index, sub, bytes, complete, &entry)) return error;
    if (index == profile_update_index) {
        if (topology_pending()) return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
        if (sub == 1)
            return stage_profile_candidate(data, bytes) ? 0 : ABORTIDX_VALUE_EXCEEDED;
        uint32_t revision = 0;
        memcpy(&revision, data, sizeof(revision));
        if (!profile_update::commit(revision)) return ABORTIDX_VALUE_EXCEEDED;
        const auto policy = profile_policy();
        auto *board = object<mc_board_config_t>(0x2001);
        board->MaxCommandLeaseMs = policy.max_command_lease_ms;
        board->FeedbackTimeoutMs = policy.feedback_timeout_ms;
        return 0;
    }
    if (index == 0x2003 && !owner_change_allowed())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (index >= 0x3007 && index <= 0x31B7 && (index - 0x3007) % 16 == 0 &&
        parameter_service_busy(((index - 0x3007) / 16) / ENCOS_MOTORS_PER_CHANNEL))
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    memcpy(entry.object->data + entry.field->offset, data, bytes);
    uint32_t now = dcu_time_now_ms();
    UINT8 result = 0;
    if (index == 0x2003) {
        mc_owner_t *v = object<mc_owner_t>(index);
        if (v->RequestedOwner > 2) {
            result = ABORTIDX_VALUE_EXCEEDED;
            v->Result = dcu::motor::RESULT_INVALID;
        } else {
            const bool accepted = set_owner(static_cast<Owner>(v->RequestedOwner));
            v->Owner = dcu::app::gateway_application::owner();
            v->Result = accepted ? dcu::motor::RESULT_OK : dcu::motor::RESULT_STATE;
            if (!accepted) result = ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
        }
    } else if (index >= 0x2200 && index <= 0x23B0 && sub == 11) {
        unsigned a = (index - 0x2200) / 16;
        adapter::commit_sdo_command(a, now);
    } else if (index >= 0x3000) {
        unsigned a = (index - 0x3000) / 16, part = (index - 0x3000) % 16;
        if (part == 7 && sub == 3)
            result = dcu::ethercat::motor_services::commit(a, now, preop()) ? 0 : ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    }
    return result;
}

void motor_control_discard_profile(void)
{
    profile_update::discard();
}
