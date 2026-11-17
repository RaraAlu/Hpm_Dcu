#include "encos_parameter_transactions.hpp"
#include "ethercat_canfd_gateway.h"
#include "detail/encos_parameter_state.hpp"
#include "motor_application.hpp"
#include "motor_dictionary.hpp"
#include "cfg/motor_policy.hpp"
#include "encos_motor.h"
#include "device_config.h"
#include <climits>
#include <cstring>

namespace dcu::app::parameters {
using namespace dcu::app::motor_application;
using namespace dcu::ethercat::motor_dictionary;
namespace motor_cfg = dcu::app::config::motor_control;
namespace {
using detail::Phase;
using detail::Purpose;
using detail::service_t;
service_t transactions[ENCOS_CHANNEL_COUNT];

static void finish(unsigned c, uint16_t result)
{
    service_t *t = &transactions[c];
    unsigned a = t->axis;
    if (!t->internal) {
        if (t->purpose == Purpose::Maintenance) {
            mc_maintenance_t *m = object<mc_maintenance_t>(axis_object_base(a) + 8);
            m->DoneSeq = t->seq;
            m->Result = result;
        } else {
            mc_transaction_t *m = object<mc_transaction_t>(axis_object_base(a) + 7);
            m->DoneSeq = t->seq;
            m->Result = result;
        }
    }
    axis_diagnostics(a)->LastServiceError = result;
    t->phase = Phase::Idle;
}

static bool cache_query(unsigned a, uint8_t code, const encos_frame_t *f, service_t *t, Observation &observation)
{
    mc_query_t *q = object<mc_query_t>(axis_object_base(a) + 5);
    int32_t value;
    const uint8_t *d = f->data;
    bool ok = false;
    if (code >= 1 && code <= 4 && f->dlc == 6) {
        double scale = code == 1 ? 1e6 * 3.141592653589793 / 180 : code == 2 ? 1e6 * 2 * 3.141592653589793 / 60
                                                               : code == 3   ? 100
                                                                             : 1000;
        if (!encos_float(d + 2, scale, &value))
            return false;
        uint32_t bits = encos_be32(d + 2);
        float real;
        memcpy(&real, &bits, 4);
        memcpy((uint8_t *)q + 4 * (code - 1), &real, 4);
        if (code == 1) {
            observation.field = Field::Position;
            observation.value = value;
            observation.update_actual = true;
        }
        if (code == 2) {
            observation.field = Field::Velocity;
            observation.value = value;
            observation.update_actual = true;
        }
        if (code == 3) {
            if (value < INT16_MIN || value > INT16_MAX)
                return false;
            observation.field = Field::Current;
            observation.value = value;
            observation.update_actual = true;
        }
        if (code == 4) {
            observation.field = Field::Power;
            observation.value = value;
        }
        q->ValidBits |= 1u << (code - 1);
        ok = true;
    } else if ((code == 5 || code == 22 || code == 31 || code == 38) && f->dlc == 4) {
        uint16_t v = encos_be16(d + 2);
        mc_basic_t *b = object<mc_basic_t>(axis_object_base(a) + 1);
        if (code == 5) {
            q->AccelerationRaw = v;
            b->AccelerationRaw = v;
            q->ValidBits |= 1u << 4;
        }
        if (code == 22)
            b->TorqueConstantRaw = v;
        if (code == 31)
            b->CanTimeoutMs = v;
        if (code == 38) {
            q->ZeroOffsetRaw = (int16_t)v;
            b->ZeroOffsetRaw = (int16_t)v;
            q->ValidBits |= 1u << 9;
        }
        ok = true;
    } else if (((code >= 23 && code <= 28) || (code >= 32 && code <= 34)) && f->dlc == 6) {
        uint16_t v[2] = {encos_be16(d + 2), encos_be16(d + 4)};
        void *dest = code <= 28 ? (uint8_t *)mc_object(axis_object_base(a) + 2) + (code - 23) * 4 : (uint8_t *)mc_object(axis_object_base(a) + 3) + (code - 32) * 4;
        memcpy(dest, v, 4);
        ok = true;
    } else if (code == 29 && f->dlc == 7 && d[2] >= 1 && d[2] <= 3) {
        t->uuid[d[2] - 1] = encos_be32(d + 3);
        t->uuid_mask |= 1u << (d[2] - 1);
        if (t->uuid_mask != 7) {
            return false;
        }
        memcpy(mc_object(axis_object_base(a) + 6), t->uuid, 12);
        ok = true;
    } else if (code == 30 && f->dlc == 8) {
        memcpy((uint8_t *)mc_object(axis_object_base(a) + 6) + 12, d + 2, 6);
        ok = true;
    } else if ((code == 35 || code == 37) && f->dlc == 3) {
        if (code == 35 && d[2] <= 2) {
            object<mc_kt_t>(axis_object_base(a) + 4)->CalibrationMode = d[2];
            ok = true;
        }
        if (code == 37 && d[2] <= 1) {
            q->BrakeState = d[2];
            q->ValidBits |= 1u << 5;
            ok = true;
        }
    }
    if (ok) {
        ++q->SnapshotSeq;
        q->AgeMs = 0;
    }
    return ok;
}
}

Status inspect(unsigned c)
{
    const auto &t = transactions[c];
    return {t.phase != Phase::Idle, t.phase == Phase::AwaitingReply, t.internal,
            t.axis, t.request.id};
}
bool available(unsigned c, uint32_t now)
{
    const auto &t = transactions[c];
    return t.phase == Phase::Idle && (int32_t)(now - t.hold_until) >= 0;
}
const dcu_can_frame_t *pending(unsigned c)
{
    return transactions[c].phase == Phase::Queued ? &transactions[c].request : nullptr;
}
void mark_sent(unsigned c, uint32_t now)
{
    auto &t = transactions[c];
    if (t.phase != Phase::Queued) return;
    t.phase = Phase::AwaitingReply;
    t.started = now;
    if (t.purpose == Purpose::Maintenance || t.parameter == 2)
        t.hold_until = now + motor_cfg::parameter_hold_ms;
}
void check_timeout(unsigned c, uint32_t now)
{
    const auto &t = transactions[c];
    if (t.phase != Phase::Idle && now - t.started >= motor_cfg::parameter_timeout_ms)
        cancel(c, RESULT_TIMEOUT);
}
void cancel(unsigned c, uint16_t result)
{
    if (transactions[c].phase == Phase::Idle) return;
    ethercat_canfd_cancel_service(c);
    finish(c, result);
}
void reset_session(uint16_t result)
{
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        cancel(c, result);
        transactions[c] = {};
    }
}
void abort_internal(unsigned c)
{
    auto &t = transactions[c];
    if (t.phase != Phase::Idle && t.internal) {
        ethercat_canfd_cancel_service(c);
        uint32_t hold = t.hold_until;
        t = {};
        t.hold_until = hold;
    }
}
bool start_query(unsigned c, unsigned a, uint8_t code, uint8_t group, uint32_t now,
                 ParameterDialect dialect, uint16_t can_id, uint8_t format)
{
    if (c >= ENCOS_CHANNEL_COUNT || a >= ENCOS_AXIS_COUNT || a / ENCOS_MOTORS_PER_CHANNEL != c)
        return false;
    if (dialect != ParameterDialect::EncosLegacy) return false;
    auto &t = transactions[c];
    if (!available(c, now)) return false;
    encos_frame_t request{};
    request.id = can_id;
    if (!encos_query(code, group, &request)) return false;
    request.flags = format ? 24 : 0;
    uint32_t hold = t.hold_until;
    t = {};
    t.hold_until = hold;
    t.axis = a;
    t.query = code;
    t.group = group;
    t.internal = true;
    t.request = request;
    t.phase = Phase::Queued;
    t.started = now;
    return true;
}

bool commit(unsigned a, bool maintenance, uint32_t now, bool permitted, bool channel_is_active)
{
    if (a >= ENCOS_AXIS_COUNT) return false;
    if (dcu::app::motor_parameter_dialect(object<mc_routing_t>(axis_object_base(a))->ModelProfileId) !=
        dcu::app::ParameterDialect::EncosLegacy)
        return false;
    unsigned c = a / ENCOS_MOTORS_PER_CHANNEL;
    service_t *t = &transactions[c];
    if (!available(c, now) || channel_is_active)
        return false;
    if (maintenance) {
        mc_maintenance_t *m = object<mc_maintenance_t>(axis_object_base(a) + 8);
        if (m->RequestSeq == m->DoneSeq)
            return true;
        if (!permitted) {
            m->Result = RESULT_STATE;
            m->DoneSeq = m->RequestSeq;
            return true;
        }
        if (m->Opcode != 3 || m->Argument0 || m->Argument1) {
            m->Result = RESULT_UNSUPPORTED;
            m->DoneSeq = m->RequestSeq;
            return true;
        }
        mc_routing_t *r = object<mc_routing_t>(axis_object_base(a));
        if (r->CanId < 1 || r->CanId > 8) {
            m->Result = RESULT_INVALID;
            m->DoneSeq = m->RequestSeq;
            return true;
        }
        uint32_t hold = t->hold_until;
        memset(t, 0, sizeof(*t));
        t->hold_until = hold;
        t->axis = a;
        t->seq = m->RequestSeq;
        t->purpose = Purpose::Maintenance;
        t->started = now;
        t->request.id = 0x7FF;
        t->request.dlc = 4;
        t->request.flags = object<mc_channel_config_t>(0x2010 + c)->FrameFormat ? 24 : 0;
        t->request.data[0] = r->CanId >> 8;
        t->request.data[1] = r->CanId;
        t->request.data[2] = 0;
        t->request.data[3] = 3;
        t->phase = Phase::Queued;
        m->Result = RESULT_BUSY;
        return true;
    }
    mc_transaction_t *tr = object<mc_transaction_t>(axis_object_base(a) + 7);
    uint16_t pid = tr->ParameterId;
    uint8_t query = encos_parameter_query(pid), op = tr->Operation;
    if (tr->RequestSeq == tr->DoneSeq)
        return true;
    if (!permitted) {
        tr->Result = RESULT_STATE;
        tr->DoneSeq = tr->RequestSeq;
        return true;
    }
    if (op < 1 || op > 3 || (!query && !(op == 2 && pid == 2))) {
        tr->Result = RESULT_UNSUPPORTED;
        tr->DoneSeq = tr->RequestSeq;
        return true;
    }
    mc_routing_t *r = object<mc_routing_t>(axis_object_base(a));
    if (r->CanId < 1 || r->CanId > 8) {
        tr->Result = RESULT_INVALID;
        tr->DoneSeq = tr->RequestSeq;
        return true;
    }
    uint32_t hold = t->hold_until;
    memset(t, 0, sizeof(*t));
    t->hold_until = hold;
    t->axis = a;
    t->seq = tr->RequestSeq;
    t->parameter = pid;
    t->query = query;
    t->group = object<mc_kt_t>(axis_object_base(a) + 4)->Group;
    t->started = now;
    t->request.id = r->CanId;
    bool ok;
    if (op == 2) {
        ok = encos_parameter(pid, object<mc_basic_t>(axis_object_base(a) + 1), object<mc_range_t>(axis_object_base(a) + 2), object<mc_gain_t>(axis_object_base(a) + 3), object<mc_kt_t>(axis_object_base(a) + 4), &t->request);
    } else
        ok = encos_query(query, t->group, &t->request);
    if (!ok || t->request.dlc < 2 || t->request.dlc > sizeof(t->request.data)) {
        tr->Result = RESULT_UNSUPPORTED;
        tr->DoneSeq = tr->RequestSeq;
        return true;
    }
    if (op == 2) {
        t->purpose = Purpose::Write;
        t->expected_size = t->request.dlc - 2;
        memcpy(t->expected, t->request.data + 2, t->expected_size);
    }
    t->request.flags = object<mc_channel_config_t>(0x2010 + c)->FrameFormat ? 24 : 0;
    t->phase = Phase::Queued;
    tr->Result = RESULT_BUSY;
    return true;
}

bool consume_maintenance(unsigned c, const dcu_can_frame_t &f)
{
    auto &t = transactions[c];
    if (t.phase != Phase::AwaitingReply || t.purpose != Purpose::Maintenance || f.id != 0x7FF || f.dlc < 4)
        return false;
    if (f.data[0] == t.request.data[0] && f.data[1] == t.request.data[1] && f.data[2] == 1) {
        memcpy(object<mc_maintenance_t>(axis_object_base(t.axis) + 8)->RawResponse, f.data, 8);
        if (f.data[3] == 3) finish(c, RESULT_OK);
        else if (f.data[3] == 0) finish(c, RESULT_IO);
    }
    return true;
}

Observation consume(unsigned c, unsigned a, const dcu_can_frame_t &f, uint32_t now)
{
    Observation observation{};
    auto &t = transactions[c];
    uint8_t type = f.data[0] >> 5;
    if ((type == 2 || type == 3) && f.dlc == 8 && (f.data[0] & 31) <= 7) {
        int32_t value;
        if (encos_float(f.data + 1, type == 2 ? 1e6 * 3.141592653589793 / 180 : 1e6 * 2 * 3.141592653589793 / 60, &value)) {
            observation.valid = true;
            observation.field = type == 2 ? Field::Position : Field::Velocity;
            observation.value = value;
        }
        return observation;
    }
    if (t.phase != Phase::AwaitingReply || a != t.axis) return observation;
    auto *tr = object<mc_transaction_t>(axis_object_base(a) + 7);
    tr->RawResponseLength = f.dlc;
    memcpy(tr->RawResponse, f.data, 8);
    if (t.purpose == Purpose::Maintenance)
        memcpy(object<mc_maintenance_t>(axis_object_base(a) + 8)->RawResponse, f.data, 8);
    if (t.purpose == Purpose::Write &&
        ((type == 4 && f.dlc == 3 && f.data[1] == t.request.data[1]) ||
         (f.data[0] == 0xFF && f.data[1] == 0xFE && f.dlc >= 4 && f.data[2] == t.request.data[1]))) {
        observation.acknowledged = true;
        if (type == 4 && (f.data[2] != 1 || (f.data[0] & 31))) {
            finish(c, RESULT_IO);
            return observation;
        }
        if (!t.query) {
            finish(c, RESULT_OK);
            return observation;
        }
        t.purpose = Purpose::Verify;
        t.phase = Phase::Queued;
        t.started = now;
        encos_query(t.query, t.group, &t.request);
        return observation;
    }
    if (type == 5 && (f.data[0] & 31) <= 7 && f.data[1] == t.query) {
        if (f.data[0] & 31) {
            finish(c, RESULT_IO);
            observation.valid = true;
        } else if (cache_query(a, t.query, &f, &t, observation)) {
            observation.valid = true;
            if (t.purpose == Purpose::Verify &&
                (f.dlc != t.expected_size + 2 || memcmp(f.data + 2, t.expected, t.expected_size)))
                finish(c, RESULT_MISMATCH);
            else
                finish(c, RESULT_OK);
        }
    }
    return observation;
}

}
