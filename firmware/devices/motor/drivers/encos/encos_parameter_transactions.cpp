#include "encos_parameter_transactions.hpp"
#include "can_transport.h"
#include "detail/encos_parameter_state.hpp"
#include "motor_result.hpp"
#include "cfg/encos_runtime.hpp"
#include "encos_service.h"
#include "device_config.h"
#include <climits>
#include <cstring>

namespace dcu::motor::encos_parameters {
namespace motor_cfg = dcu::motor::encos_runtime;
namespace {
using detail::Phase;
using detail::Purpose;
using detail::service_t;
service_t transactions[DCU_DEVICE_CAN_BUS_COUNT];
Observer observer;

static void reset_transaction(service_t &transaction)
{
    const uint32_t hold_started = transaction.hold_started;
    const bool hold_active = transaction.hold_active;
    transaction = {};
    transaction.hold_started = hold_started;
    transaction.hold_active = hold_active;
}

static void finish(unsigned c, uint16_t result)
{
    service_t *t = &transactions[c];
    unsigned a = t->device;
    if (observer.complete)
        observer.complete(a, t->purpose == Purpose::Maintenance, t->internal, t->seq, result);
    t->phase = Phase::Idle;
    t->tx_id = 0;
}

static void verify(service_t &transaction, uint32_t now)
{
    transaction.purpose = Purpose::Verify;
    transaction.phase = Phase::Queued;
    transaction.started = now;
    encos_query(transaction.query, transaction.group, &transaction.request);
}

static void complete_query(unsigned c, const dcu_can_frame_t &frame)
{
    const auto &transaction = transactions[c];
    const bool mismatch = transaction.purpose == Purpose::Verify &&
        (frame.dlc != transaction.expected_size + 2 ||
         memcmp(frame.data + 2, transaction.expected, transaction.expected_size));
    finish(c, mismatch ? RESULT_MISMATCH : RESULT_OK);
}

static bool decode_query(unsigned a, uint8_t code, const dcu_can_frame_t *f, service_t *t, Observation &observation)
{
    QueryValue decoded{};
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
        memcpy(&decoded.real, &bits, 4);
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
        ok = true;
    } else if ((code == 5 || code == 22 || code == 31 || code == 38) && f->dlc == 4) {
        decoded.pair[0] = encos_be16(d + 2);
        ok = true;
    } else if (((code >= 23 && code <= 28) || (code >= 32 && code <= 34)) && f->dlc == 6) {
        decoded.pair[0] = encos_be16(d + 2);
        decoded.pair[1] = encos_be16(d + 4);
        ok = true;
    } else if (code == 29 && f->dlc == 7 && d[2] >= 1 && d[2] <= 3) {
        t->uuid[d[2] - 1] = encos_be32(d + 3);
        t->uuid_mask |= 1u << (d[2] - 1);
        if (t->uuid_mask != 7) {
            return false;
        }
        for (unsigned i = 0; i < 3; ++i) decoded.uuid[i] = t->uuid[i];
        ok = true;
    } else if (code == 30 && f->dlc == 8) {
        for (unsigned i = 0; i < 6; ++i) decoded.version[i] = d[i + 2];
        ok = true;
    } else if ((code == 35 || code == 37) && f->dlc == 3) {
        if (code == 35 && d[2] <= 2) {
            decoded.choice = d[2];
            ok = true;
        }
        if (code == 37 && d[2] <= 1) {
            decoded.choice = d[2];
            ok = true;
        }
    } else if (code == 39 && f->dlc == 7 && d[2] <= 1) {
        decoded.encoder.calibration_enabled = d[2] != 0;
        const uint16_t raw = encos_be16(d + 3), calibrated = encos_be16(d + 5);
        decoded.encoder.raw_centidegrees = raw < 0x8000 ? raw : int32_t(raw) - 0x10000;
        decoded.encoder.calibrated_centidegrees = calibrated < 0x8000 ? calibrated : int32_t(calibrated) - 0x10000;
        ok = true;
    }
    if (ok && observer.query) observer.query(a, code, decoded);
    return ok;
}
}

void initialize(const Observer &sink)
{
    observer = sink;
    for (auto &transaction : transactions) transaction = {};
}

Status inspect(unsigned c)
{
    const auto &t = transactions[c];
    const bool awaiting_tx = t.phase == Phase::AwaitingTransmit;
    return {t.phase != Phase::Idle, awaiting_tx || t.phase == Phase::AwaitingReply, t.internal,
            t.device, t.request.id, t.dialect, t.frame_format, awaiting_tx, t.started};
}
bool available(unsigned c, uint32_t now)
{
    const auto &t = transactions[c];
    return t.phase == Phase::Idle && (!t.hold_active || now - t.hold_started >= motor_cfg::parameter_hold_ms);
}
const dcu_can_frame_t *pending(unsigned c)
{
    return transactions[c].phase == Phase::Queued ? &transactions[c].request : nullptr;
}
void mark_submitted(unsigned c, uint32_t now, dcu_can_tx_id_t id)
{
    auto &t = transactions[c];
    if (t.phase != Phase::Queued || !id) return;
    t.phase = Phase::AwaitingTransmit;
    t.tx_id = id;
    t.started = now;
    if (t.purpose == Purpose::Maintenance || t.parameter == 2) {
        t.hold_started = now;
        t.hold_active = true;
    }
}
void finish_tx(unsigned c, const dcu_can_tx_event_t &event)
{
    auto &t = transactions[c];
    if (t.phase != Phase::AwaitingTransmit || t.tx_id != event.id) return;
    t.tx_id = 0;
    if (event.status == DCU_CAN_TX_COMPLETED)
        t.phase = Phase::AwaitingReply;
    else
        finish(c, RESULT_IO);
}

void check_timeout(unsigned c, uint32_t now)
{
    auto &t = transactions[c];
    if (t.hold_active && now - t.hold_started >= motor_cfg::parameter_hold_ms)
        t.hold_active = false;
    if (t.phase != Phase::Idle && now - t.started >= motor_cfg::parameter_timeout_ms)
        cancel(c, RESULT_TIMEOUT);
}
void cancel(unsigned c, uint16_t result)
{
    if (transactions[c].phase == Phase::Idle) return;
    if (transactions[c].tx_id) dcu_can_cancel_tx(c, transactions[c].tx_id);
    finish(c, result);
}
void reset_session(uint16_t result)
{
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        cancel(c, result);
        transactions[c] = {};
    }
}
void abort_internal(unsigned c)
{
    auto &t = transactions[c];
    if (t.phase != Phase::Idle && t.internal) {
        if (t.tx_id) dcu_can_cancel_tx(c, t.tx_id);
        reset_transaction(t);
    }
}
bool start_query(unsigned c, unsigned a, uint8_t code, uint8_t group, uint32_t now,
                 ParameterDialect dialect, uint16_t can_id, uint8_t format)
{
    if (c >= DCU_DEVICE_CAN_BUS_COUNT || a >= DCU_MOTOR_DEVICE_CAPACITY)
        return false;
    if (dialect != ParameterDialect::EncosLegacy || can_id < 1 || can_id > 8 || format > 1) return false;
    auto &t = transactions[c];
    if (!available(c, now)) return false;
    dcu_can_frame_t request{};
    request.id = can_id;
    if (!encos_query(code, group, &request)) return false;
    request.flags = format ? 24 : 0;
    reset_transaction(t);
    t.device = a;
    t.dialect = dialect;
    t.frame_format = format;
    t.query = code;
    t.group = group;
    t.internal = true;
    t.request = request;
    t.phase = Phase::Queued;
    t.started = now;
    return true;
}

CommitOutcome commit(const Request &request, uint32_t now, bool permitted, bool channel_is_active)
{
    const unsigned a = request.device;
    if (a >= DCU_MOTOR_DEVICE_CAPACITY || request.dialect != ParameterDialect::EncosLegacy) return {};
    const unsigned c = request.bus;
    if (c >= DCU_DEVICE_CAN_BUS_COUNT) return {};
    auto &t = transactions[c];
    if (!available(c, now) || channel_is_active) return {};
    if (request.sequence == request.done_sequence) return {true, false, RESULT_OK, request.done_sequence};
    if (!permitted) return {true, true, RESULT_STATE, request.sequence};
    const uint16_t pid = request.parameter;
    const uint8_t query = encos_parameter_query(pid);
    const uint16_t op = request.opcode;
    if (request.maintenance ? (op != 3 || request.argument0 || request.argument1) :
        (op < 1 || op > 3 || (!query && !(op == 2 && pid == 2))))
        return {true, true, RESULT_UNSUPPORTED, request.sequence};
    if (request.can_id < 1 || request.can_id > 8 || request.frame_format > 1)
        return {true, true, RESULT_INVALID, request.sequence};

    dcu_can_frame_t verification{};
    if (!request.maintenance && query && !encos_query(query, request.group, &verification))
        return {true, true, RESULT_UNSUPPORTED, request.sequence};

    dcu_can_frame_t frame{};
    if (request.maintenance) {
        frame.id = 0x7FF;
        frame.dlc = 4;
        frame.data[0] = request.can_id >> 8;
        frame.data[1] = request.can_id;
        frame.data[3] = 3;
    } else {
        bool ok;
        if (op == 2) {
            frame = request.write;
            ok = request.write_valid;
        } else {
            ok = encos_query(query, request.group, &frame);
        }
        if (!ok || frame.dlc < 2 || frame.dlc > DCU_CAN_CLASSIC_MAX_PAYLOAD)
            return {true, true, RESULT_UNSUPPORTED, request.sequence};
        frame.id = request.can_id;
    }
    frame.flags = request.frame_format ? DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS : 0;
    reset_transaction(t);
    t.device = a;
    t.seq = request.sequence;
    t.dialect = request.dialect;
    t.frame_format = request.frame_format;
    t.parameter = request.maintenance ? 0 : pid;
    t.query = query;
    t.group = request.group;
    t.started = now;
    t.request = frame;
    if (request.maintenance) {
        t.purpose = Purpose::Maintenance;
    } else if (op == 2) {
        t.purpose = Purpose::Write;
        t.expected_size = frame.dlc - 2;
        memcpy(t.expected, frame.data + 2, t.expected_size);
    }
    t.phase = Phase::Queued;
    return {true, true, RESULT_BUSY, request.done_sequence};
}

bool consume_maintenance(unsigned c, const dcu_can_frame_t &f)
{
    auto &t = transactions[c];
    if (t.phase != Phase::AwaitingReply || t.purpose != Purpose::Maintenance || f.id != 0x7FF || f.dlc < 4)
        return false;
    if (f.data[0] == t.request.data[0] && f.data[1] == t.request.data[1] && f.data[2] == 1) {
        if (observer.response) observer.response(t.device, true, f);
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
    if (t.phase != Phase::AwaitingReply || a != t.device || f.id != t.request.id) return observation;
    if (observer.response) observer.response(a, t.purpose == Purpose::Maintenance, f);
    if (t.query == 36 && f.dlc == 8 && f.data[0] == 0xEE && (f.data[1] & 15U) == t.group) {
        // V1.19EAP section 9.2.15 disagrees about the mode bits. Group 1..10 and
        // the four coefficients are unambiguous; do not derive a mode from this frame.
        if (t.purpose == Purpose::Write) {
            observation.acknowledged = true;
            verify(t, now);
        } else {
            QueryValue decoded{};
            decoded.kt.group = t.group;
            for (unsigned i = 0; i < 2; ++i) {
                const auto *data = f.data + 2 + 3 * i;
                decoded.kt.words[2 * i] = uint16_t(data[0]) << 4 | data[1] >> 4;
                decoded.kt.words[2 * i + 1] = uint16_t(data[1] & 15U) << 8 | data[2];
            }
            if (observer.query) observer.query(a, 36, decoded);
            observation.valid = true;
            complete_query(c, f);
        }
        return observation;
    }
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
        verify(t, now);
        return observation;
    }
    if (type == 5 && (f.data[0] & 31) <= 7 && f.data[1] == t.query) {
        if (f.data[0] & 31) {
            finish(c, RESULT_IO);
            observation.valid = true;
        } else if (decode_query(a, t.query, &f, &t, observation)) {
            observation.valid = true;
            complete_query(c, f);
        }
    }
    return observation;
}

}
