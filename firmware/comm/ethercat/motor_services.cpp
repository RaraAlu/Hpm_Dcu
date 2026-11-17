#include "motor_services.hpp"
#include "motor_axis_codec.hpp"
#include "motor_dictionary.hpp"
#include "motor_application.hpp"
#include "motor_driver_binding.hpp"
#include "encos_parameter_transactions.hpp"
#include "encos_motor.h"
#include <cstring>

namespace dcu::ethercat::motor_services {
namespace parameters = app::parameters;
using namespace motor_dictionary;

namespace {
void response(unsigned axis, bool maintenance, const dcu_can_frame_t &frame)
{
    if (maintenance) {
        auto *target = object<mc_maintenance_t>(axis_object_base(axis) + 8);
        std::memcpy(target->RawResponse, frame.data, sizeof(target->RawResponse));
    } else {
        auto *target = object<mc_transaction_t>(axis_object_base(axis) + 7);
        target->RawResponseLength = frame.dlc;
        std::memcpy(target->RawResponse, frame.data, sizeof(target->RawResponse));
    }
}

void complete(unsigned axis, bool maintenance, bool internal, uint32_t sequence, uint16_t result)
{
    if (!internal) {
        if (maintenance) {
            auto *target = object<mc_maintenance_t>(axis_object_base(axis) + 8);
            target->DoneSeq = sequence;
            target->Result = result;
        } else {
            auto *target = object<mc_transaction_t>(axis_object_base(axis) + 7);
            target->DoneSeq = sequence;
            target->Result = result;
        }
    }
    axis_diagnostics(axis)->LastServiceError = result;
}

void query(unsigned axis, uint8_t code, const parameters::QueryValue &value)
{
    const auto base = axis_object_base(axis);
    auto *snapshot = object<mc_query_t>(base + 5);
    auto *basic = object<mc_basic_t>(base + 1);
    auto *range = object<mc_range_t>(base + 2);
    auto *gain = object<mc_gain_t>(base + 3);
    auto *identity = object<mc_identity_t>(base + 6);
    switch (code) {
    case 1: snapshot->PositionDeg = value.real; break;
    case 2: snapshot->SpeedRpm = value.real; break;
    case 3: snapshot->PhaseCurrentA = value.real; break;
    case 4: snapshot->PowerW = value.real; break;
    case 5:
        snapshot->AccelerationRaw = basic->AccelerationRaw = value.pair[0];
        snapshot->ValidBits |= 1U << 4;
        break;
    case 22: basic->TorqueConstantRaw = value.pair[0]; break;
    case 23: range->KpMin = value.pair[0]; range->KpMax = value.pair[1]; break;
    case 24: range->KdMin = value.pair[0]; range->KdMax = value.pair[1]; break;
    case 25: range->PosMinRaw = static_cast<int16_t>(value.pair[0]); range->PosMaxRaw = static_cast<int16_t>(value.pair[1]); break;
    case 26: range->SpdMinRaw = static_cast<int16_t>(value.pair[0]); range->SpdMaxRaw = static_cast<int16_t>(value.pair[1]); break;
    case 27: range->TorMinRaw = static_cast<int16_t>(value.pair[0]); range->TorMaxRaw = static_cast<int16_t>(value.pair[1]); break;
    case 28: range->CurMinRaw = static_cast<int16_t>(value.pair[0]); range->CurMaxRaw = static_cast<int16_t>(value.pair[1]); break;
    case 29:
        identity->UuidPart1 = value.uuid[0];
        identity->UuidPart2 = value.uuid[1];
        identity->UuidPart3 = value.uuid[2];
        break;
    case 30:
        identity->HardwareVersion0 = value.version[0];
        identity->HardwareVersion1 = value.version[1];
        identity->HardwareVersion2 = value.version[2];
        identity->SoftwareVersion0 = value.version[3];
        identity->SoftwareVersion1 = value.version[4];
        identity->SoftwareVersion2 = value.version[5];
        break;
    case 31: basic->CanTimeoutMs = value.pair[0]; break;
    case 32: gain->CurrentKpRaw = value.pair[0]; gain->CurrentKiRaw = value.pair[1]; break;
    case 33: gain->SpeedKpRaw = value.pair[0]; gain->SpeedKiRaw = value.pair[1]; break;
    case 34: gain->PositionKpRaw = value.pair[0]; gain->PositionKdRaw = value.pair[1]; break;
    case 35: object<mc_kt_t>(base + 4)->CalibrationMode = value.choice; break;
    case 36: {
        auto *kt = object<mc_kt_t>(base + 4);
        kt->Group = value.kt.group;
        kt->Kt1Raw = value.kt.words[0];
        kt->Kt2Raw = value.kt.words[1];
        kt->Kt3Raw = value.kt.words[2];
        kt->Kt4Raw = value.kt.words[3];
        break;
    }
    case 37:
        snapshot->BrakeState = value.choice;
        snapshot->ValidBits |= 1U << 5;
        break;
    case 38:
        snapshot->ZeroOffsetRaw = basic->ZeroOffsetRaw = static_cast<int16_t>(value.pair[0]);
        snapshot->ValidBits |= 1U << 9;
        break;
    case 39:
        snapshot->EncoderOutputEnabled = value.encoder.calibration_enabled;
        snapshot->EncoderOriginalRaw = value.encoder.raw_centidegrees;
        snapshot->EncoderCalibratedRaw = value.encoder.calibrated_centidegrees;
        snapshot->ValidBits |= (1U << 6) | (1U << 7) | (1U << 8);
        break;
    default: return;
    }
    if (code >= 1 && code <= 4) snapshot->ValidBits |= 1U << (code - 1);
    ++snapshot->SnapshotSeq;
    snapshot->AgeMs = 0;
}
}

void initialize()
{
    parameters::initialize({response, query, complete});
}

bool commit(unsigned axis, bool maintenance, uint32_t now, bool preop)
{
    if (axis >= ENCOS_AXIS_COUNT) return false;
    const auto base = axis_object_base(axis);
    const auto &route = *object<mc_routing_t>(base);
    parameters::Request request{};
    request.axis = axis;
    request.dialect = app::motor_parameter_dialect(route.ModelProfileId);
    request.can_id = route.CanId;
    request.frame_format = object<mc_channel_config_t>(0x2010 + axis / ENCOS_MOTORS_PER_CHANNEL)->FrameFormat;
    request.maintenance = maintenance;
    if (maintenance) {
        const auto &source = *object<mc_maintenance_t>(base + 8);
        request.sequence = source.RequestSeq;
        request.done_sequence = source.DoneSeq;
        request.opcode = source.Opcode;
        request.argument0 = source.Argument0;
        request.argument1 = source.Argument1;
    } else {
        const auto &source = *object<mc_transaction_t>(base + 7);
        request.sequence = source.RequestSeq;
        request.done_sequence = source.DoneSeq;
        request.opcode = source.Operation;
        request.parameter = source.ParameterId;
        request.group = object<mc_kt_t>(base + 4)->Group;
        if (request.opcode == 2) {
            const auto values = motor_axis_codec::parameter(request.parameter, *object<mc_basic_t>(base + 1),
                *object<mc_range_t>(base + 2), *object<mc_gain_t>(base + 3), *object<mc_kt_t>(base + 4));
            request.write_valid = encos_parameter(request.parameter, &values, &request.write);
        }
    }
    const auto outcome = app::motor_application::commit_parameter(request, now, preop);
    if (outcome.changed) {
        if (maintenance) {
            auto *target = object<mc_maintenance_t>(base + 8);
            target->DoneSeq = outcome.done_sequence;
            target->Result = outcome.result;
        } else {
            auto *target = object<mc_transaction_t>(base + 7);
            target->DoneSeq = outcome.done_sequence;
            target->Result = outcome.result;
        }
    }
    return outcome.accepted;
}
}
