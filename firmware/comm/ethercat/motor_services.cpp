#include "motor_services.hpp"
#include "motor_adapter.hpp"
#include "motor_dictionary.hpp"
#include "gateway_application.hpp"
#include "encos_parameter_transactions.hpp"
#include <cstring>

namespace dcu::ethercat::motor_services {
namespace parameters = motor::encos_parameters;
using namespace motor_dictionary;

namespace {
void response(unsigned device, bool maintenance, const dcu_can_frame_t &frame)
{
    if (maintenance) return;
    const auto axis = motor_adapter::slot_for_device(device);
    if (axis >= ENCOS_AXIS_COUNT) return;
    auto *target = object<mc_transaction_t>(axis_object_base(axis) + 7);
    target->RawResponseLength = frame.dlc;
    std::memcpy(target->RawResponse, frame.data, sizeof(target->RawResponse));
}

void complete(unsigned device, bool maintenance, bool internal, uint32_t sequence, uint16_t result)
{
    const auto axis = motor_adapter::slot_for_device(device);
    if (axis >= ENCOS_AXIS_COUNT) return;
    if (!maintenance && !internal) {
        auto *target = object<mc_transaction_t>(axis_object_base(axis) + 7);
        target->DoneSeq = sequence;
        target->Result = result;
    }
    axis_diagnostics(axis)->LastServiceError = result;
}

void query(unsigned device, uint8_t code, const parameters::QueryValue &value)
{
    const auto axis = motor_adapter::slot_for_device(device);
    if (axis >= ENCOS_AXIS_COUNT) return;
    const auto base = axis_object_base(axis);
    auto *snapshot = object<mc_query_t>(base + 5);
    auto *identity = object<mc_identity_t>(base + 6);
    switch (code) {
    case 1: snapshot->PositionDeg = value.real; break;
    case 2: snapshot->SpeedRpm = value.real; break;
    case 3: snapshot->PhaseCurrentA = value.real; break;
    case 4: snapshot->PowerW = value.real; break;
    case 5:
        snapshot->AccelerationRaw = value.pair[0];
        snapshot->ValidBits |= 1U << 4;
        break;
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
    case 37:
        snapshot->BrakeState = value.choice;
        snapshot->ValidBits |= 1U << 5;
        break;
    case 38:
        snapshot->ZeroOffsetRaw = static_cast<int16_t>(value.pair[0]);
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

bool commit(unsigned axis, uint32_t now, bool preop)
{
    const auto device = motor_adapter::device_for_slot(axis);
    if (device >= DCU_MOTOR_DEVICE_CAPACITY) return false;
    auto *source = object<mc_transaction_t>(axis_object_base(axis) + 7);
    if (source->Operation != 1 || source->ParameterId == 0x124) {
        source->Result = motor::RESULT_UNSUPPORTED;
        return false;
    }
    const app::gateway_application::ParameterRead request{source->RequestSeq, source->DoneSeq,
                                                          source->ParameterId};
    const auto outcome = app::gateway_application::read_parameter(device, request, now, preop);
    if (outcome.changed) {
        source->DoneSeq = outcome.done_sequence;
        source->Result = outcome.result;
    }
    return outcome.accepted;
}
}
