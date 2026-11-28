#ifndef DCU_MOTOR_AXIS_CODEC_HPP
#define DCU_MOTOR_AXIS_CODEC_HPP

#include "motor_feedback.hpp"
#include "encos_interface.h"

namespace dcu::ethercat::motor_axis_codec {

static_assert(motor::Arm == MC_AXIS_ARM && motor::Stop == MC_AXIS_STOP &&
              motor::ResetFault == MC_AXIS_RESET && motor::StartMotion == MC_AXIS_START_MOTION);
static_assert(motor::QuerySource == MC_STATUS_QUERY_SOURCE && motor::ArmedQueryOnly == MC_STATUS_ARMED_QUERY_ONLY);

inline motor::Command command(const mc_axis_rx_t &wire)
{
    return {static_cast<motor::ControlMode>(wire.Mode), wire.ControlWord, wire.Flags,
        wire.TargetPosition, wire.TargetVelocity, wire.TargetTorque, wire.TargetCurrent,
        wire.CurrentLimit, wire.Kp, wire.Kd};
}

inline void sample(const motor::Feedback &value, mc_axis_tx_t &wire)
{
    wire.ActualPosition = value.position_urad;
    wire.ActualVelocity = value.velocity_urad_s;
    wire.EstimatedTorque = value.torque_mNm;
    wire.ActualCurrent = static_cast<int16_t>(value.current_cA);
    wire.MotorTemperature = value.motor_temperature_deciC;
    wire.MosTemperature = value.mos_temperature_deciC;
    wire.MotorFaultRaw = value.motor_fault > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(value.motor_fault);
    wire.ValidFlags &= 64U;
    if (value.valid_fields & motor::Position) wire.ValidFlags |= 1U;
    if (value.valid_fields & motor::Velocity) wire.ValidFlags |= 2U;
    if (value.valid_fields & motor::Current) wire.ValidFlags |= 4U;
    if (value.valid_fields & motor::Torque) wire.ValidFlags |= 8U;
    if (value.valid_fields & motor::MotorTemperature) wire.ValidFlags |= 16U;
    if (value.valid_fields & motor::MosTemperature) wire.ValidFlags |= 32U;
}

inline mc_axis_tx_t feedback(const motor::MotorFeedback &value)
{
    mc_axis_tx_t result{};
    sample(value.sample, result);
    if (value.power_valid) result.ValidFlags |= 64U;
    result.StatusWord = value.status_word;
    result.ModeDisplay = static_cast<uint8_t>(value.mode);
    result.BridgeFaultCode = static_cast<uint8_t>(value.bridge_fault);
    result.ActualPower = value.power_mW;
    result.FeedbackAgeMs = value.feedback_age_ms;
    result.PowerAgeMs = value.power_age_ms;
    return result;
}

inline void diagnostics(const motor::MotorDiagnostics &value, mc_diag_t &wire)
{
    wire.TxEnqueueCount = value.tx_enqueue_count;
    wire.TxCompleteCount = value.tx_complete_count;
    wire.TxDropCount = value.tx_drop_count;
    wire.RxValidCount = value.rx_valid_count;
    wire.ProtocolErrorCount = value.protocol_error_count;
    wire.ActiveFaultBits = value.active_fault_bits;
    wire.QueriedPosition = value.queried_position_urad;
    wire.QueriedVelocity = value.queried_velocity_urad_s;
    wire.PositionQueryAgeUs = value.position_query_age_us;
    wire.VelocityQueryAgeUs = value.velocity_query_age_us;
    // LastServiceError is owned by the EtherCAT service adapter.
}


}
#endif
