#ifndef DCU_MOTOR_AXIS_CODEC_HPP
#define DCU_MOTOR_AXIS_CODEC_HPP

#include "axis_contract.hpp"
#include "encos_interface.h"

namespace dcu::ethercat::motor_axis_codec {

static_assert(motor::Arm == MC_AXIS_ARM && motor::Stop == MC_AXIS_STOP &&
              motor::ResetFault == MC_AXIS_RESET && motor::StartMotion == MC_AXIS_START_MOTION);
static_assert(app::QuerySource == MC_STATUS_QUERY_SOURCE && app::ArmedQueryOnly == MC_STATUS_ARMED_QUERY_ONLY);

inline app::AxisConfiguration configuration(const mc_routing_t &r, const mc_basic_t &b, const mc_range_t &range)
{
    app::AxisConfiguration result{};
    result.route = {r.CanId, r.PositionLimitMin, r.PositionLimitMax, r.VelocityLimit, r.TorqueLimit, r.CurrentLimit};
    result.model_profile_id = r.ModelProfileId;
    result.power_query_period_ms = r.PowerQueryPeriodMs;
    result.stop_policy = r.StopPolicy;
    result.feedback_policy = r.FeedbackPolicy;
    result.encos = {{{double(range.KpMin), double(range.KpMax)}, {double(range.KdMin), double(range.KdMax)},
        {range.PosMinRaw / 100., range.PosMaxRaw / 100.}, {range.SpdMinRaw / 100., range.SpdMaxRaw / 100.},
        {range.TorMinRaw / 10., range.TorMaxRaw / 10.}, {range.CurMinRaw / 10., range.CurMaxRaw / 10.},
        b.TorqueConstantRaw / 100.}, b.CanTimeoutMs, b.CommunicationMode};
    return result;
}

// PDO and SDO carry the same command fields, but have distinct wire layouts.
template <typename Wire> motor::Command command(const Wire &wire)
{
    return {static_cast<motor::ControlMode>(wire.Mode), wire.ControlWord, wire.Flags,
        wire.TargetPosition, wire.TargetVelocity, wire.TargetTorque, wire.TargetCurrent,
        wire.CurrentLimit, wire.Kp, wire.Kd};
}

inline mc_axis_rx_t command(const motor::Command &value)
{
    return {value.control_bits, static_cast<uint8_t>(value.mode), value.reserved_flags,
        value.position_urad, value.velocity_urad_s, value.torque_mNm, static_cast<int16_t>(value.current_cA),
        value.kp_centi, value.kd_milli, value.current_limit_dA};
}

inline void sample(const motor::Feedback &value, mc_axis_tx_t &wire)
{
    wire.ActualPosition = value.position_urad;
    wire.ActualVelocity = value.velocity_urad_s;
    wire.EstimatedTorque = value.torque_mNm;
    wire.ActualCurrent = static_cast<int16_t>(value.current_cA);
    wire.MotorTemperature = value.motor_temperature_deciC;
    wire.MosTemperature = value.mos_temperature_deciC;
    wire.MotorFaultRaw = static_cast<uint8_t>(value.motor_fault);
    wire.ValidFlags &= 64U;
    if (value.valid_fields & motor::Position) wire.ValidFlags |= 1U;
    if (value.valid_fields & motor::Velocity) wire.ValidFlags |= 2U;
    if (value.valid_fields & motor::Current) wire.ValidFlags |= 4U;
    if (value.valid_fields & motor::Torque) wire.ValidFlags |= 8U;
    if (value.valid_fields & motor::MotorTemperature) wire.ValidFlags |= 16U;
    if (value.valid_fields & motor::MosTemperature) wire.ValidFlags |= 32U;
}

inline mc_axis_tx_t feedback(const app::AxisFeedback &value)
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

inline void diagnostics(const app::AxisDiagnostics &value, mc_diag_t &wire)
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

inline encos_parameter_values_t parameter(uint16_t id, const mc_basic_t &b, const mc_range_t &r,
                                         const mc_gain_t &g, const mc_kt_t &k)
{
    encos_parameter_values_t result{};
    result.group = k.Group;
    switch (id) {
    case 2: result.words[0] = b.CommunicationMode; break;
    case 3: result.words[0] = b.TorqueConstantRaw; break;
    case 4: result.words[0] = r.KpMin; result.words[1] = r.KpMax; break;
    case 5: result.words[0] = r.KdMin; result.words[1] = r.KdMax; break;
    case 6: result.words[0] = uint16_t(r.PosMinRaw); result.words[1] = uint16_t(r.PosMaxRaw); break;
    case 7: result.words[0] = uint16_t(r.SpdMinRaw); result.words[1] = uint16_t(r.SpdMaxRaw); break;
    case 8: result.words[0] = uint16_t(r.TorMinRaw); result.words[1] = uint16_t(r.TorMaxRaw); break;
    case 9: result.words[0] = uint16_t(r.CurMinRaw); result.words[1] = uint16_t(r.CurMaxRaw); break;
    case 10: result.words[0] = b.CanTimeoutMs; break;
    case 11: result.words[0] = g.CurrentKpRaw; result.words[1] = g.CurrentKiRaw; break;
    case 12: result.words[0] = g.SpeedKpRaw; result.words[1] = g.SpeedKiRaw; break;
    case 13: result.words[0] = g.PositionKpRaw; result.words[1] = g.PositionKdRaw; break;
    case 14: result.words[0] = k.CalibrationMode; break;
    case 15:
        result.words[0] = k.Kt1Raw; result.words[1] = k.Kt2Raw;
        result.words[2] = k.Kt3Raw; result.words[3] = k.Kt4Raw;
        break;
    }
    return result;
}

}
#endif
