#include "axis_controller.hpp"
#include "encos_parameter_transactions.hpp"
#include "ethercat_canfd_gateway.h"
#include "motor_application.hpp"
#include "motor_dictionary.hpp"
#include <climits>
#include <cmath>
#include <cstring>

namespace dcu::app {
namespace motor_cfg = config::motor_control;
using namespace motor_application;
using namespace dcu::ethercat::motor_dictionary;

namespace {
uint32_t age(uint32_t now, uint32_t then, bool seen)
{
    uint32_t d = now - then;
    return !seen || d > UINT32_MAX / 1000 ? UINT32_MAX : d * 1000;
}
uint16_t age_ms(uint32_t now, uint32_t then, bool seen)
{
    uint32_t d = now - then;
    return !seen || d > UINT16_MAX ? UINT16_MAX : (uint16_t)d;
}
}

void AxisController::initialize(unsigned axis)
{
    axis_ = axis;
}
bool AxisController::configure(bool enabled, uint8_t frame_format)
{
    state_ = {};
    auto *x = &state_;
    const unsigned a = axis_;
    std::memset(axis_feedback(a), 0, sizeof(mc_axis_tx_t));
    x->route = *object<mc_routing_t>(axis_object_base(a));
    x->basic = *object<mc_basic_t>(axis_object_base(a) + 1);
    x->range = *object<mc_range_t>(axis_object_base(a) + 2);
    if (!enabled) return true;
    const auto binding = configure_motor_driver(a, x->route, x->basic, x->range, frame_format);
    if (!binding.driver) return false;
    x->device.bind(*binding.driver);
    x->parameter_dialect = binding.parameter_dialect;
    x->configured = true;
    return true;
}
void AxisController::disable()
{
    stop(0);
    state_.configured = false;
    state_.device.reset();
}
void AxisController::stop(uint16_t fault)
{
    const unsigned a = axis_;
    auto *x = &state_;
    mc_axis_tx_t *s = axis_feedback(a);
    if (x->active)
        ethercat_canfd_cancel_axis(a);
    x->active = false;
    x->armed = false;
    x->query_armed = false;
    x->control_sent = false;
    x->feedback_seen = false;
    if (fault && !s->BridgeFaultCode)
        s->BridgeFaultCode = fault;
    if (fault)
        s->StatusWord |= (1u << 5) | (1u << 9);
    if (fault < 32 && fault)
        axis_diagnostics(a)->ActiveFaultBits |= 1u << fault;
}
void AxisController::reset_session()
{
    std::memset(&state_.command, 0, sizeof(state_.command));
    state_.previous_control = 0;
}
bool AxisController::query_ready(uint32_t now) const
{
    const unsigned a = axis_;
    const auto *x = &state_;
    return x->configured && x->online_seen && now - x->online_ms <= motor_cfg::query_online_ms && x->position_seen && now - x->position_ms <= motor_cfg::query_position_ms && x->velocity_seen && now - x->velocity_ms <= motor_cfg::query_velocity_ms && x->current_seen && now - x->current_ms <= motor_cfg::query_current_ms && fabs((double)axis_feedback(a)->ActualCurrent) <= x->route.CurrentLimit * 10.0 && !axis_feedback(a)->MotorFaultRaw && !axis_feedback(a)->BridgeFaultCode && ethercat_canfd_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3;
}
bool AxisController::accept(const mc_axis_rx_t &intent, bool permit, uint32_t now)
{
    const unsigned a = axis_;
    const auto *command = &intent;
    auto *x = &state_;
    mc_axis_tx_t *s = axis_feedback(a);
    uint16_t prev = x->previous_control;
    x->previous_control = command->ControlWord;
    if (!permit || (command->ControlWord & MC_AXIS_STOP) || !(command->ControlWord & MC_AXIS_ARM)) {
        stop(0);
        if (permit && !(command->ControlWord & MC_AXIS_ARM) && !(command->ControlWord & MC_AXIS_STOP)) {
            x->armed = true;
            if ((command->ControlWord & MC_AXIS_RESET) && !(prev & MC_AXIS_RESET) && ethercat_canfd_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3 && !s->MotorFaultRaw) {
                s->BridgeFaultCode = 0;
                s->StatusWord &= ~((1u << 5) | (1u << 8) | (1u << 9));
                axis_diagnostics(a)->ActiveFaultBits = 0;
            }
        }
        memset(&x->command, 0, sizeof(x->command));
        return true;
    }
    if ((command->ControlWord & ~15u) || command->Flags) {
        stop(2);
        return false;
    }
    if (!x->query_armed && !x->active) {
        if (!x->configured || !x->armed || (prev & MC_AXIS_ARM) || (command->ControlWord & MC_AXIS_START_MOTION))
            return false;
        x->query_armed = true;
        x->armed = false;
    }
    if (!(command->ControlWord & MC_AXIS_START_MOTION)) {
        if (x->active) {
            stop(0);
            x->query_armed = true;
        }
        memset(&x->command, 0, sizeof(x->command));
        return true;
    }
    dcu_can_frame_t f;
    if (!x->configured || !dcu::app::encode_motor_command(x->device, *command, f)) {
        stop(x->configured ? 2 : 1);
        s->StatusWord |= 1u << 8;
        return false;
    }
    if (x->active && x->command.Mode != command->Mode) {
        stop(8);
        s->StatusWord |= 1u << 8;
        return false;
    }
    if (!x->active) {
        /* StartMotion needs a fresh edge after a separate query-only cycle. */
        if ((prev & MC_AXIS_START_MOTION) || !query_ready(now) ||
            fabs((double)axis_diagnostics(a)->QueriedVelocity) > 20000.0 ||
            ((command->Mode == 1 || command->Mode == 4) &&
             fabs((double)command->TargetPosition - axis_diagnostics(a)->QueriedPosition) > 100000.0))
            return false;
        unsigned bus = a / ENCOS_MOTORS_PER_CHANNEL;
        auto transaction = parameters::inspect(bus);
        if (transaction.busy && transaction.internal) {
            ethercat_canfd_cancel_service(bus);
            parameters::abort_internal(bus);
        }
        x->startup_command = *command;
        x->startup_command.ControlWord = MC_AXIS_ARM;
        x->startup_command.Mode = 1;
        x->startup_command.TargetPosition = axis_diagnostics(a)->QueriedPosition;
        x->startup_command.TargetVelocity = 0;
        x->startup_command.TargetTorque = 0;
        x->startup_command.TargetCurrent = 0;
        x->startup_command.Kp = 0;
        x->startup_command.Kd = 0;
        x->startup_command.CurrentLimit = x->route.CurrentLimit < 4095 ? x->route.CurrentLimit : 4095;
        if (!dcu::app::encode_motor_command(x->device, x->startup_command, f))
            return false;
        x->active_ms = now;
        x->control_sent = false;
        x->feedback_seen = false;
        x->new_feedback = false;
        s->ValidFlags &= 64u;
    }
    x->active = true;
    x->command = *command;
    s->ModeDisplay = x->feedback_seen ? command->Mode : 1;
    s->StatusWord &= ~(1u << 8);
    memcpy(mc_object(0x7200 + a), command, sizeof(*command));
    return true;
}
void AxisController::check_timeouts(uint32_t now, bool sdo_owner, uint16_t feedback_timeout_ms)
{
    auto *x = &state_;
    if (x->active || x->query_armed) {
        if (sdo_owner && uint32_t(now - x->commit_ms) >= x->lease_ms)
            stop(3);
        // First feedback is timed only after the first successful CAN submission.
        if (x->control_sent && uint32_t(now - (x->feedback_seen ? x->feedback_ms : x->active_ms)) >= feedback_timeout_ms)
            stop(4);
    }
}
void AxisController::check_startup(uint32_t now)
{
    if (state_.active && !state_.control_sent && !query_ready(now))
        stop(8);
}
bool AxisController::encode_control(dcu_can_frame_t &frame) const
{
    return encode_motor_command(state_.device, state_.feedback_seen ? state_.command : state_.startup_command, frame);
}
void AxisController::control_submitted(uint32_t now)
{
    if (!state_.control_sent) {
        state_.control_sent = true;
        state_.active_ms = now;
    }
    axis_feedback(axis_)->ModeDisplay = state_.feedback_seen ? state_.command.Mode : 1;
    ++axis_diagnostics(axis_)->TxEnqueueCount;
}
void AxisController::set_sdo_lease(uint16_t lease_ms, uint32_t now)
{
    state_.lease_ms = lease_ms;
    state_.commit_ms = now;
}
bool AxisController::start_query(uint32_t now, uint8_t owner, uint8_t frame_format)
{
    auto *x = &state_;
    if (!x->configured || (!x->active && owner == OWNER_SDO && !x->query_armed) ||
        (x->active && (owner != OWNER_PDO || !x->route.PowerQueryPeriodMs)))
        return false;
    unsigned first = x->active ? 3u : 0u;
    for (unsigned k = first; k < motor_cfg::query_period_ms.size(); ++k) {
        unsigned period_ms = k == 3 ? x->route.PowerQueryPeriodMs : motor_cfg::query_period_ms[k];
        if (!period_ms || ((x->query_requested_mask & (1u << k)) && now - x->query_request_ms[k] < period_ms))
            continue;
        if (!parameters::start_query(axis_ / ENCOS_MOTORS_PER_CHANNEL, axis_, k + 1, 1, now,
                                     x->parameter_dialect, x->route.CanId, frame_format))
            return false;
        x->query_request_ms[k] = now;
        x->query_requested_mask |= 1u << k;
        return true;
    }
    return false;
}
void AxisController::consume_feedback(const dcu_can_frame_t &frame, uint32_t stamp, uint32_t now)
{
    const auto *f = &frame;
    const unsigned a = axis_, c = axis_ / ENCOS_MOTORS_PER_CHANNEL;
    const auto transaction = parameters::inspect(c);
    auto *x = &state_;
    uint8_t type = f->data[0] >> 5;
    bool valid = false;
    bool cyclic_feedback = false;
    if (x->active && x->control_sent && x->configured &&
        dcu::app::decode_motor_feedback(x->device, *f, *axis_feedback(a))) {
        cyclic_feedback = true;
        if (!x->feedback_seen) {
            x->position_seen = x->velocity_seen = x->current_seen = false;
        }
        x->feedback_seen = true;
        x->feedback_ms = stamp;
        x->new_feedback = true;
        x->query_armed = false;
        valid = true;
        ++x->type1_count;
        if (fabs((double)axis_feedback(a)->ActualCurrent) > x->route.CurrentLimit * 10.0)
            stop(2);
    }
    else {
        // PREOP parameter transactions may precede Apply configuration.
        auto dialect = transaction.busy && transaction.sent && a == transaction.axis ?
            dcu::app::motor_parameter_dialect(object<mc_routing_t>(axis_object_base(a))->ModelProfileId) :
            x->parameter_dialect;
        if (dialect != dcu::app::ParameterDialect::EncosLegacy) return;
        auto observation = parameters::consume(c, a, *f, now);
        if (observation.acknowledged) return;
        valid = observation.valid;
        if (valid) {
            switch (observation.field) {
            case parameters::Field::Position:
                axis_diagnostics(a)->QueriedPosition = observation.value;
                x->position_ms = stamp;
                x->position_seen = true;
                if (observation.update_actual && !x->active) axis_feedback(a)->ActualPosition = observation.value;
                break;
            case parameters::Field::Velocity:
                axis_diagnostics(a)->QueriedVelocity = observation.value;
                x->velocity_ms = stamp;
                x->velocity_seen = true;
                if (observation.update_actual && !x->active) axis_feedback(a)->ActualVelocity = observation.value;
                break;
            case parameters::Field::Current:
                x->current_ms = stamp;
                x->current_seen = true;
                if (!x->active) axis_feedback(a)->ActualCurrent = static_cast<int16_t>(observation.value);
                break;
            case parameters::Field::Power:
                axis_feedback(a)->ActualPower = observation.value;
                x->power_ms = stamp;
                x->power_seen = x->new_power = true;
                break;
            case parameters::Field::None:
                break;
            }
        }
    }
    if (valid) {
        x->online_seen = true;
        x->online_ms = stamp;
        axis_diagnostics(a)->RxValidCount++;
        if (cyclic_feedback || (type >= 1 && type <= 5)) {
            if (!cyclic_feedback)
                axis_feedback(a)->MotorFaultRaw = f->data[0] & 31;
            if (axis_feedback(a)->MotorFaultRaw)
                stop(0);
        }
    } else
        axis_diagnostics(a)->ProtocolErrorCount++;
}
AxisPublication AxisController::publish(uint32_t now, uint16_t feedback_timeout_ms)
{
    const unsigned a = axis_, c = axis_ / ENCOS_MOTORS_PER_CHANNEL;
    auto *x = &state_;
    mc_axis_tx_t *s = axis_feedback(a);
    bool type1 = x->active && x->feedback_seen;
    bool online = x->online_seen && now - x->online_ms <= (type1 ? unsigned(feedback_timeout_ms) : unsigned(motor_cfg::query_online_ms));
    bool power_stale = !x->route.PowerQueryPeriodMs || !x->power_seen || now - x->power_ms >= x->route.PowerQueryPeriodMs * 3u;
    s->PowerAgeMs = age_ms(now, x->power_ms, x->power_seen);
    bool stale;
    if (type1) {
        s->FeedbackAgeMs = age_ms(now, x->feedback_ms, true);
        stale = s->FeedbackAgeMs >= feedback_timeout_ms;
        if (stale)
            s->ValidFlags &= 64;
    } else {
        bool pos = x->position_seen && now - x->position_ms <= motor_cfg::query_position_ms;
        bool vel = x->velocity_seen && now - x->velocity_ms <= motor_cfg::query_velocity_ms;
        bool current = x->current_seen && now - x->current_ms <= motor_cfg::query_current_ms;
        s->ValidFlags = (pos ? 1 : 0) | (vel ? 2 : 0) | (current ? 4 : 0);
        const double torque_constant = x->device.capabilities().torque_constant_Nm_A;
        if (current && torque_constant > 0) {
            double torque = (double)s->ActualCurrent * torque_constant * 10;
            if (torque >= INT32_MIN && torque <= INT32_MAX) {
                s->EstimatedTorque = (int32_t)torque;
                s->ValidFlags |= 8;
            }
        }
        s->MotorTemperature = 0;
        s->MosTemperature = 0;
        stale = !(pos && vel && current);
        if (stale)
            s->FeedbackAgeMs = UINT16_MAX;
        else {
            uint32_t oldest = now - x->position_ms;
            if (now - x->velocity_ms > oldest)
                oldest = now - x->velocity_ms;
            if (now - x->current_ms > oldest)
                oldest = now - x->current_ms;
            s->FeedbackAgeMs = oldest > UINT16_MAX ? UINT16_MAX : (uint16_t)oldest;
        }
    }
    if (power_stale)
        s->ValidFlags &= 63;
    else
        s->ValidFlags |= 64;
    bool ready = type1 ? (!stale && !s->BridgeFaultCode && !s->MotorFaultRaw && ethercat_canfd_bus_state(c) != 3) : query_ready(now);
    s->StatusWord = (s->StatusWord & 0x320) | (x->configured ? 1 : 0) | (online ? 2 : 0) | (ready ? 4 : 0) | (type1 ? 8 : 0) | (x->new_feedback ? 16 : 0) | (s->BridgeFaultCode ? 32 : 0) | (s->MotorFaultRaw ? 64 : 0) | (stale ? 128 : 0) | (x->new_power ? 1024 : 0) | (power_stale ? 2048 : 0) | (!type1 && (x->position_seen || x->velocity_seen || x->current_seen) ? MC_STATUS_QUERY_SOURCE : 0) | (x->query_armed && !type1 ? MC_STATUS_ARMED_QUERY_ONLY : 0);
    x->new_feedback = x->new_power = false;
    axis_diagnostics(a)->PositionQueryAgeUs = age(now, x->position_ms, x->position_seen);
    axis_diagnostics(a)->VelocityQueryAgeUs = age(now, x->velocity_ms, x->velocity_seen);
    axis_diagnostics(a)->TxCompleteCount = ethercat_canfd_completions(a);
    return {online, bool(s->BridgeFaultCode || s->MotorFaultRaw)};
}

}
