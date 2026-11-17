#include "axis_controller.hpp"
#include "encos_parameter_transactions.hpp"
#include "can_transport.h"
#include "motor_result.hpp"
#include "device_config.h"
#include <climits>
#include <cmath>

namespace dcu::app {
namespace motor_cfg = config::motor_control;
using namespace motor_application;
using detail::AxisPhase;

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
bool estimate_torque(AxisFeedback &feedback, double torque_constant)
{
    if (!(torque_constant > 0)) return false;
    const double torque = double(feedback.sample.current_cA) * torque_constant * 10;
    if (!(torque >= INT32_MIN && torque <= INT32_MAX)) return false;
    feedback.sample.torque_mNm = static_cast<int32_t>(torque);
    return true;
}
}

void AxisController::initialize(unsigned axis)
{
    axis_ = axis;
}
bool AxisController::configure(bool enabled, uint8_t frame_format, const AxisConfiguration &configuration)
{
    state_ = {};
    auto *x = &state_;
    const unsigned a = axis_;
    x->route = configuration.route;
    x->power_query_period_ms = configuration.power_query_period_ms;
    if (!enabled) return true;
    const auto binding = configure_motor_driver(a, configuration, frame_format);
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
    AxisFeedback *s = &x->feedback;
    if (active())
        dcu_can_cancel_axis(a);
    x->phase = AxisPhase::Stopped;
    if (fault && !s->bridge_fault)
        s->bridge_fault = fault;
    if (fault)
        s->status_word |= (1u << 5) | (1u << 9);
    if (fault < 32 && fault)
        history_.active_fault_bits |= 1u << fault;
}
void AxisController::reset_session()
{
    state_.command = {};
    state_.previous_control = 0;
}
bool AxisController::query_ready(uint32_t now) const
{
    const unsigned a = axis_;
    const auto *x = &state_;
    return x->configured && x->online_seen && now - x->online_ms <= motor_cfg::query_online_ms &&
        x->position_seen && now - x->position_ms <= motor_cfg::query_position_ms &&
        x->velocity_seen && now - x->velocity_ms <= motor_cfg::query_velocity_ms &&
        x->current_seen && now - x->current_ms <= motor_cfg::query_current_ms &&
        fabs((double)x->feedback.sample.current_cA) <= x->route.current_limit_dA * 10.0 &&
        !x->feedback.sample.motor_fault && !x->feedback.bridge_fault &&
        dcu_can_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3;
}
bool AxisController::accept(const motor::Command &intent, bool permit, uint32_t now)
{
    const unsigned a = axis_;
    const auto *command = &intent;
    auto *x = &state_;
    AxisFeedback *s = &x->feedback;
    uint16_t prev = x->previous_control;
    x->previous_control = command->control_bits;
    if (!permit || (command->control_bits & motor::Stop) || !(command->control_bits & motor::Arm)) {
        stop(0);
        if (permit && !(command->control_bits & motor::Arm) && !(command->control_bits & motor::Stop)) {
            x->phase = AxisPhase::ReadyForArm;
            if ((command->control_bits & motor::ResetFault) && !(prev & motor::ResetFault) && dcu_can_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3 && !s->sample.motor_fault) {
                s->bridge_fault = 0;
                s->status_word &= ~((1u << 5) | (1u << 8) | (1u << 9));
                history_.active_fault_bits = 0;
            }
        }
        x->command = {};
        return true;
    }
    if ((command->control_bits & ~15u) || command->reserved_flags) {
        stop(2);
        return false;
    }
    if (x->phase == AxisPhase::Stopped || x->phase == AxisPhase::ReadyForArm) {
        if (!x->configured || x->phase != AxisPhase::ReadyForArm || (prev & motor::Arm) || (command->control_bits & motor::StartMotion))
            return false;
        x->phase = AxisPhase::QueryOnly;
    }
    if (!(command->control_bits & motor::StartMotion)) {
        if (active()) {
            stop(0);
            x->phase = AxisPhase::QueryOnly;
        }
        x->command = {};
        return true;
    }
    dcu_can_frame_t f;
    if (!x->configured || x->device.encode(*command, f) != motor::Result::Ok) {
        stop(x->configured ? 2 : 1);
        s->status_word |= 1u << 8;
        return false;
    }
    if (active() && x->command.mode != command->mode) {
        stop(8);
        s->status_word |= 1u << 8;
        return false;
    }
    if (!active()) {
        /* StartMotion needs a fresh edge after a separate query-only cycle. */
        if ((prev & motor::StartMotion) || !query_ready(now) ||
            fabs((double)history_.queried_velocity) > 20000.0 ||
            ((command->mode == motor::ControlMode::Position || command->mode == motor::ControlMode::Impedance) &&
             fabs((double)command->position_urad - history_.queried_position) > 100000.0))
            return false;
        unsigned bus = a / ENCOS_MOTORS_PER_CHANNEL;
        parameters::abort_internal(bus);
        x->startup_command = *command;
        x->startup_command.control_bits = motor::Arm;
        x->startup_command.mode = motor::ControlMode::Position;
        x->startup_command.position_urad = history_.queried_position;
        x->startup_command.velocity_urad_s = 0;
        x->startup_command.torque_mNm = 0;
        x->startup_command.current_cA = 0;
        x->startup_command.kp_centi = 0;
        x->startup_command.kd_milli = 0;
        x->startup_command.current_limit_dA = x->route.current_limit_dA < 4095 ? x->route.current_limit_dA : 4095;
        if (x->device.encode(x->startup_command, f) != motor::Result::Ok)
            return false;
        x->active_ms = now;
        x->phase = AxisPhase::StartupQueued;
        x->new_feedback = false;
        s->sample.valid_fields = 0;
    }
    x->command = *command;
    s->mode = x->phase == AxisPhase::Running ? command->mode : motor::ControlMode::Position;
    s->status_word &= ~(1u << 8);
    return true;
}
void AxisController::check_timeouts(uint32_t now, bool sdo_owner, uint16_t feedback_timeout_ms)
{
    auto *x = &state_;
    if (active() || x->phase == AxisPhase::QueryOnly) {
        if (sdo_owner && uint32_t(now - x->commit_ms) >= x->lease_ms)
            stop(3);
        // First feedback is timed only after the first successful CAN submission.
        if (control_sent() && uint32_t(now - (x->phase == AxisPhase::Running ? x->feedback_ms : x->active_ms)) >= feedback_timeout_ms)
            stop(4);
    }
}
void AxisController::check_startup(uint32_t now)
{
    if (state_.phase == AxisPhase::StartupQueued && !query_ready(now))
        stop(8);
}
bool AxisController::encode_control(dcu_can_frame_t &frame) const
{
    return active() && state_.device.encode(
        state_.phase == AxisPhase::Running ? state_.command : state_.startup_command, frame) == motor::Result::Ok;
}
void AxisController::control_submitted(uint32_t now)
{
    if (!active()) return;
    if (state_.phase == AxisPhase::StartupQueued) {
        state_.phase = AxisPhase::AwaitingFeedback;
        state_.active_ms = now;
    }
    state_.feedback.mode = state_.phase == AxisPhase::Running ? state_.command.mode : motor::ControlMode::Position;
    ++history_.tx_enqueue_count;
}
void AxisController::set_sdo_lease(uint16_t lease_ms, uint32_t now)
{
    state_.lease_ms = lease_ms;
    state_.commit_ms = now;
}
bool AxisController::start_query(uint32_t now, uint8_t owner, uint8_t frame_format)
{
    auto *x = &state_;
    if (!x->configured || (!active() && owner == OWNER_SDO && x->phase != AxisPhase::QueryOnly) ||
        (active() && (owner != OWNER_PDO || !x->power_query_period_ms)))
        return false;
    unsigned first = active() ? 3u : 0u;
    for (unsigned k = first; k < motor_cfg::query_period_ms.size(); ++k) {
        unsigned period_ms = k == 3 ? x->power_query_period_ms : motor_cfg::query_period_ms[k];
        if (!period_ms || ((x->query_requested_mask & (1u << k)) && now - x->query_request_ms[k] < period_ms))
            continue;
        if (!parameters::start_query(axis_ / ENCOS_MOTORS_PER_CHANNEL, axis_, k + 1, 1, now,
                                     x->parameter_dialect, x->route.can_id, frame_format))
            return false;
        x->query_request_ms[k] = now;
        x->query_requested_mask |= 1u << k;
        return true;
    }
    return false;
}
void AxisController::consume_feedback(const dcu_can_frame_t &frame, uint32_t stamp, uint32_t now,
                                      ParameterDialect service_dialect)
{
    const auto *f = &frame;
    const unsigned a = axis_, c = axis_ / ENCOS_MOTORS_PER_CHANNEL;
    const auto transaction = parameters::inspect(c);
    auto *x = &state_;
    uint8_t type = f->data[0] >> 5;
    bool valid = false;
    bool cyclic_feedback = false;
    motor::Feedback sample{};
    constexpr uint32_t required = motor::Position | motor::Velocity | motor::Current;
    if (control_sent() && x->configured && x->device.decode(*f, sample) == motor::Result::Ok &&
        (sample.valid_fields & required) == required) {
        x->feedback.sample = sample;
        cyclic_feedback = true;
        if (x->phase != AxisPhase::Running) {
            x->position_seen = x->velocity_seen = x->current_seen = false;
        }
        x->phase = AxisPhase::Running;
        x->feedback_ms = stamp;
        x->new_feedback = true;
        valid = true;
        ++x->type1_count;
        if (fabs((double)x->feedback.sample.current_cA) > x->route.current_limit_dA * 10.0)
            stop(2);
    }
    else {
        // PREOP parameter transactions may precede Apply configuration.
        auto dialect = transaction.busy && transaction.sent && a == transaction.axis ?
            service_dialect : x->parameter_dialect;
        if (dialect != dcu::app::ParameterDialect::EncosLegacy) return;
        auto observation = parameters::consume(c, a, *f, now);
        if (observation.acknowledged) return;
        valid = observation.valid;
        if (valid) {
            switch (observation.field) {
            case parameters::Field::Position:
                history_.queried_position = observation.value;
                x->position_ms = stamp;
                x->position_seen = true;
                if (observation.update_actual && !active()) x->feedback.sample.position_urad = observation.value;
                break;
            case parameters::Field::Velocity:
                history_.queried_velocity = observation.value;
                x->velocity_ms = stamp;
                x->velocity_seen = true;
                if (observation.update_actual && !active()) x->feedback.sample.velocity_urad_s = observation.value;
                break;
            case parameters::Field::Current:
                x->current_ms = stamp;
                x->current_seen = true;
                if (!active()) {
                    x->feedback.sample.current_cA = static_cast<int16_t>(observation.value);
                    estimate_torque(x->feedback, x->device.capabilities().torque_constant_Nm_A);
                }
                break;
            case parameters::Field::Power:
                x->feedback.power_mW = observation.value;
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
        ++history_.rx_valid_count;
        if (cyclic_feedback || (type >= 1 && type <= 5)) {
            if (!cyclic_feedback)
                x->feedback.sample.motor_fault = f->data[0] & 31;
            if (x->feedback.sample.motor_fault)
                stop(0);
        }
    } else
        ++history_.protocol_error_count;
}
AxisPublication AxisController::read_feedback(uint32_t now, uint16_t feedback_timeout_ms,
                                              AxisFeedback &feedback, AxisDiagnostics &diagnostics) const
{
    const unsigned a = axis_, c = axis_ / ENCOS_MOTORS_PER_CHANNEL;
    const auto *x = &state_;
    feedback = x->feedback;
    AxisFeedback *s = &feedback;
    bool type1 = x->phase == AxisPhase::Running;
    bool online = x->online_seen && now - x->online_ms <= (type1 ? unsigned(feedback_timeout_ms) : unsigned(motor_cfg::query_online_ms));
    bool power_stale = !x->power_query_period_ms || !x->power_seen || now - x->power_ms >= x->power_query_period_ms * 3u;
    s->power_age_ms = age_ms(now, x->power_ms, x->power_seen);
    bool stale;
    if (type1) {
        s->feedback_age_ms = age_ms(now, x->feedback_ms, true);
        stale = s->feedback_age_ms >= feedback_timeout_ms;
        if (stale)
            s->sample.valid_fields = 0;
    } else {
        bool pos = x->position_seen && now - x->position_ms <= motor_cfg::query_position_ms;
        bool vel = x->velocity_seen && now - x->velocity_ms <= motor_cfg::query_velocity_ms;
        bool current = x->current_seen && now - x->current_ms <= motor_cfg::query_current_ms;
        s->sample.valid_fields = (pos ? uint32_t(motor::Position) : 0U) |
            (vel ? uint32_t(motor::Velocity) : 0U) | (current ? uint32_t(motor::Current) : 0U);
        const double torque_constant = x->device.capabilities().torque_constant_Nm_A;
        if (current && estimate_torque(*s, torque_constant))
            s->sample.valid_fields |= motor::Torque;
        s->sample.motor_temperature_deciC = 0;
        s->sample.mos_temperature_deciC = 0;
        stale = !(pos && vel && current);
        if (stale)
            s->feedback_age_ms = UINT16_MAX;
        else {
            uint32_t oldest = now - x->position_ms;
            if (now - x->velocity_ms > oldest)
                oldest = now - x->velocity_ms;
            if (now - x->current_ms > oldest)
                oldest = now - x->current_ms;
            s->feedback_age_ms = oldest > UINT16_MAX ? UINT16_MAX : (uint16_t)oldest;
        }
    }
    s->power_valid = !power_stale;
    bool ready = type1 ? (!stale && !s->bridge_fault && !s->sample.motor_fault && dcu_can_bus_state(c) != 3) : query_ready(now);
    s->status_word = (s->status_word & 0x320) | (x->configured ? 1 : 0) | (online ? 2 : 0) | (ready ? 4 : 0) | (type1 ? 8 : 0) | (x->new_feedback ? 16 : 0) | (s->bridge_fault ? 32 : 0) | (s->sample.motor_fault ? 64 : 0) | (stale ? 128 : 0) | (x->new_power ? 1024 : 0) | (power_stale ? 2048 : 0) | (!type1 && (x->position_seen || x->velocity_seen || x->current_seen) ? QuerySource : 0) | (!type1 && (x->phase == AxisPhase::QueryOnly || active()) ? ArmedQueryOnly : 0);
    diagnostics.tx_enqueue_count = history_.tx_enqueue_count;
    diagnostics.tx_drop_count = history_.tx_drop_count;
    diagnostics.rx_valid_count = history_.rx_valid_count;
    diagnostics.protocol_error_count = history_.protocol_error_count;
    diagnostics.active_fault_bits = history_.active_fault_bits;
    diagnostics.queried_position_urad = history_.queried_position;
    diagnostics.queried_velocity_urad_s = history_.queried_velocity;
    diagnostics.position_query_age_us = age(now, x->position_ms, x->position_seen);
    diagnostics.velocity_query_age_us = age(now, x->velocity_ms, x->velocity_seen);
    diagnostics.tx_complete_count = dcu_can_completions(a);
    return {online, bool(s->bridge_fault || s->sample.motor_fault)};
}
void AxisController::acknowledge_publication()
{
    state_.new_feedback = state_.new_power = false;
}

}
