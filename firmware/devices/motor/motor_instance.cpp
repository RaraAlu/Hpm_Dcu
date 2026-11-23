#include "motor_instance.hpp"
#include "motor_driver_binding.hpp"
#include "motor_limits.hpp"
#include <climits>

namespace dcu::motor {
using detail::MotorPhase;

namespace {
uint16_t age_ms(uint32_t now, uint32_t then, bool seen)
{
    uint32_t d = now - then;
    return !seen || d > UINT16_MAX ? UINT16_MAX : (uint16_t)d;
}
bool estimate_torque(MotorFeedback &feedback, double torque_constant)
{
    if (!(torque_constant > 0)) return false;
    const double torque = double(feedback.sample.current_cA) * torque_constant * 10;
    if (!(torque >= INT32_MIN && torque <= INT32_MAX)) return false;
    feedback.sample.torque_mNm = static_cast<int32_t>(torque);
    return true;
}
}

bool MotorInstance::bind(unsigned device, const MotorConfiguration &configuration, uint8_t frame_format)
{
    disable();
    state_ = {};
    control_tx_id_ = 0;
    bus_ = configuration.bus;
    route_ = configuration.route;
    driver_ = configure_motor_driver(device, configuration, frame_format);
    return driver_ != nullptr;
}

void MotorInstance::disable()
{
    stop(0);
    driver_ = nullptr;
}
void MotorInstance::stop(uint16_t fault)
{
    auto *x = &state_;
    MotorFeedback *s = &x->feedback;
    if (active() && control_tx_id_)
        dcu_can_cancel_tx(bus_, control_tx_id_);
    x->phase = MotorPhase::Stopped;
    x->neutral_armed = x->neutral_pending = false;
    if (fault && !s->bridge_fault)
        s->bridge_fault = fault;
    if (fault)
        s->status_word |= (1u << 5) | (1u << 9);
    if (fault < 32 && fault)
        history_.active_fault_bits |= 1u << fault;
}
void MotorInstance::reset_session()
{
    state_.command = {};
    state_.previous_control = 0;
    state_.neutral_armed = state_.neutral_pending = false;
    state_.neutral_reply = false;
    if (neutral_mit_only()) state_.new_feedback = false;
}

bool MotorInstance::accept_neutral_mit(const Command &command, bool permit, bool continuing)
{
    auto &x = state_;
    if (!neutral_mit_only() || !permit || !is_zero_mit(command)) {
        x.neutral_armed = x.neutral_pending = false;
        return false;
    }
    if (command.control_bits == Arm && !continuing) {
        x.neutral_armed = true;
        return true;
    }
    if (command.control_bits != (Arm | StartMotion)) {
        x.neutral_armed = x.neutral_pending = false;
        return false;
    }
    x.neutral_pending = x.neutral_armed || continuing;
    x.neutral_armed = false;
    return true;
}

bool MotorInstance::take_neutral_mit(CanFrame &frame, uint8_t counter)
{
    if (!state_.neutral_pending) return false;
    state_.neutral_pending = false; // A busy bus or failed submit must not schedule a later send.
    return driver_->encode_neutral_mit(frame, counter);
}

bool MotorInstance::consume_neutral_mit(const CanFrame &frame, uint32_t stamp)
{
    constexpr uint32_t required = Position | Velocity | Current;
    if (!neutral_mit_only() || !consume_status_reply(frame, stamp) ||
        state_.feedback.sample.motor_fault || (state_.feedback.sample.valid_fields & required) != required)
        return false;
    state_.neutral_reply = true;
    ++state_.cyclic_feedback_count;
    return true;
}
void MotorInstance::abort_neutral_mit(uint16_t fault)
{
    stop(fault);
    state_.neutral_reply = state_.new_feedback = false;
}
bool MotorInstance::query_ready(uint32_t now) const
{
    return driver_ && driver_->query_ready(now) &&
        (driver_->zero_mit_loopback() ||
         within_magnitude(state_.feedback.sample.current_cA, int64_t{route_.current_limit_dA} * 10)) &&
        !state_.feedback.sample.motor_fault && !state_.feedback.bridge_fault &&
        dcu_can_bus_state(bus_) != DCU_CAN_BUS_OFF;
}
bool MotorInstance::accept(const Command &intent, bool permit, uint32_t now)
{
    const auto *command = &intent;
    auto *x = &state_;
    MotorFeedback *s = &x->feedback;
    uint16_t prev = x->previous_control;
    x->previous_control = command->control_bits;
    if (!permit || (command->control_bits & Stop) || !(command->control_bits & Arm)) {
        stop(0);
        if (permit && !(command->control_bits & Arm) && !(command->control_bits & Stop)) {
            x->phase = MotorPhase::ReadyForArm;
            if ((command->control_bits & ResetFault) && !(prev & ResetFault) && dcu_can_bus_state(bus_) != DCU_CAN_BUS_OFF && !s->sample.motor_fault) {
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
    if (!driver_ || !driver_->capabilities().control_modes) return false;
    if (x->phase == MotorPhase::Stopped || x->phase == MotorPhase::ReadyForArm) {
        if (!configured() || x->phase != MotorPhase::ReadyForArm || (prev & Arm) || (command->control_bits & StartMotion))
            return false;
        x->phase = MotorPhase::QueryOnly;
    }
    if (!(command->control_bits & StartMotion)) {
        if (active()) {
            stop(0);
            x->phase = MotorPhase::QueryOnly;
        }
        x->command = {};
        return true;
    }
    dcu_can_frame_t f;
    if (!configured() || !within_limits(*command, route_) || driver_->encode(*command, f) != Result::Ok) {
        stop(configured() ? 2 : 1);
        s->status_word |= 1u << 8;
        return false;
    }
    // A zero-effort start skips the position-hold entry; adding effort requires a fresh arm.
    if (active() && (x->command.mode != command->mode ||
        (impedance_without_effort(x->startup_command) && !impedance_without_effort(*command)))) {
        stop(8);
        s->status_word |= 1u << 8;
        return false;
    }
    if (!active()) {
        // Motion always requires a fresh edge and a driver-approved first command.
        if ((prev & StartMotion) || !query_ready(now) ||
            !driver_->prepare_start(*command, now, x->startup_command) || !within_limits(x->startup_command, route_))
            return false;
        x->active_ms = now;
        x->phase = MotorPhase::StartupQueued;
        x->new_feedback = false;
        s->sample.valid_fields = 0;
    }
    x->command = *command;
    s->mode = x->phase == MotorPhase::Running ? command->mode : x->startup_command.mode;
    s->status_word &= ~(1u << 8);
    return true;
}
void MotorInstance::check_timeouts(uint32_t now, bool sdo_owner, uint16_t feedback_timeout_ms)
{
    auto *x = &state_;
    if (active() || x->phase == MotorPhase::QueryOnly) {
        if (sdo_owner && uint32_t(now - x->commit_ms) >= x->lease_ms)
            stop(3);
        // First feedback is timed only after the first successful CAN submission.
        if (control_sent() && uint32_t(now - (x->phase == MotorPhase::Running ? x->feedback_ms : x->active_ms)) >= feedback_timeout_ms)
            stop(4);
    }
}
void MotorInstance::check_startup(uint32_t now)
{
    if (state_.phase == MotorPhase::StartupQueued && !query_ready(now))
        stop(8);
}
bool MotorInstance::encode_control(dcu_can_frame_t &frame) const
{
    return active() && driver_->encode(
        state_.phase == MotorPhase::Running ? state_.command : state_.startup_command, frame) == Result::Ok;
}
void MotorInstance::control_submitted(uint32_t now, dcu_can_tx_id_t id)
{
    if (!active() || !id) return;
    control_tx_id_ = id;
    if (state_.phase == MotorPhase::StartupQueued) {
        state_.phase = MotorPhase::AwaitingFeedback;
        state_.active_ms = now;
    }
    state_.feedback.mode = state_.phase == MotorPhase::Running ? state_.command.mode : state_.startup_command.mode;
    ++history_.tx_enqueue_count;
}
bool MotorInstance::finish_tx(const dcu_can_tx_event_t &event)
{
    if (!control_tx_id_ || event.id != control_tx_id_) return false;
    control_tx_id_ = 0;
    if (event.status == DCU_CAN_TX_COMPLETED) ++history_.tx_complete_count;
    else if (active()) stop(6);
    return true;
}
void MotorInstance::set_sdo_lease(uint16_t lease_ms, uint32_t now)
{
    state_.lease_ms = lease_ms;
    state_.commit_ms = now;
}
bool MotorInstance::start_query(uint32_t now, bool cyclic_owner)
{
    if (!driver_ || (!cyclic_owner && (active() || state_.phase != MotorPhase::QueryOnly)))
        return false;
    return driver_->start_query(now, active());
}

bool MotorInstance::prepare_status_poll(uint32_t now, StatusPoll &poll)
{
    return driver_ && driver_->prepare_status_poll(now, poll);
}

bool MotorInstance::consume_status_reply(const CanFrame &frame, uint32_t stamp)
{
    if (!matches(frame.id) || !driver_->observe_status_reply(frame, stamp, state_.feedback.sample,
            history_.queried_position, history_.queried_velocity))
        return false;
    state_.online_ms = stamp;
    state_.new_feedback = true;
    ++history_.rx_valid_count;
    if (state_.feedback.sample.motor_fault) stop(0);
    return true;
}

bool MotorInstance::consume_feedback(const CanFrame &frame, uint32_t stamp, uint32_t now)
{
    if (!matches(frame.id) || frame.dlc < 2) return false;
    auto &x = state_;
    Feedback sample{};
    constexpr uint32_t required = Position | Velocity | Current;
    if (control_sent() && driver_->decode(frame, sample) == Result::Ok &&
        (sample.valid_fields & required) == required) {
        x.feedback.sample = sample;
        driver_->observe_cyclic(stamp, x.phase != MotorPhase::Running);
        x.phase = MotorPhase::Running;
        x.feedback_ms = stamp;
        x.new_feedback = true;
        ++x.cyclic_feedback_count;
        if (!driver_->zero_mit_loopback() &&
            !within_magnitude(sample.current_cA, int64_t{route_.current_limit_dA} * 10)) stop(2);
    } else {
        const auto result = driver_->observe(frame, stamp, now, active(), x.feedback.sample,
                                             history_.queried_position, history_.queried_velocity);
        if (result == ObservationResult::Acknowledged) return false;
        if (result != ObservationResult::Valid) {
            ++history_.protocol_error_count;
            return false;
        }
    }
    x.online_ms = stamp;
    ++history_.rx_valid_count;
    if (x.feedback.sample.motor_fault) stop(0);
    return true;
}

MotorPublication MotorInstance::read_feedback(uint32_t now, uint16_t feedback_timeout_ms,
                                              MotorFeedback &feedback, MotorDiagnostics &diagnostics) const
{
    const auto &x = state_;
    feedback = x.feedback;
    const auto query = driver_ && (!neutral_mit_only() || x.neutral_reply) ?
        driver_->query_sample(now) : QuerySample{};
    const bool running = x.phase == MotorPhase::Running;
    const bool online = running ? now - x.online_ms <= feedback_timeout_ms : query.online;
    feedback.power_mW = query.power_mW;
    feedback.power_age_ms = query.power_age_ms;
    feedback.power_valid = query.power_valid;

    bool stale;
    if (running) {
        feedback.feedback_age_ms = age_ms(now, x.feedback_ms, true);
        stale = feedback.feedback_age_ms >= feedback_timeout_ms;
        if (stale) feedback.sample.valid_fields = 0;
    } else {
        feedback.sample.valid_fields = query.valid_fields;
        if ((query.valid_fields & Current) && estimate_torque(feedback, driver_->capabilities().torque_constant_Nm_A))
            feedback.sample.valid_fields |= Torque;
        if (!(query.valid_fields & MotorTemperature)) feedback.sample.motor_temperature_deciC = 0;
        if (!(query.valid_fields & MosTemperature)) feedback.sample.mos_temperature_deciC = 0;
        feedback.feedback_age_ms = query.feedback_age_ms;
        stale = (query.valid_fields & (Position | Velocity | Current)) != (Position | Velocity | Current);
    }
    const bool ready = running ? (!stale && !feedback.bridge_fault && !feedback.sample.motor_fault &&
                                  dcu_can_bus_state(bus_) != DCU_CAN_BUS_OFF) : query_ready(now);
    feedback.status_word = (feedback.status_word & 0x320) | (configured() ? 1 : 0) |
        (online ? 2 : 0) | (ready ? 4 : 0) | (running ? 8 : 0) | (x.new_feedback ? 16 : 0) |
        (feedback.bridge_fault ? 32 : 0) | (feedback.sample.motor_fault ? 64 : 0) | (stale ? 128 : 0) |
        (query.new_power ? 1024 : 0) | (!query.power_valid ? 2048 : 0) |
        (!running && query.has_motion_data ? QuerySource : 0) |
        (!running && (x.phase == MotorPhase::QueryOnly || active()) ? ArmedQueryOnly : 0);
    diagnostics.tx_enqueue_count = history_.tx_enqueue_count;
    diagnostics.tx_drop_count = history_.tx_drop_count;
    diagnostics.rx_valid_count = history_.rx_valid_count;
    diagnostics.protocol_error_count = history_.protocol_error_count;
    diagnostics.active_fault_bits = history_.active_fault_bits;
    diagnostics.queried_position_urad = history_.queried_position;
    diagnostics.queried_velocity_urad_s = history_.queried_velocity;
    diagnostics.position_query_age_us = query.position_age_us;
    diagnostics.velocity_query_age_us = query.velocity_age_us;
    diagnostics.tx_complete_count = history_.tx_complete_count;
    return {online, bool(feedback.bridge_fault || feedback.sample.motor_fault)};
}
void MotorInstance::acknowledge_publication()
{
    state_.new_feedback = false;
    if (driver_) driver_->acknowledge_queries();
}

}
