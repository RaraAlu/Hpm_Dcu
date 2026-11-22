#include "detail/can_transactions.hpp"
#include "encos_parameter_transactions.hpp"
#include "can/cfg/gateway_schedule_policy.hpp"
#if defined(CONFIG_PDO_PHASE_TRACE)
#include "realtime_port.h"
#endif

namespace dcu::app::gateway_application::detail {
namespace parameters = motor::encos_parameters;
using config::gateway_schedule::neutral_mit_reply_us;
namespace {
#if defined(CONFIG_PDO_PHASE_TRACE)
volatile NeutralMitPhaseTrace neutral_mit_phase_trace[DCU_DEVICE_CAN_BUS_COUNT][DCU_MOTOR_DEVICES_PER_BUS];

void end_neutral_trace(volatile NeutralMitPhaseTrace &trace, NeutralMitTraceOutcome outcome, uint32_t now_us)
{
    trace.ended_us = now_us;
    trace.outcome = outcome;
}
#endif

bool strictly_after(dcu_can_eof_t rx, dcu_can_eof_t tx)
{
    // Compare only within one MCAN and an already bounded transaction window.
    const uint32_t delta = rx.tick - tx.tick;
    return rx.valid && tx.valid && delta != 0 && delta < 0x80000000U;
}
}

bool CanTransactions::busy(unsigned bus, unsigned device) const
{
    const auto legacy = parameters::inspect(bus);
    return polls_[bus][slots_[device]].tx_id ||
           (legacy.busy && (legacy.device == device || legacy.can_id == legacy_broadcast_id));
}

bool CanTransactions::any_busy() const
{
    for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus) {
        if (legacy_busy(bus)) return true;
        for (const auto &poll : polls_[bus]) if (poll.tx_id) return true;
    }
    return false;
}

bool CanTransactions::legacy_busy(unsigned bus) const
{
    return bus < DCU_DEVICE_CAN_BUS_COUNT && parameters::inspect(bus).busy;
}

bool CanTransactions::awaiting_tx(unsigned bus) const
{
    for (const auto &poll : polls_[bus]) if (poll.tx_id && !poll.transmitted) return true;
    return parameters::inspect(bus).awaiting_tx;
}

bool CanTransactions::raw_send_allowed(unsigned bus, uint32_t now) const
{
    for (const auto &poll : polls_[bus]) if (poll.tx_id) return false;
    return parameters::available(bus, now);
}

uint8_t CanTransactions::reply_format(unsigned bus, uint32_t id, uint8_t fallback) const
{
    const auto transaction = parameters::inspect(bus);
    return transaction.busy && transaction.can_id == id ? transaction.frame_format : fallback;
}

bool CanTransactions::consume_broadcast_reply(unsigned bus, const dcu_can_frame_t &frame, uint8_t bytes)
{
    return bytes >= 2 && bytes <= DCU_CAN_CLASSIC_MAX_PAYLOAD && parameters::consume_maintenance(bus, frame);
}

bool CanTransactions::consume_reply(unsigned bus, unsigned device, motor::MotorInstance &instance,
                                    const dcu_can_frame_t &frame, uint8_t bytes,
                                    const dcu_can_rx_stamp_t &stamp, uint32_t now, uint32_t now_us)
{
    auto &poll = polls_[bus][slots_[device]];
#if defined(CONFIG_PDO_PHASE_TRACE)
    auto &trace = neutral_mit_phase_trace[bus][slots_[device]];
    if (poll.tx_id && poll.neutral) {
        trace.rx_checked_us = now_us;
        trace.rx_eof_us = stamp.eof.us;
        trace.rx_eof_tick = stamp.eof.tick;
        trace.rx_eof_valid = stamp.eof.valid;
    }
#endif
    if (poll.tx_id && poll.transmitted &&
        (poll.neutral ? now_us - poll.submitted_us < neutral_mit_reply_us :
            stamp.observed_ms - poll.submitted_ms < poll.timeout_ms && now - poll.submitted_ms < poll.timeout_ms) &&
        (!poll.requires_eof || strictly_after(stamp.eof, poll.eof)) &&
        (poll.neutral ? instance.consume_neutral_mit(frame, stamp.observed_ms) :
                        instance.consume_status_reply(frame, stamp.observed_ms))) {
#if defined(CONFIG_PDO_PHASE_TRACE)
        if (poll.neutral) {
            trace.consumed_us = dcu_time_now_us();
            dcu_trace_neutral_consumed(trace.consumed_us);
            end_neutral_trace(trace, NeutralMitTraceOutcome::Accepted, trace.consumed_us);
        }
#endif
        poll = {};
        return true;
    } else if (bytes >= 2 && instance.supports_legacy_queries()) {
        const auto before = parameters::inspect(bus);
        const auto cyclic_before = instance.cyclic_feedback_count();
        if (instance.consume_feedback(frame, stamp.observed_ms, now))
            return instance.cyclic_feedback_count() != cyclic_before ||
                (before.busy && before.sent && !before.awaiting_tx && before.internal &&
                 before.device == device && before.can_id == frame.id &&
                 stamp.observed_ms - before.started_ms <= now - before.started_ms && !parameters::inspect(bus).busy);
    }
    return false;
}

bool CanTransactions::finish_tx(unsigned bus, const dcu_can_tx_event_t &event)
{
    parameters::finish_tx(bus, event);
    if (!event.id) return false;
    unsigned slot = 0;
    while (slot < DCU_MOTOR_DEVICES_PER_BUS && polls_[bus][slot].tx_id != event.id) ++slot;
    if (slot == DCU_MOTOR_DEVICES_PER_BUS) return false;
    auto &poll = polls_[bus][slot];
#if defined(CONFIG_PDO_PHASE_TRACE)
    auto &trace = neutral_mit_phase_trace[bus][slot];
    if (poll.neutral) {
        trace.tx_observed_us = dcu_time_now_us();
        trace.tx_eof_us = event.eof.us;
        trace.tx_eof_tick = event.eof.tick;
        trace.tx_eof_valid = event.eof.valid;
    }
#endif
    if (event.status == DCU_CAN_TX_COMPLETED && (!poll.requires_eof || event.eof.valid)) {
        poll.transmitted = true;
        poll.eof = event.eof;
    } else {
#if defined(CONFIG_PDO_PHASE_TRACE)
        if (poll.neutral)
            end_neutral_trace(trace, NeutralMitTraceOutcome::TxNotProven, trace.tx_observed_us);
#endif
        poll = {};
    }
    return true;
}

void CanTransactions::abort_slot(unsigned bus, unsigned slot)
{
    auto &poll = polls_[bus][slot];
#if defined(CONFIG_PDO_PHASE_TRACE)
    auto &trace = neutral_mit_phase_trace[bus][slot];
    if (poll.tx_id && poll.neutral && trace.outcome == NeutralMitTraceOutcome::Pending)
        end_neutral_trace(trace, NeutralMitTraceOutcome::Cancelled, dcu_time_now_us());
#endif
    if (poll.tx_id && !poll.transmitted) dcu_can_cancel_tx(bus, poll.tx_id);
    poll = {};
}

void CanTransactions::expire_poll(unsigned bus, unsigned device, uint32_t now, uint32_t now_us)
{
    const auto slot = slots_[device];
    const auto &poll = polls_[bus][slot];
    if (poll.tx_id && (poll.neutral ? now_us - poll.submitted_us >= neutral_mit_reply_us :
                                     now - poll.submitted_ms >= poll.timeout_ms)) {
#if defined(CONFIG_PDO_PHASE_TRACE)
        if (poll.neutral) end_neutral_trace(neutral_mit_phase_trace[bus][slot], NeutralMitTraceOutcome::Expired, now_us);
#endif
        abort_slot(bus, slot);
    }
}

bool CanTransactions::neutral_pending(unsigned bus, unsigned device) const
{
    const auto &poll = polls_[bus][slots_[device]];
    return poll.tx_id && poll.neutral;
}

void CanTransactions::cancel_neutral(unsigned bus, unsigned device)
{
    if (neutral_pending(bus, device)) abort_slot(bus, slots_[device]);
}

bool CanTransactions::submit_neutral(unsigned bus, unsigned device, motor::MotorInstance &instance,
                                     BusScheduler &schedule, uint32_t now, uint32_t now_us, uint8_t counter,
                                     bool drained)
{
    dcu_can_frame_t frame{};
    if (!instance.take_neutral_mit(frame, counter)) return false;
    const auto *channel = dcu_can_channel_config(bus);
    const unsigned cost = 2U * BusScheduler::frame_cost(true, 16);
    if (!drained || !channel || !channel->timestamp_eof || busy(bus, device) || dcu_can_pending(bus, slots_[device]) ||
        !dcu_can_tx_free(bus) || !schedule.guard_elapsed(now_us) || !schedule.can_afford(cost))
        return false;
    const auto id = dcu_can_submit(bus, slots_[device], &frame);
    if (!id) return false;
#if defined(CONFIG_PDO_PHASE_TRACE)
    auto &trace = neutral_mit_phase_trace[bus][slots_[device]];
    trace.submit_us = now_us;
    trace.tx_eof_us = trace.rx_eof_us = trace.consumed_us = 0;
    trace.tx_observed_us = trace.rx_checked_us = 0;
    trace.ended_us = 0;
    trace.tx_eof_tick = 0;
    trace.tx_eof_valid = 0;
    trace.rx_eof_tick = 0;
    trace.rx_eof_valid = 0;
    trace.outcome = NeutralMitTraceOutcome::Pending;
#endif
    schedule.submitted(now_us, cost);
    polls_[bus][slots_[device]] = {id, now, 0, false, true, {}, true, now_us};
    return true;
}

void CanTransactions::expire_legacy(unsigned bus, uint32_t now) { parameters::check_timeout(bus, now); }

void CanTransactions::cancel(unsigned bus, uint16_t result)
{
    parameters::cancel(bus, result);
    for (unsigned slot = 0; slot < DCU_MOTOR_DEVICES_PER_BUS; ++slot) abort_slot(bus, slot);
}

void CanTransactions::reset_session(uint16_t result)
{
    parameters::reset_session(result);
    for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus)
        for (unsigned slot = 0; slot < DCU_MOTOR_DEVICES_PER_BUS; ++slot) abort_slot(bus, slot);
}

bool CanTransactions::submit_poll(unsigned bus, unsigned device, motor::MotorInstance &instance,
                                  BusScheduler &schedule, ControlSet controls, bool fd,
                                  uint32_t now, uint32_t now_us)
{
    const auto *channel = dcu_can_channel_config(bus);
    if (!channel || dcu_can_pending(bus, slots_[device]) || !dcu_can_tx_free(bus) || !schedule.guard_elapsed(now_us)) return false;
    motor::StatusPoll request{};
    if (!instance.prepare_status_poll(now, request) || request.frame.id != instance.can_id() ||
        !dcu_can_frame_valid(&request.frame) || !request.timeout_ms || request.reply_bytes > DCU_CAN_MAX_PAYLOAD)
        return false;
    const auto bytes = dcu_can_dlc_to_bytes(request.frame.dlc, request.frame.flags & DCU_CAN_FLAG_FD);
    const unsigned cost = BusScheduler::frame_cost(fd, bytes) + BusScheduler::frame_cost(fd, request.reply_bytes);
    if (!schedule.query_window(controls, now_us, cost, 2u * BusScheduler::frame_cost(fd, 8))) return false;
    const auto id = dcu_can_submit(bus, slots_[device], &request.frame);
    if (!id) return false;
    schedule.submitted(now_us, cost);
    polls_[bus][slots_[device]] = {id, now, request.timeout_ms, false, channel->timestamp_eof, {}, false, 0};
    return true;
}

bool CanTransactions::start_query(unsigned bus, unsigned device, motor::MotorInstance &instance,
                                  BusScheduler &schedule, ControlSet controls, bool pdo_owner, bool fd,
                                  uint32_t now, uint32_t now_us)
{
    if (busy(bus, device)) return false;
    return submit_poll(bus, device, instance, schedule, controls, fd, now, now_us) || instance.start_query(now, pdo_owner);
}

void CanTransactions::submit_queued(unsigned bus, BusScheduler &schedule, ControlSet controls, bool fd,
                                    uint32_t now, uint32_t now_us)
{
    const auto *request = parameters::pending(bus);
    if (!request) return;
    const auto legacy = parameters::inspect(bus);
    const unsigned slot = legacy.can_id == legacy_broadcast_id ? DCU_CAN_DEVICE_ANY : slots_[legacy.device];
    if (slot == DCU_CAN_DEVICE_ANY) {
        for (const auto &poll : polls_[bus]) if (poll.tx_id) return;
    } else if (polls_[bus][slot].tx_id) return;
    if (dcu_can_pending(bus, slot) || !dcu_can_tx_free(bus) || !schedule.guard_elapsed(now_us)) return;
    const unsigned reply_cost = BusScheduler::frame_cost(fd, 8);
    const unsigned cost = BusScheduler::frame_cost(fd, request->dlc) + reply_cost;
    if (legacy.internal && !schedule.query_window(controls, now_us, cost, 2u * reply_cost))
        return;
    if (!schedule.can_afford(cost)) return;
    if (const auto id = dcu_can_submit(bus, slot, request)) {
        schedule.submitted(now_us, cost);
        parameters::mark_submitted(bus, now, id);
    }
}

ParameterReadResult CanTransactions::read_parameter(unsigned device, const motor::MotorInstance &instance,
                                                    const ParameterRead &read, uint8_t format, uint32_t now,
                                                    bool permitted, bool channel_active)
{
    if (!instance.supports_legacy_queries()) return {false, true, motor::RESULT_UNSUPPORTED, read.sequence};
    if (polls_[instance.bus()][slots_[device]].tx_id) return {};
    parameters::Request request{};
    request.device = device;
    request.bus = instance.bus();
    request.dialect = motor::ParameterDialect::EncosLegacy;
    request.can_id = instance.can_id();
    request.frame_format = format;
    request.sequence = read.sequence;
    request.done_sequence = read.done_sequence;
    request.parameter = read.parameter;
    request.opcode = 1;
    const auto outcome = parameters::commit(request, now, permitted, channel_active);
    return {outcome.accepted, outcome.changed, outcome.result, outcome.done_sequence};
}

}
