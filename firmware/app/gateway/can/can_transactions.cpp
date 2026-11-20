#include "detail/can_transactions.hpp"
#include "encos_parameter_transactions.hpp"

namespace dcu::app::gateway_application::detail {
namespace parameters = motor::encos_parameters;

bool CanTransactions::busy(unsigned bus) const { return polls_[bus].tx_id || legacy_busy(bus); }

bool CanTransactions::any_busy() const
{
    for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus)
        if (busy(bus)) return true;
    return false;
}

bool CanTransactions::legacy_busy(unsigned bus) const
{
    return bus < DCU_DEVICE_CAN_BUS_COUNT && parameters::inspect(bus).busy;
}

bool CanTransactions::awaiting_tx(unsigned bus) const
{
    const auto &poll = polls_[bus];
    return parameters::inspect(bus).awaiting_tx || (poll.tx_id && !poll.transmitted);
}

bool CanTransactions::raw_send_allowed(unsigned bus, uint32_t now) const
{
    return !polls_[bus].tx_id && parameters::available(bus, now);
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
                                    const dcu_can_frame_t &frame, uint8_t bytes, uint32_t stamp, uint32_t now)
{
    auto &poll = polls_[bus];
    if (poll.tx_id && poll.device == device && poll.transmitted &&
        stamp - poll.submitted_ms < poll.timeout_ms && now - poll.submitted_ms < poll.timeout_ms &&
        instance.consume_status_reply(frame, stamp)) {
        poll = {};
        return true;
    } else if (bytes >= 2 && instance.supports_legacy_queries()) {
        const auto before = parameters::inspect(bus);
        const auto cyclic_before = instance.cyclic_feedback_count();
        if (instance.consume_feedback(frame, stamp, now))
            return instance.cyclic_feedback_count() != cyclic_before ||
                (before.busy && before.sent && !before.awaiting_tx && before.internal &&
                 before.device == device && before.can_id == frame.id &&
                 stamp - before.started_ms <= now - before.started_ms && !parameters::inspect(bus).busy);
    }
    return false;
}

bool CanTransactions::finish_tx(unsigned bus, const dcu_can_tx_event_t &event)
{
    parameters::finish_tx(bus, event);
    auto &poll = polls_[bus];
    if (!poll.tx_id || poll.tx_id != event.id) return false;
    if (event.status == DCU_CAN_TX_COMPLETED) poll.transmitted = true;
    else poll = {};
    return true;
}

void CanTransactions::abort_poll(unsigned bus)
{
    auto &poll = polls_[bus];
    if (poll.tx_id && !poll.transmitted) dcu_can_cancel_tx(bus, poll.tx_id);
    poll = {};
}

void CanTransactions::expire_poll(unsigned bus, uint32_t now)
{
    const auto &poll = polls_[bus];
    if (poll.tx_id && now - poll.submitted_ms >= poll.timeout_ms) abort_poll(bus);
}

void CanTransactions::expire_legacy(unsigned bus, uint32_t now) { parameters::check_timeout(bus, now); }

void CanTransactions::cancel(unsigned bus, uint16_t result)
{
    parameters::cancel(bus, result);
    abort_poll(bus);
}

void CanTransactions::reset_session(uint16_t result)
{
    parameters::reset_session(result);
    for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus) abort_poll(bus);
}

bool CanTransactions::submit_poll(unsigned bus, unsigned device, motor::MotorInstance &instance,
                                  BusScheduler &schedule, ControlSet controls, bool fd,
                                  uint32_t now, uint32_t now_us)
{
    if (dcu_can_pending(bus) || !dcu_can_tx_free(bus) || !schedule.guard_elapsed(now_us)) return false;
    motor::StatusPoll request{};
    if (!instance.prepare_status_poll(now, request) || request.frame.id != instance.can_id() ||
        !dcu_can_frame_valid(&request.frame) || !request.timeout_ms || request.reply_bytes > DCU_CAN_MAX_PAYLOAD)
        return false;
    const auto bytes = dcu_can_dlc_to_bytes(request.frame.dlc, request.frame.flags & DCU_CAN_FLAG_FD);
    const unsigned cost = BusScheduler::frame_cost(fd, bytes) + BusScheduler::frame_cost(fd, request.reply_bytes);
    if (!schedule.query_window(controls, now_us, cost, 2u * BusScheduler::frame_cost(fd, 8))) return false;
    const auto id = dcu_can_submit(bus, &request.frame);
    if (!id) return false;
    schedule.submitted(now_us, cost);
    polls_[bus] = {id, now, device, request.timeout_ms, false};
    return true;
}

bool CanTransactions::start_query(unsigned bus, unsigned device, motor::MotorInstance &instance,
                                  BusScheduler &schedule, ControlSet controls, bool pdo_owner, bool fd,
                                  uint32_t now, uint32_t now_us)
{
    if (busy(bus)) return false;
    return submit_poll(bus, device, instance, schedule, controls, fd, now, now_us) || instance.start_query(now, pdo_owner);
}

void CanTransactions::submit_queued(unsigned bus, BusScheduler &schedule, ControlSet controls, bool fd,
                                    uint32_t now, uint32_t now_us)
{
    if (polls_[bus].tx_id) return;
    const auto *request = parameters::pending(bus);
    if (!request || dcu_can_pending(bus) || !dcu_can_tx_free(bus) || !schedule.guard_elapsed(now_us)) return;
    const unsigned reply_cost = BusScheduler::frame_cost(fd, 8);
    const unsigned cost = BusScheduler::frame_cost(fd, request->dlc) + reply_cost;
    if (parameters::inspect(bus).internal && !schedule.query_window(controls, now_us, cost, 2u * reply_cost))
        return;
    if (!schedule.can_afford(cost)) return;
    if (const auto id = dcu_can_submit(bus, request)) {
        schedule.submitted(now_us, cost);
        parameters::mark_submitted(bus, now, id);
    }
}

ParameterReadResult CanTransactions::read_parameter(unsigned device, const motor::MotorInstance &instance,
                                                    const ParameterRead &read, uint8_t format, uint32_t now,
                                                    bool permitted, bool channel_active)
{
    if (!instance.supports_legacy_queries()) return {false, true, motor::RESULT_UNSUPPORTED, read.sequence};
    if (polls_[instance.bus()].tx_id) return {};
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
