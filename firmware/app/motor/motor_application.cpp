#include "detail/motor_application_impl.hpp"
#include "encos_parameter_transactions.hpp"
#include "can_transport.h"
#include "realtime_port.h"

namespace dcu::app::motor_application::detail {
namespace motor_cfg = dcu::app::config::motor_control;

namespace {
bool newer(uint32_t a, uint32_t b)
{
    uint32_t d = a - b;
    return d && d < 0x80000000u;
}
bool any_service(void)
{
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        if (parameters::inspect(c).busy)
            return true;
    return false;
}
}

unsigned MotorApplication::frame_us(unsigned c, unsigned bytes) const
{
    return dcu::app::BusScheduler::frame_cost(state_.channels[c].frame_format != 0, bytes);
}
ControlSet MotorApplication::controls_on_bus(unsigned c) const
{
    dcu::app::ControlSet controls{};
    controls.period_us = state_.channel_command[c].period_us;
    for (unsigned slot = 0; slot < ENCOS_MOTORS_PER_CHANNEL; ++slot) {
        auto &axis = axes_[c * ENCOS_MOTORS_PER_CHANNEL + slot];
        if (!axis.active()) continue;
        controls.active_mask |= 1u << slot;
        if (!axis.control_sent()) controls.first_send_mask |= 1u << slot;
    }
    return controls;
}
bool MotorApplication::any_active() const
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        if (axes_[a].active())
            return true;
    return false;
}
bool MotorApplication::channel_active(unsigned c) const
{
    for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
        if (axes_[a].active())
            return true;
    return false;
}
bool MotorApplication::command_permitted(uint32_t now) const
{
    return state_.operational && state_.session_armed && state_.seen_cycle &&
        (state_.board_command.control_bits & 1) && !(state_.board_command.control_bits & 2) &&
        now - state_.last_command_ms < state_.board_command.lease_ms;
}
void MotorApplication::stop_all(uint16_t fault)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        axes_[a].stop(axes_[a].configured() ? fault : 0);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        dcu_can_cancel(c);
    state_.session_armed = false;
}
void MotorApplication::operational(bool op)
{
    if (!op) {
        /* Leaving OP ends this PDO session.  The idle EtherCAT master may
         * keep the slave in PREOP, but that must not keep CAN polling alive. */
        state_.resume_pdo_owner = state_.resume_pdo_owner || state_.owner == OWNER_PDO;
        state_.owner = OWNER_NONE;
    } else if (state_.resume_pdo_owner && state_.owner == OWNER_NONE) {
        /* A transient OP loss in the same HMI session can recover without
         * repeating the PREOP SDO setup.  A new owner write cancels this. */
        state_.owner = OWNER_PDO;
        state_.resume_pdo_owner = false;
    }
    state_.operational = op;
    state_.seen_cycle = false;
    raw_.reset_session();
    state_.board_command = {};
    stop_all(0);
    if (!op)
        parameters::reset_session(RESULT_STATE);
    for (auto &axis : axes_)
        axis.reset_session();
}
void MotorApplication::cycle_fault()
{
    operational(false);
    /* Keep the existing invalid-PDO bit understood by the master's 0xfc mask.
     * Detailed cycle causes remain in cycle_diagnostics. */
    state_.board_fault |= 64U;
}
void MotorApplication::check_timeouts(uint32_t now)
{
    if (!state_.operational) return;
    if (state_.seen_cycle && uint32_t(now - state_.last_command_ms) >= state_.board_command.lease_ms) {
        state_.board_fault |= 4;
        stop_all(3);
    }
    const bool sdo_owner = state_.owner == OWNER_SDO;
    for (auto &axis : axes_)
        axis.check_timeouts(now, sdo_owner, state_.config.feedback_timeout_ms);
}
bool MotorApplication::accept_candidate(const CycleCommand &candidate, uint32_t now, ProcessResult &result)
{
    const auto &b = candidate.board;
    /* Stop intent always wins, including duplicate sequence or malformed targets. */
    bool stop = !(b.control_bits & 1) || (b.control_bits & 2);
    if (stop)
        stop_all(0);
    if (state_.mode != OperatingMode::Raw)
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
            const auto &c = candidate.axes[a];
            const auto &ch = candidate.channels[a / ENCOS_MOTORS_PER_CHANNEL];
            if (!(c.control_bits & 1) || (c.control_bits & 2) || !(ch.control_bits & 1) || (ch.control_bits & 2) || !(b.enabled_channels & (1u << (a / ENCOS_MOTORS_PER_CHANNEL))) || !(b.enabled_axes & (1u << a)) || !(ch.active_motors & (1u << (a % ENCOS_MOTORS_PER_CHANNEL))))
                axes_[a].stop(0);
        }
    if (!b.layout_valid || (b.control_bits & ~7u) || (b.enabled_channels & 0xF0) || (b.enabled_axes & 0xF0000000u) || !b.lease_ms || b.lease_ms > state_.config.max_command_lease_ms) {
        state_.board_fault |= 64;
        stop_all(9);
        return false;
    }
    if (!state_.operational || (state_.seen_cycle && !newer(b.sequence, state_.last_cycle)))
        return false;
    if (!state_.session_armed && !stop) {
        stop_all(8);
        return false;
    }
    if (stop && !b.enabled_axes)
        state_.session_armed = true;
    if ((b.control_bits & 4) && !(state_.board_command.control_bits & 4) && stop)
        state_.board_fault = 0;
    state_.last_cycle = b.sequence;
    state_.seen_cycle = true;
    state_.last_command_ms = now;
    state_.board_command = b;
    if (state_.mode == OperatingMode::Raw) {
        return true;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        const auto &ch = candidate.channels[c];
        state_.channel_command[c] = ch;
        bool valid = !(ch.control_bits & ~3u) && !(ch.active_motors & 0x80u) && ch.period_us >= 1000u && ch.period_us <= 10000u && ch.period_us % 1000u == 0 && ch.expected_role == state_.channels[c].role;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++) {
            unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
            const auto &cmd = candidate.axes[a];
            bool permit = valid && !stop && (b.enabled_channels & (1u << c)) && (b.enabled_axes & (1u << a)) && (ch.control_bits & 1) && !(ch.control_bits & 2) && (ch.active_motors & (1u << j));
            if (!valid) {
                axes_[a].stop(7);
                continue;
            }
            if (state_.owner == OWNER_PDO && axes_[a].accept(cmd, permit, now) && axes_[a].active())
                result.mirrored_axes |= 1u << a;
        }
    }
    return true;
}
bool MotorApplication::apply_configuration(const Configuration &configuration)
{
    const auto *b = &configuration.board;
    if (!configuration_writable() || any_service() || b->max_command_lease_ms < 2 || b->max_command_lease_ms > 10 || b->feedback_timeout_ms < 5 || b->feedback_timeout_ms > 20)
        return false;
    uint32_t mask = 0;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        const auto *ch = &configuration.channels[c];
        if (ch->role > 3 || ch->frame_format > 1 || ch->nominal_bitrate != 1000000 || ch->data_bitrate != 5000000 || ch->schedule_us != 2000 || ch->raw_rate_limit > 8)
            return false;
        if ((ch->role == 0 || ch->role == 2) && ch->configured_motors)
            return false;
        if (state_.mode == OperatingMode::Raw && ch->configured_motors)
            return false;
        if (ch->configured_motors & 0x80u)
            return false;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++)
            if (ch->configured_motors & (1u << j)) {
                unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
                const auto &axis_config = configuration.axes[a];
                if (!validate_motor_configuration(axis_config, ch->frame_format))
                    return false;
                for (unsigned previous = 0; previous < j; ++previous)
                    if ((ch->configured_motors & (1u << previous)) &&
                        configuration.axes[c * ENCOS_MOTORS_PER_CHANNEL + previous].route.can_id == axis_config.route.can_id)
                        return false;
                mask |= 1u << a;
            }
    }
    if (mask != b->configured_axes)
        return false;
    stop_all(0);
    state_.config = *b;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        state_.channels[c] = configuration.channels[c];
        schedule_[c].reset_budget(dcu_time_now_ms());
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        if (!axes_[a].configure((mask & (1u << a)) != 0,
                                state_.channels[a / ENCOS_MOTORS_PER_CHANNEL].frame_format,
                                configuration.axes[a])) {
            axes_[a].stop(1);
            stop_all(1);
            state_.config_ready = false;
            for (auto &axis : axes_) axis.disable();
            return false;
        }
    }
    state_.config_ready = true;
    return true;
}
void MotorApplication::consume_feedback(unsigned c, const dcu_can_frame_t &frame, uint32_t stamp, uint32_t now)
{
    const auto *f = &frame;
    auto transaction = parameters::inspect(c);
    const uint8_t format = transaction.busy && transaction.can_id == f->id ?
        transaction.frame_format : state_.channels[c].frame_format;
    if ((f->flags & 2) || f->flags != (format ? 24 : 0) || f->dlc < 2)
        return;
    unsigned a = ENCOS_AXIS_COUNT;
    if (parameters::consume_maintenance(c, *f))
        return;
    for (unsigned j = c * ENCOS_MOTORS_PER_CHANNEL; j < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; j++)
        if (axes_[j].matches(f->id)) {
            a = j;
            break;
        }
    if (transaction.busy && transaction.can_id == f->id)
        a = transaction.axis;
    if (a == ENCOS_AXIS_COUNT)
        return;
    const auto service_dialect = transaction.busy && transaction.sent && transaction.axis == a ?
        transaction.dialect : ParameterDialect::None;
    axes_[a].consume_feedback(frame, stamp, now, service_dialect);
}
ProcessResult MotorApplication::process(const CycleCommand *candidate)
{
    ProcessResult result{};
    uint32_t now_us = dcu_time_now_us();
    if ((uint32_t)(now_us - state_.last_process_us) < motor_cfg::process_period_us)
        return result;
    state_.last_process_us = now_us;
    uint32_t now = dcu_time_now_ms();
    result.consumed = candidate != nullptr;
    result.accepted = candidate && accept_candidate(*candidate, now, result);
    check_timeouts(now);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        schedule_[c].refill(now);
        dcu_can_frame_t f;
        uint32_t stamp;
        for (unsigned n = 0; n < 32 && dcu_can_receive(c, &f, &stamp); n++) {
            consume_feedback(c, f, stamp, now);
            if (state_.channels[c].role >= 2)
                raw_.receive(c, f);
        }
        if (dcu_can_bus_state(c) == 3) {
            state_.board_fault |= 8;
            parameters::cancel(c, RESULT_IO);
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (axes_[a].configured())
                    axes_[a].stop(5);
            continue;
        }
        parameters::check_timeout(c, now);
        const auto transaction = parameters::inspect(c);
        /* Do not queue a delayed start after its query snapshot has expired. */
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            axes_[a].check_startup(now);
        auto controls = controls_on_bus(c);
        if (schedule_[c].stalled(now, dcu_can_tx_progress(c),
                                 dcu_can_pending(c), state_.config.feedback_timeout_ms)) {
            /* 只把“队列持续无任何发送完成”视为真正的 TX 卡死。
             * 正常多轴流水发送时 TXBRP 可以跨调度周期保持非零。 */
            dcu_can_cancel(c);
            ++state_.channel_tx_drop[c];
            ++state_.tx_total;
            state_.board_fault |= 32;
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (axes_[a].active()) {
                    axes_[a].record_tx_drop();
                    axes_[a].stop(6);
                }
            continue;
        }
        if (state_.operational && state_.session_armed && state_.mode != OperatingMode::Raw) {
            unsigned cost = 2u * frame_us(c, 8);
            bool ready = !dcu_can_pending(c) && dcu_can_tx_free(c) > 0;
            int selected = schedule_[c].next_control(controls, now_us, cost, ready);
            if (selected >= 0) {
                unsigned slot = static_cast<unsigned>(selected);
                unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                auto &axis = axes_[a];
                if (axis.encode_control(f)) {
                    f.flags = state_.channels[c].frame_format ? 24 : 0;
                    if (dcu_can_send(c, &f, a)) {
                        schedule_[c].control_submitted(slot, now_us, controls.period_us, !axis.control_sent(), cost);
                        axis.control_submitted(now);
                        controls.first_send_mask &= ~(1u << slot);
                    } else {
                        axis.record_tx_drop();
                        ++state_.channel_tx_drop[c];
                        ++state_.tx_total;
                    }
                } else {
                    axis.stop(2);
                    controls.active_mask &= ~(1u << slot);
                }
            }
        }
        const auto *request = parameters::pending(c);
        if (request && !dcu_can_pending(c) && dcu_can_tx_free(c) &&
            schedule_[c].guard_elapsed(now_us)) {
            unsigned cost = frame_us(c, request->dlc) + frame_us(c, 8);
            bool query_window = !transaction.internal || schedule_[c].query_window(controls, now_us, cost, 2u * frame_us(c, 8));
            if (query_window && schedule_[c].can_afford(cost) && dcu_can_send(c, request, ENCOS_AXIS_COUNT)) {
                schedule_[c].submitted(now_us, cost);
                parameters::mark_sent(c, now);
            }
        }
        const Owner owner = state_.owner;
        if (!transaction.busy && (owner == OWNER_PDO || owner == OWNER_SDO) && state_.mode != OperatingMode::Raw && state_.channels[c].role != 0) {
            /* Internal queries are strictly best effort.  Reserve one complete
             * control sweep and enough wall-clock slack before creating the
             * service, otherwise the query waits without stealing a deadline.
             * Query-only axes take priority; active axes need only code 4
             * because Type-1 already carries position, velocity, current,
             * temperatures and the motor fault. */
            unsigned query_cost = frame_us(c, 2) + frame_us(c, 8);
            if (schedule_[c].query_window(controls, now_us, query_cost, 2u * frame_us(c, 8))) {
                for (unsigned pass = 0; pass < 2 && !parameters::inspect(c).busy; pass++) {
                    for (unsigned n = 0; n < ENCOS_MOTORS_PER_CHANNEL; n++) {
                        unsigned slot = (state_.query_slot[c] + n) % ENCOS_MOTORS_PER_CHANNEL;
                        unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                        auto &axis = axes_[a];
                        if (!axis.configured() || (pass == 0 ? axis.active() : !axis.active()))
                            continue;
                        if (axis.start_query(now, owner, state_.channels[c].frame_format))
                            state_.query_slot[c] = (slot + 1) % ENCOS_MOTORS_PER_CHANNEL;
                        if (parameters::inspect(c).busy)
                            break;
                    }
                }
            }
        }
    }
    if (result.accepted)
        result.raw_mirrored = process_raw(*candidate, now);
    return result;
}
bool MotorApplication::process_raw(const CycleCommand &candidate, uint32_t now)
{
    const auto mode = state_.mode;
    if (mode == OperatingMode::Motor) return false;
    RawCanPolicy policy{};
    policy.permitted = command_permitted(now);
    if (mode == OperatingMode::Raw && !policy.permitted) return false;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        auto &bus = policy.buses[c];
        const auto &channel = state_.channels[c];
        bus.can_send = (state_.board_command.enabled_channels & (1u << c)) &&
            (channel.role == 2 || channel.role == 3) && parameters::available(c, now);
        bus.rate_limit = channel.raw_rate_limit;
        bus.fd_response = channel.frame_format != 0;
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; ++a)
            if (axes_[a].configured())
                bus.protected_ids[bus.protected_count++] = axes_[a].can_id();
    }
    const unsigned batches = mode == OperatingMode::Raw ? ENCOS_CHANNEL_COUNT : 1;
    for (unsigned c = 0; c < batches; ++c) {
        const auto drops = raw_.process(candidate.raw[c], policy, now, schedule_);
        state_.tx_total += drops.total;
        for (unsigned bus = 0; bus < ENCOS_CHANNEL_COUNT; ++bus)
            state_.channel_tx_drop[bus] += drops.channel[bus];
    }
    return true;
}
void MotorApplication::prepare_publication(Publication &snapshot)
{
    const uint32_t now = dcu_time_now_ms();
    snapshot.mode = state_.mode;
    snapshot.sequence = ++state_.publication_sequence;
    snapshot.accepted_cycle_sequence = state_.last_cycle;
    snapshot.cycle_sequence = state_.board_command.sequence;
    snapshot.board_status = state_.board_fault | (state_.config_ready ? 1 : 0) | (state_.operational ? 2 : 0) | (state_.owner == OWNER_SDO ? 128 : 0);
    snapshot.online_axes = snapshot.fault_axes = 0;
    snapshot.online_channels = snapshot.fault_channels = 0;
    uint32_t rx_drops_total = 0;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        auto *ch = &snapshot.channels[c];
        *ch = {};
        uint32_t drops = dcu_can_rx_drops(c) + raw_.rx_drops(c);
        rx_drops_total += drops;
        ch->status = (state_.config_ready ? 1 : 0) | (channel_active(c) ? 2 : 0) | (dcu_can_bus_state(c) == 3 ? 4 : 0) | (drops ? 8 : 0) | (state_.channel_tx_drop[c] ? 16 : 0);
        ch->rx_drops = drops;
        ch->tx_drops = state_.channel_tx_drop[c];
        ch->role = state_.channels[c].role;
        ch->raw_drops = raw_.rx_drops(c);
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++) {
            auto &axis = snapshot.axes[a];
            const auto status = axes_[a].read_feedback(now, state_.config.feedback_timeout_ms, axis.feedback, axis.diagnostics);
            axis.type1_count = axes_[a].type1_count();
            axes_[a].acknowledge_publication();
            if (status.online) {
                snapshot.online_axes |= 1u << a;
                ch->online_motors |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
            if (status.fault) {
                snapshot.fault_axes |= 1u << a;
                ch->fault_motors |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
        }
        if (ch->online_motors) {
            snapshot.online_channels |= 1u << c;
        }
        if (ch->fault_motors || (ch->status & 4)) {
            snapshot.fault_channels |= 1u << c;
        }
    }
    snapshot.rx_drops = rx_drops_total;
    snapshot.tx_drops = state_.tx_total;
    if (rx_drops_total) {
        state_.board_fault |= 16;
        snapshot.board_status |= 16;
    }
}
bool MotorApplication::configuration_writable() const { return !state_.operational && !any_active(); }
bool MotorApplication::owner_change_allowed() const { return !any_active() && !any_service(); }
bool MotorApplication::select_mode(OperatingMode mode)
{
    if (mode > OperatingMode::Raw ||
        (state_.operational && mode != state_.mode) ||
        (mode == OperatingMode::Raw && state_.config.configured_axes)) return false;
    state_.mode = mode;
    return true;
}
bool MotorApplication::take_raw_frame(unsigned bus, dcu_can_frame_t &frame) { return raw_.take(bus, frame); }
bool MotorApplication::read_axis(unsigned axis, AxisFeedback &feedback, AxisDiagnostics &diagnostics) const
{
    if (axis >= ENCOS_AXIS_COUNT) return false;
    axes_[axis].read_feedback(dcu_time_now_ms(), state_.config.feedback_timeout_ms, feedback, diagnostics);
    return true;
}
bool MotorApplication::set_owner(Owner owner)
{
    if (owner > OWNER_SDO || !owner_change_allowed()) return false;
    stop_all(0);
    state_.resume_pdo_owner = false;
    state_.owner = owner;
    return true;
}
CommandResult MotorApplication::commit_sdo_command(unsigned axis, const SdoCommand &command, uint32_t now)
{
    if (axis >= ENCOS_AXIS_COUNT) return {};
    CommandResult result{};
    result.accepted_sequence = state_.sdo_accepted_sequence[axis];
    if (command.sequence == result.accepted_sequence) {
        result.changed = false;
        return result;
    }
    bool gate = command_permitted(now);
    if (state_.owner != OWNER_SDO || !gate) {
        result.result = RESULT_STATE;
        axes_[axis].stop(8);
    } else if (!command.lease_ms || command.lease_ms > state_.config.max_command_lease_ms ||
               !newer(command.sequence, result.accepted_sequence)) {
        axes_[axis].stop(2);
    } else {
        bool ok = axes_[axis].accept(command.command, true, now);
        result.result = ok ? RESULT_OK : RESULT_INVALID;
        if (ok) {
            result.accepted_sequence = state_.sdo_accepted_sequence[axis] = command.sequence;
            result.mirror_command = axes_[axis].active();
            axes_[axis].set_sdo_lease(command.lease_ms, now);
        }
    }
    return result;
}
parameters::CommitOutcome MotorApplication::commit_parameter(const parameters::Request &request, uint32_t now, bool is_preop)
{
    const unsigned axis = request.axis;
    if (axis >= ENCOS_AXIS_COUNT) return {};
    bool permitted = is_preop && configuration_writable() && state_.owner == OWNER_SDO;
    return parameters::commit(request, now, permitted, channel_active(axis / ENCOS_MOTORS_PER_CHANNEL));
}
void MotorApplication::initialize(const Configuration &defaults)
{
    state_.config = defaults.board;
    state_.last_process_us = UINT32_MAX;
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        axes_[a].initialize(a);
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        state_.channels[c] = defaults.channels[c];
}
}
namespace {
// Static storage is intentional; the composed objects must not use the task stack.
dcu::app::motor_application::detail::MotorApplication application;
}

namespace dcu::app::motor_application {
void initialize(const Configuration &defaults) { application.initialize(defaults); }
ProcessResult process(const CycleCommand *candidate) { return application.process(candidate); }
void operational(bool operational) { application.operational(operational); }
void cycle_fault() { application.cycle_fault(); }
bool configuration_writable() { return application.configuration_writable(); }
bool owner_change_allowed() { return application.owner_change_allowed(); }
bool transaction_busy(unsigned axis)
{
    return axis < ENCOS_AXIS_COUNT && parameters::inspect(axis / ENCOS_MOTORS_PER_CHANNEL).busy;
}
bool apply_configuration(const Configuration &configuration) { return application.apply_configuration(configuration); }
bool select_mode(OperatingMode mode) { return application.select_mode(mode); }
OperatingMode mode() { return application.mode(); }
bool set_owner(Owner owner) { return application.set_owner(owner); }
Owner owner() { return application.owner(); }
CommandResult commit_sdo_command(unsigned axis, const SdoCommand &command, uint32_t now_ms)
{
    return application.commit_sdo_command(axis, command, now_ms);
}
parameters::CommitOutcome commit_parameter(const parameters::Request &request, uint32_t now_ms, bool preop)
{
    return application.commit_parameter(request, now_ms, preop);
}
bool read_axis(unsigned axis, AxisFeedback &feedback, AxisDiagnostics &diagnostics)
{
    return application.read_axis(axis, feedback, diagnostics);
}
void prepare_publication(Publication &snapshot) { application.prepare_publication(snapshot); }
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame) { return application.take_raw_frame(bus, frame); }
}
