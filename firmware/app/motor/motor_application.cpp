#include "motor_application.h"
#include "detail/motor_application_impl.hpp"
#include "encos_parameter_transactions.hpp"
#include "ethercat_canfd_gateway.h"
#include "motor_dictionary.hpp"
#include "motor_pdo_codec.hpp"
#include <cstring>

namespace dcu::app::motor_application::detail {
namespace motor_cfg = dcu::app::config::motor_control;
namespace pdo_layout = dcu::ethercat::pdo_layout;
namespace motor_pdo = dcu::ethercat::motor_pdo;
using namespace dcu::ethercat::motor_dictionary;

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
    return dcu::app::BusScheduler::frame_cost(state_.channels[c].FrameFormat != 0, bytes);
}
ControlSet MotorApplication::controls_on_bus(unsigned c) const
{
    dcu::app::ControlSet controls{};
    controls.period_us = state_.channel_command[c].Reserved;
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
        (state_.board_command.BoardControl & 1) && !(state_.board_command.BoardControl & 2) &&
        now - state_.last_command_ms < state_.board_command.CommandLeaseMs;
}
void MotorApplication::stop_all(uint16_t fault)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        axes_[a].stop(axes_[a].configured() ? fault : 0);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        ethercat_canfd_cancel(c);
    raw_.cancel();
    state_.session_armed = false;
}
void MotorApplication::operational(bool op)
{
    mc_owner_t *owner = object<mc_owner_t>(0x2003);
    if (!op) {
        /* Leaving OP ends this PDO session.  The idle EtherCAT master may
         * keep the slave in PREOP, but that must not keep CAN polling alive. */
        state_.resume_pdo_owner = state_.resume_pdo_owner || owner->Owner == OWNER_PDO;
        owner->Owner = OWNER_NONE;
        owner->RequestedOwner = OWNER_NONE;
        owner->Result = RESULT_OK;
    } else if (state_.resume_pdo_owner && owner->Owner == OWNER_NONE) {
        /* A transient OP loss in the same HMI session can recover without
         * repeating the PREOP SDO setup.  A new owner write cancels this. */
        owner->Owner = OWNER_PDO;
        owner->RequestedOwner = OWNER_PDO;
        state_.resume_pdo_owner = false;
    }
    state_.operational = op;
    state_.pending = false;
    state_.seen_cycle = false;
    raw_.reset_session();
    memset(&state_.board_command, 0, sizeof(state_.board_command));
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
     * Detailed cycle causes remain in g_dcu_cycle_diagnostics. */
    state_.board_fault |= 64U;
}
void MotorApplication::check_timeouts(uint32_t now)
{
    if (!state_.operational) return;
    if (state_.seen_cycle && uint32_t(now - state_.last_command_ms) >= state_.board_command.CommandLeaseMs) {
        state_.board_fault |= 4;
        stop_all(3);
    }
    const bool sdo_owner = object<mc_owner_t>(0x2003)->Owner == OWNER_SDO;
    for (auto &axis : axes_)
        axis.check_timeouts(now, sdo_owner, state_.config.FeedbackTimeoutMs);
}
void MotorApplication::output(const void *p)
{
    memcpy(state_.candidate, p, state_.output_size);
    state_.pending = true;
}
void MotorApplication::accept_candidate(uint32_t now)
{
    if (!state_.pending) {
        return;
    }
    state_.pending = false;
    auto b = motor_pdo::board(state_.candidate);
    /* Stop intent always wins, including duplicate sequence or malformed targets. */
    bool stop = !(b.BoardControl & 1) || (b.BoardControl & 2);
    if (stop)
        stop_all(0);
    if (state_.config.ActivePdoProfile != 2)
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
            auto c = motor_pdo::axis(state_.candidate, a);
            auto ch = motor_pdo::channel(state_.candidate, a / ENCOS_MOTORS_PER_CHANNEL);
            if (!(c.ControlWord & 1) || (c.ControlWord & 2) || !(ch.ChannelControl & 1) || (ch.ChannelControl & 2) || !(b.ChannelEnableMask & (1u << (a / ENCOS_MOTORS_PER_CHANNEL))) || !(b.AxisEnableMask & (1u << a)) || !(ch.ActiveMask & (1u << (a % ENCOS_MOTORS_PER_CHANNEL))))
                axes_[a].stop(0);
        }
    if (b.InterfaceVersion != MC_INTERFACE_VERSION || (b.BoardControl & ~7u) || (b.ChannelEnableMask & 0xF0) || (b.AxisEnableMask & 0xF0000000u) || b.Reserved0 || b.Reserved1 || b.Reserved2 || b.Reserved3 || b.Reserved4 || !b.CommandLeaseMs || b.CommandLeaseMs > state_.config.MaxCommandLeaseMs) {
        state_.board_fault |= 64;
        stop_all(9);
        return;
    }
    if (!state_.operational || (state_.seen_cycle && !newer(b.CycleSeq, state_.last_cycle)))
        return;
    if (!state_.session_armed && !stop) {
        stop_all(8);
        return;
    }
    if (stop && !b.AxisEnableMask)
        state_.session_armed = true;
    if ((b.BoardControl & 4) && !(state_.board_command.BoardControl & 4) && stop)
        state_.board_fault = 0;
    state_.last_cycle = b.CycleSeq;
    state_.seen_cycle = true;
    state_.last_command_ms = now;
    state_.board_command = b;
    *object<mc_board_rx_t>(0x7000) = b;
    object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq = b.CycleSeq;
    if (state_.config.ActivePdoProfile == 2) {
        raw_.stage_full(!stop);
        return;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        auto ch = motor_pdo::channel(state_.candidate, c);
        state_.channel_command[c] = ch;
        *object<mc_channel_rx_t>(0x7100 + c) = ch;
        bool valid = !(ch.ChannelControl & ~3u) && !(ch.ActiveMask & 0x80u) && ch.Reserved >= 1000u && ch.Reserved <= 10000u && ch.Reserved % 1000u == 0 && ch.ExpectedRole == state_.channels[c].Role;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++) {
            unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
            auto cmd = motor_pdo::axis(state_.candidate, a);
            bool permit = valid && !stop && (b.ChannelEnableMask & (1u << c)) && (b.AxisEnableMask & (1u << a)) && (ch.ChannelControl & 1) && !(ch.ChannelControl & 2) && (ch.ActiveMask & (1u << j));
            if (!valid) {
                axes_[a].stop(7);
                continue;
            }
            if (object<mc_owner_t>(0x2003)->Owner == OWNER_PDO)
                axes_[a].accept(cmd, permit, now);
        }
    }
    if (state_.config.ActivePdoProfile == 1) {
        mc_raw_t *r = object<mc_raw_t>(0x7300);
        *r = motor_pdo::raw(state_.candidate);
        raw_.stage_batch(r->BatchSeq, !stop);
    }
}
bool MotorApplication::apply_configuration()
{
    mc_board_config_t *b = object<mc_board_config_t>(0x2001);
    if (!configuration_writable() || any_service() || b->MaxCommandLeaseMs < 2 || b->MaxCommandLeaseMs > 10 || b->FeedbackTimeoutMs < 5 || b->FeedbackTimeoutMs > 20)
        return false;
    uint32_t mask = 0;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        mc_channel_config_t *ch = object<mc_channel_config_t>(0x2010 + c);
        if (ch->Role > 3 || ch->FrameFormat > 1 || ch->NominalBitrate != 1000000 || ch->DataBitrate != 5000000 || ch->ScheduleUs != 2000 || ch->RawRateLimit > 8)
            return false;
        if ((ch->Role == 0 || ch->Role == 2) && ch->MotorConfiguredMask)
            return false;
        if (state_.config.ActivePdoProfile == 2 && ch->MotorConfiguredMask)
            return false;
        if (ch->MotorConfiguredMask & 0x80u)
            return false;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++)
            if (ch->MotorConfiguredMask & (1u << j)) {
                unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
                mc_routing_t *r = object<mc_routing_t>(axis_object_base(a));
                mc_basic_t *basic = object<mc_basic_t>(axis_object_base(a) + 1);
                if (!dcu::app::validate_motor_configuration(*r, *basic, *object<mc_range_t>(axis_object_base(a) + 2), ch->FrameFormat))
                    return false;
                for (unsigned previous = 0; previous < j; ++previous)
                    if ((ch->MotorConfiguredMask & (1u << previous)) &&
                        object<mc_routing_t>(axis_object_base(c * ENCOS_MOTORS_PER_CHANNEL + previous))->CanId == r->CanId)
                        return false;
                mask |= 1u << a;
            }
    }
    if (mask != b->AxisConfiguredMask)
        return false;
    stop_all(0);
    state_.config = *b;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        state_.channels[c] = *object<mc_channel_config_t>(0x2010 + c);
        schedule_[c].reset_budget(ethercat_canfd_now_ms());
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        if (!axes_[a].configure((mask & (1u << a)) != 0,
                                state_.channels[a / ENCOS_MOTORS_PER_CHANNEL].FrameFormat)) {
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
    if ((f->flags & 2) || f->flags != (object<mc_channel_config_t>(0x2010 + c)->FrameFormat ? 24 : 0) || f->dlc < 2)
        return;
    auto transaction = parameters::inspect(c);
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
    axes_[a].consume_feedback(frame, stamp, now);
}
void MotorApplication::process()
{
    uint32_t now_us = ethercat_canfd_now_us();
    if ((uint32_t)(now_us - state_.last_process_us) < motor_cfg::process_period_us)
        return;
    state_.last_process_us = now_us;
    uint32_t now = ethercat_canfd_now_ms();
    accept_candidate(now);
    check_timeouts(now);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        schedule_[c].refill(now);
        dcu_can_frame_t f;
        uint32_t stamp;
        for (unsigned n = 0; n < 32 && ethercat_canfd_receive(c, &f, &stamp); n++) {
            consume_feedback(c, f, stamp, now);
            if (state_.channels[c].Role >= 2)
                raw_.receive(c, f);
        }
        if (ethercat_canfd_bus_state(c) == 3) {
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
        if (schedule_[c].stalled(now, ethercat_canfd_tx_progress(c),
                                 ethercat_canfd_pending(c), state_.config.FeedbackTimeoutMs)) {
            /* 只把“队列持续无任何发送完成”视为真正的 TX 卡死。
             * 正常多轴流水发送时 TXBRP 可以跨调度周期保持非零。 */
            ethercat_canfd_cancel(c);
            ++state_.channel_tx_drop[c];
            ++state_.tx_total;
            state_.board_fault |= 32;
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (axes_[a].active()) {
                    ++axis_diagnostics(a)->TxDropCount;
                    axes_[a].stop(6);
                }
            continue;
        }
        if (state_.operational && state_.session_armed && state_.config.ActivePdoProfile != 2) {
            unsigned cost = 2u * frame_us(c, 8);
            bool ready = !ethercat_canfd_pending(c) && ethercat_canfd_tx_free(c) > 0;
            int selected = schedule_[c].next_control(controls, now_us, cost, ready);
            if (selected >= 0) {
                unsigned slot = static_cast<unsigned>(selected);
                unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                auto &axis = axes_[a];
                if (axis.encode_control(f)) {
                    f.flags = state_.channels[c].FrameFormat ? 24 : 0;
                    if (ethercat_canfd_send(c, &f, a)) {
                        schedule_[c].control_submitted(slot, now_us, controls.period_us, !axis.control_sent(), cost);
                        axis.control_submitted(now);
                        controls.first_send_mask &= ~(1u << slot);
                    } else {
                        ++axis_diagnostics(a)->TxDropCount;
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
        if (request && !ethercat_canfd_pending(c) && ethercat_canfd_tx_free(c) &&
            schedule_[c].guard_elapsed(now_us)) {
            unsigned cost = frame_us(c, request->dlc) + frame_us(c, 8);
            bool query_window = !transaction.internal || schedule_[c].query_window(controls, now_us, cost, 2u * frame_us(c, 8));
            if (query_window && schedule_[c].can_afford(cost) && ethercat_canfd_send(c, request, ENCOS_AXIS_COUNT)) {
                schedule_[c].submitted(now_us, cost);
                parameters::mark_sent(c, now);
            }
        }
        unsigned owner = object<mc_owner_t>(0x2003)->Owner;
        if (!transaction.busy && (owner == OWNER_PDO || owner == OWNER_SDO) && state_.config.ActivePdoProfile != 2 && state_.channels[c].Role != 0) {
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
                        if (axis.start_query(now, owner, state_.channels[c].FrameFormat))
                            state_.query_slot[c] = (slot + 1) % ENCOS_MOTORS_PER_CHANNEL;
                        if (parameters::inspect(c).busy)
                            break;
                    }
                }
            }
        }
    }
    RawCanCycle cycle{state_.config.ActivePdoProfile, state_.board_command.ChannelEnableMask,
                      command_permitted(now), state_.candidate};
    const auto drops = raw_.process(cycle, now, state_.channels, axes_, schedule_);
    state_.tx_total += drops.total;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c)
        state_.channel_tx_drop[c] += drops.channel[c];
}
void MotorApplication::publish(uint32_t now)
{
    mc_board_tx_t *b = object<mc_board_tx_t>(0x6000);
    ++b->PublishSeq;
    b->BoardStatus = state_.board_fault | (state_.config_ready ? 1 : 0) | (state_.operational ? 2 : 0) | (object<mc_owner_t>(0x2003)->Owner == OWNER_SDO ? 128 : 0);
    b->AxisOnlineMask = b->AxisFaultMask = 0;
    b->ChannelOnlineMask = b->ChannelFaultMask = 0;
    uint32_t rx_drops_total = 0;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        mc_channel_tx_t *ch = object<mc_channel_tx_t>(0x6100 + c);
        memset(ch, 0, sizeof(*ch));
        uint32_t drops = ethercat_canfd_rx_drops(c) + raw_.rx_drops(c);
        rx_drops_total += drops;
        ch->ChannelStatus = (state_.config_ready ? 1 : 0) | (channel_active(c) ? 2 : 0) | (ethercat_canfd_bus_state(c) == 3 ? 4 : 0) | (drops ? 8 : 0) | (state_.channel_tx_drop[c] ? 16 : 0);
        ch->RxDropCount = drops > 65535 ? 65535 : drops;
        ch->TxDropCount = state_.channel_tx_drop[c] > 65535 ? 65535 : state_.channel_tx_drop[c];
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++) {
            const auto status = axes_[a].publish(now, state_.config.FeedbackTimeoutMs);
            if (status.online) {
                b->AxisOnlineMask |= 1u << a;
                ch->OnlineMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
            if (status.fault) {
                b->AxisFaultMask |= 1u << a;
                ch->FaultMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
        }
        if (ch->OnlineMask) {
            b->ChannelOnlineMask |= 1u << c;
        }
        if (ch->FaultMask || (ch->ChannelStatus & 4)) {
            b->ChannelFaultMask |= 1u << c;
        }
    }
    b->RxDropCount = rx_drops_total;
    b->TxDropCount = state_.tx_total;
    if (rx_drops_total) {
        state_.board_fault |= 16;
        b->BoardStatus |= 16;
    }
}
bool MotorApplication::configuration_writable() const { return !state_.operational && !any_active(); }
bool MotorApplication::owner_change_allowed() const { return !any_active() && !any_service(); }
bool MotorApplication::select_profile(unsigned profile)
{
    if (profile >= pdo_layout::profile_count ||
        (state_.operational && profile != state_.config.ActivePdoProfile) ||
        (profile == 2 && state_.config.AxisConfiguredMask)) return false;
    state_.config.ActivePdoProfile = profile;
    state_.output_size = pdo_layout::profiles[profile].output;
    return true;
}
PdoPublication MotorApplication::prepare_publication()
{
    publish(ethercat_canfd_now_ms());
    PdoPublication snapshot{};
    snapshot.profile = state_.config.ActivePdoProfile;
    snapshot.cycle_sequence = state_.board_command.CycleSeq;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        snapshot.roles[c] = state_.channels[c].Role;
        snapshot.raw_drops[c] = raw_.rx_drops(c);
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
        snapshot.type1_count[a] = axes_[a].type1_count();
    return snapshot;
}
bool MotorApplication::take_raw_frame(unsigned bus, dcu_can_frame_t &frame) { return raw_.take(bus, frame); }
void MotorApplication::owner_changed() { stop_all(0); state_.resume_pdo_owner = false; }
void MotorApplication::commit_sdo_command(unsigned axis, mc_sdo_command_t &command, uint32_t now)
{
    if (axis >= ENCOS_AXIS_COUNT) {
        command.CommandResult = RESULT_INVALID;
        return;
    }
    bool gate = command_permitted(now);
    if (object<mc_owner_t>(0x2003)->Owner != OWNER_SDO || !gate) {
        command.CommandResult = RESULT_STATE;
        axes_[axis].stop(8);
    } else if (!command.LeaseMs || command.LeaseMs > state_.config.MaxCommandLeaseMs ||
               !newer(command.CommitSeq, command.AcceptedSeq)) {
        command.CommandResult = RESULT_INVALID;
        axes_[axis].stop(2);
    } else {
        mc_axis_rx_t intent;
        memcpy(&intent, &command, sizeof(intent));
        bool ok = axes_[axis].accept(intent, true, now);
        command.CommandResult = ok ? RESULT_OK : RESULT_INVALID;
        if (ok) {
            command.AcceptedSeq = command.CommitSeq;
            axes_[axis].set_sdo_lease(command.LeaseMs, now);
        }
    }
}
bool MotorApplication::commit_parameter(unsigned axis, bool maintenance, uint32_t now, bool is_preop)
{
    if (axis >= ENCOS_AXIS_COUNT) return false;
    bool permitted = is_preop && configuration_writable() && object<mc_owner_t>(0x2003)->Owner == OWNER_SDO;
    return parameters::commit(axis, maintenance, now, permitted, channel_active(axis / ENCOS_MOTORS_PER_CHANNEL));
}
void MotorApplication::initialize()
{
    state_.config = *object<mc_board_config_t>(0x2001);
    state_.output_size = pdo_layout::max_output_bytes;
    state_.last_process_us = UINT32_MAX;
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        axes_[a].initialize(a);
        object<mc_routing_t>(axis_object_base(a))->CanId = a % ENCOS_MOTORS_PER_CHANNEL + 1;
        axis_feedback(a)->FeedbackAgeMs = axis_feedback(a)->PowerAgeMs = UINT16_MAX;
        axis_diagnostics(a)->PositionQueryAgeUs = axis_diagnostics(a)->VelocityQueryAgeUs = UINT32_MAX;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        state_.channels[c] = *object<mc_channel_config_t>(0x2010 + c);
    object<mc_board_info_t>(0x2000)->Capabilities = 7; /* Three PDO profiles; hardware validation still required. */
}
}
namespace {
// Static storage is intentional; the composed objects must not use the task stack.
dcu::app::motor_application::detail::MotorApplication application;
}

void motor_control_init(void) { application.initialize(); }
void motor_control_process(void) { application.process(); }
void motor_control_operational(bool operational) { application.operational(operational); }
void motor_control_cycle_fault(void) { application.cycle_fault(); }
void motor_control_output(const void *data) { application.output(data); }

namespace dcu::app::motor_application {
bool configuration_writable() { return application.configuration_writable(); }
bool owner_change_allowed() { return application.owner_change_allowed(); }
bool transaction_busy(unsigned axis)
{
    return axis < ENCOS_AXIS_COUNT && parameters::inspect(axis / ENCOS_MOTORS_PER_CHANNEL).busy;
}
bool apply_configuration() { return application.apply_configuration(); }
bool select_profile(unsigned profile) { return application.select_profile(profile); }
void owner_changed() { application.owner_changed(); }
void commit_sdo_command(unsigned axis, mc_sdo_command_t &command, uint32_t now_ms)
{
    application.commit_sdo_command(axis, command, now_ms);
}
bool commit_parameter(unsigned axis, bool maintenance, uint32_t now_ms, bool preop)
{
    return application.commit_parameter(axis, maintenance, now_ms, preop);
}
PdoPublication prepare_publication() { return application.prepare_publication(); }
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame) { return application.take_raw_frame(bus, frame); }
}
