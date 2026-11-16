#include "motor_application.h"
#include "detail/motor_application_state.hpp"
#include "motor_application.hpp"
#include "encos_parameter_transactions.hpp"
#include "can_bus_scheduler.hpp"
#include "motor_dictionary.hpp"
#include "cfg/motor_policy.hpp"
#include "encos_motor.h"
#include "ethercat_canfd_gateway.h"
#include "motor_driver_binding.hpp"
#include "motor_pdo_codec.hpp"
#include <limits.h>
#include <math.h>
#include <string.h>

using namespace dcu::app::motor_application;

namespace motor_cfg = dcu::app::config::motor_control;
namespace pdo_layout = dcu::ethercat::pdo_layout;
namespace motor_pdo = dcu::ethercat::motor_pdo;
namespace parameters = dcu::app::parameters;
using dcu::app::motor_application::detail::axis_t;
using dcu::app::motor_application::detail::raw_queue_t;
using dcu::app::motor_application::detail::MotorControlState;

// Static lifetime is intentional; this state must not consume the task stack.
static MotorControlState state;

static bool newer(uint32_t a, uint32_t b)
{
    uint32_t d = a - b;
    return d && d < 0x80000000u;
}
static uint32_t age(uint32_t now, uint32_t then, bool seen)
{
    uint32_t d = now - then;
    return !seen || d > UINT32_MAX / 1000 ? UINT32_MAX : d * 1000;
}
static uint16_t age_ms(uint32_t now, uint32_t then, bool seen)
{
    uint32_t d = now - then;
    return !seen || d > UINT16_MAX ? UINT16_MAX : (uint16_t)d;
}
static unsigned frame_us(unsigned c, unsigned bytes)
{
    return dcu::app::BusScheduler::frame_cost(state.channels[c].FrameFormat != 0, bytes);
}
static dcu::app::ControlSet controls_on_bus(unsigned c)
{
    dcu::app::ControlSet controls{};
    controls.period_us = state.channel_command[c].Reserved;
    for (unsigned slot = 0; slot < ENCOS_MOTORS_PER_CHANNEL; ++slot) {
        auto &axis = state.axes[c * ENCOS_MOTORS_PER_CHANNEL + slot];
        if (!axis.active) continue;
        controls.active_mask |= 1u << slot;
        if (!axis.control_sent) controls.first_send_mask |= 1u << slot;
    }
    return controls;
}
using namespace dcu::ethercat::motor_dictionary;
static bool any_active(void)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        if (state.axes[a].active)
            return true;
    return false;
}
static bool channel_active(unsigned c)
{
    for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
        if (state.axes[a].active)
            return true;
    return false;
}
static bool any_service(void)
{
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        if (parameters::inspect(c).busy)
            return true;
    return false;
}
static void stop_axis(unsigned a, uint16_t fault)
{
    axis_t *x = &state.axes[a];
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
static void stop_all(uint16_t fault)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        stop_axis(a, state.axes[a].configured ? fault : 0);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        ethercat_canfd_cancel(c);
    state.raw_pending = false;
    state.session_armed = false;
}
void motor_control_operational(bool op)
{
    mc_owner_t *owner = object<mc_owner_t>(0x2003);
    if (!op) {
        /* Leaving OP ends this PDO session.  The idle EtherCAT master may
         * keep the slave in PREOP, but that must not keep CAN polling alive. */
        state.resume_pdo_owner = state.resume_pdo_owner || owner->Owner == OWNER_PDO;
        owner->Owner = OWNER_NONE;
        owner->RequestedOwner = OWNER_NONE;
        owner->Result = RESULT_OK;
    } else if (state.resume_pdo_owner && owner->Owner == OWNER_NONE) {
        /* A transient OP loss in the same HMI session can recover without
         * repeating the PREOP SDO setup.  A new owner write cancels this. */
        owner->Owner = OWNER_PDO;
        owner->RequestedOwner = OWNER_PDO;
        state.resume_pdo_owner = false;
    }
    state.operational = op;
    state.pending = false;
    state.seen_cycle = false;
    state.raw_seen = false;
    memset(state.full_seen, 0, sizeof(state.full_seen));
    memset(&state.board_command, 0, sizeof(state.board_command));
    stop_all(0);
    if (!op)
        parameters::reset_session(RESULT_STATE);
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        memset(&state.axes[a].command, 0, sizeof(state.axes[a].command));
        state.axes[a].previous_control = 0;
    }
}
void motor_control_cycle_fault(void)
{
    motor_control_operational(false);
    /* Keep the existing invalid-PDO bit understood by the master's 0xfc mask.
     * Detailed cycle causes remain in g_dcu_cycle_diagnostics. */
    state.board_fault |= 64U;
}
static void check_timeouts(uint32_t now)
{
    if (!state.operational)
        return;
    if (state.seen_cycle && (uint32_t)(now - state.last_command_ms) >= state.board_command.CommandLeaseMs) {
        state.board_fault |= 4;
        stop_all(3);
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        if (state.axes[a].active || state.axes[a].query_armed) {
            axis_t *x = &state.axes[a];
            if (object<mc_owner_t>(0x2003)->Owner == OWNER_SDO && (uint32_t)(now - x->commit_ms) >= x->lease_ms)
                stop_axis(a, 3);
            /* A previous session's Type-1 timestamp must not expire a new session.
             * Start the first-feedback deadline only after its first control frame
             * has actually entered the CAN transmitter. */
            if (x->control_sent && (uint32_t)(now - (x->feedback_seen ? x->feedback_ms : x->active_ms)) >= state.config.FeedbackTimeoutMs)
                stop_axis(a, 4);
        }
}
static bool query_ready(unsigned a, uint32_t now)
{
    axis_t *x = &state.axes[a];
    return x->configured && x->online_seen && now - x->online_ms <= motor_cfg::query_online_ms && x->position_seen && now - x->position_ms <= motor_cfg::query_position_ms && x->velocity_seen && now - x->velocity_ms <= motor_cfg::query_velocity_ms && x->current_seen && now - x->current_ms <= motor_cfg::query_current_ms && fabs((double)axis_feedback(a)->ActualCurrent) <= x->route.CurrentLimit * 10.0 && !axis_feedback(a)->MotorFaultRaw && !axis_feedback(a)->BridgeFaultCode && ethercat_canfd_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3;
}
static bool accept_axis(unsigned a, const mc_axis_rx_t *command, bool permit, uint32_t now)
{
    axis_t *x = &state.axes[a];
    mc_axis_tx_t *s = axis_feedback(a);
    uint16_t prev = x->previous_control;
    x->previous_control = command->ControlWord;
    if (!permit || (command->ControlWord & MC_AXIS_STOP) || !(command->ControlWord & MC_AXIS_ARM)) {
        stop_axis(a, 0);
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
        stop_axis(a, 2);
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
            stop_axis(a, 0);
            x->query_armed = true;
        }
        memset(&x->command, 0, sizeof(x->command));
        return true;
    }
    encos_frame_t f;
    if (!x->configured || !dcu::app::encode_motor_command(x->device, *command, f)) {
        stop_axis(a, x->configured ? 2 : 1);
        s->StatusWord |= 1u << 8;
        return false;
    }
    if (x->active && x->command.Mode != command->Mode) {
        stop_axis(a, 8);
        s->StatusWord |= 1u << 8;
        return false;
    }
    if (!x->active) {
        /* StartMotion needs a fresh edge after a separate query-only cycle. */
        if ((prev & MC_AXIS_START_MOTION) || !query_ready(a, now) ||
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
void motor_control_output(const void *p)
{
    memcpy(state.candidate, p, state.output_size);
    state.pending = true;
}
static void accept_candidate(uint32_t now)
{
    if (!state.pending) {
        return;
    }
    state.pending = false;
    auto b = motor_pdo::board(state.candidate);
    /* Stop intent always wins, including duplicate sequence or malformed targets. */
    bool stop = !(b.BoardControl & 1) || (b.BoardControl & 2);
    if (stop)
        stop_all(0);
    if (state.config.ActivePdoProfile != 2)
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
            auto c = motor_pdo::axis(state.candidate, a);
            auto ch = motor_pdo::channel(state.candidate, a / ENCOS_MOTORS_PER_CHANNEL);
            if (!(c.ControlWord & 1) || (c.ControlWord & 2) || !(ch.ChannelControl & 1) || (ch.ChannelControl & 2) || !(b.ChannelEnableMask & (1u << (a / ENCOS_MOTORS_PER_CHANNEL))) || !(b.AxisEnableMask & (1u << a)) || !(ch.ActiveMask & (1u << (a % ENCOS_MOTORS_PER_CHANNEL))))
                stop_axis(a, 0);
        }
    if (b.InterfaceVersion != MC_INTERFACE_VERSION || (b.BoardControl & ~7u) || (b.ChannelEnableMask & 0xF0) || (b.AxisEnableMask & 0xF0000000u) || b.Reserved0 || b.Reserved1 || b.Reserved2 || b.Reserved3 || b.Reserved4 || !b.CommandLeaseMs || b.CommandLeaseMs > state.config.MaxCommandLeaseMs) {
        state.board_fault |= 64;
        stop_all(9);
        return;
    }
    if (!state.operational || (state.seen_cycle && !newer(b.CycleSeq, state.last_cycle)))
        return;
    if (!state.session_armed && !stop) {
        stop_all(8);
        return;
    }
    if (stop && !b.AxisEnableMask)
        state.session_armed = true;
    if ((b.BoardControl & 4) && !(state.board_command.BoardControl & 4) && stop)
        state.board_fault = 0;
    state.last_cycle = b.CycleSeq;
    state.seen_cycle = true;
    state.last_command_ms = now;
    state.board_command = b;
    *object<mc_board_rx_t>(0x7000) = b;
    object<mc_board_tx_t>(0x6000)->AcceptedCycleSeq = b.CycleSeq;
    if (state.config.ActivePdoProfile == 2) {
        state.raw_pending = !stop;
        return;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        auto ch = motor_pdo::channel(state.candidate, c);
        state.channel_command[c] = ch;
        *object<mc_channel_rx_t>(0x7100 + c) = ch;
        bool valid = !(ch.ChannelControl & ~3u) && !(ch.ActiveMask & 0x80u) && ch.Reserved >= 1000u && ch.Reserved <= 10000u && ch.Reserved % 1000u == 0 && ch.ExpectedRole == state.channels[c].Role;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++) {
            unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
            auto cmd = motor_pdo::axis(state.candidate, a);
            bool permit = valid && !stop && (b.ChannelEnableMask & (1u << c)) && (b.AxisEnableMask & (1u << a)) && (ch.ChannelControl & 1) && !(ch.ChannelControl & 2) && (ch.ActiveMask & (1u << j));
            if (!valid) {
                stop_axis(a, 7);
                continue;
            }
            if (object<mc_owner_t>(0x2003)->Owner == OWNER_PDO)
                accept_axis(a, &cmd, permit, now);
        }
    }
    if (state.config.ActivePdoProfile == 1) {
        mc_raw_t *r = object<mc_raw_t>(0x7300);
        *r = motor_pdo::raw(state.candidate);
        if (!state.raw_seen || newer(r->BatchSeq, state.last_raw_batch)) {
            state.last_raw_batch = r->BatchSeq;
            state.raw_seen = true;
            state.raw_pending = !stop;
        }
    }
}
static bool apply_config(void)
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
        if (state.config.ActivePdoProfile == 2 && ch->MotorConfiguredMask)
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
    state.config = *b;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        state.channels[c] = *object<mc_channel_config_t>(0x2010 + c);
        state.schedule[c].reset_budget(ethercat_canfd_now_ms());
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        axis_t *x = &state.axes[a];
        *x = {};
        memset(axis_feedback(a), 0, sizeof(mc_axis_tx_t));
        x->configured = (mask & (1u << a)) != 0;
        x->route = *object<mc_routing_t>(axis_object_base(a));
        x->basic = *object<mc_basic_t>(axis_object_base(a) + 1);
        x->range = *object<mc_range_t>(axis_object_base(a) + 2);
        if (x->configured) {
            const auto binding = dcu::app::configure_motor_driver(
                a, x->route, x->basic, x->range, state.channels[a / ENCOS_MOTORS_PER_CHANNEL].FrameFormat);
            if (!binding.driver) {
                stop_all(1);
                state.config_ready = false;
                for (auto &axis : state.axes) {
                    axis.configured = false;
                    axis.device.reset();
                }
                return false;
            }
            x->device.bind(*binding.driver);
            x->parameter_dialect = binding.parameter_dialect;
        }
    }
    state.config_ready = true;
    return true;
}
static void consume_feedback(unsigned c, const encos_frame_t *f, uint32_t stamp, uint32_t now)
{
    if ((f->flags & 2) || f->flags != (object<mc_channel_config_t>(0x2010 + c)->FrameFormat ? 24 : 0) || f->dlc < 2)
        return;
    auto transaction = parameters::inspect(c);
    unsigned a = ENCOS_AXIS_COUNT;
    if (parameters::consume_maintenance(c, *f))
        return;
    for (unsigned j = c * ENCOS_MOTORS_PER_CHANNEL; j < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; j++)
        if (state.axes[j].configured && state.axes[j].route.CanId == f->id) {
            a = j;
            break;
        }
    if (transaction.busy && transaction.can_id == f->id)
        a = transaction.axis;
    if (a == ENCOS_AXIS_COUNT)
        return;
    axis_t *x = &state.axes[a];
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
            stop_axis(a, 2);
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
                stop_axis(a, 0);
        }
    } else
        axis_diagnostics(a)->ProtocolErrorCount++;
}
static void raw_push(unsigned c, const encos_frame_t *f)
{
    raw_queue_t *q = &state.raw_rx[c];
    uint8_t next = (q->write + 1) & (motor_cfg::raw_queue_capacity - 1);
    if (next == q->read) {
        ++q->drops;
        return;
    }
    q->frames[q->write] = *f;
    q->write = next;
}
static bool raw_pop(unsigned c, encos_frame_t *f)
{
    raw_queue_t *q = &state.raw_rx[c];
    if (q->read == q->write)
        return false;
    *f = q->frames[q->read];
    q->read = (q->read + 1) & (motor_cfg::raw_queue_capacity - 1);
    return true;
}
static bool raw_send(unsigned c, const encos_frame_t *f, uint32_t now)
{
    if (c >= ENCOS_CHANNEL_COUNT || !(state.board_command.ChannelEnableMask & (1u << c)) || (state.channels[c].Role != 2 && state.channels[c].Role != 3) || parameters::inspect(c).busy || (int32_t)(now - parameters::inspect(c).hold_until) < 0)
        return false;
    if (state.config.ActivePdoProfile != 2) {
        if (f->id == 0x7FF)
            return false;
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            if (state.axes[a].configured && f->id == state.axes[a].route.CanId)
                return false;
    }
    if (now - state.raw_budget_ms[c] >= 2) {
        state.raw_budget_ms[c] = now;
        state.raw_budget[c] = 0;
    }
    if (state.raw_budget[c] >= state.channels[c].RawRateLimit) {
        return false;
    }
    unsigned cost = ((f->flags & 8u) ? 48u + 2u * f->dlc : 50u + 11u * f->dlc) + frame_us(c, 8);
    if (!state.schedule[c].can_afford(cost) || !ethercat_canfd_send(c, f, ENCOS_AXIS_COUNT))
        return false;
    state.schedule[c].charge(cost);
    ++state.raw_budget[c];
    return true;
}
static void process_raw(uint32_t now)
{
    if (!state.raw_pending) {
        return;
    }
    state.raw_pending = false;
    if (!state.operational || !state.session_armed || !(state.board_command.BoardControl & 1) || (state.board_command.BoardControl & 2) || now - state.last_command_ms >= state.board_command.CommandLeaseMs)
        return;
    if (state.config.ActivePdoProfile == 1) {
        mc_raw_t *r = object<mc_raw_t>(0x7300);
        if ((r->ValidMask & ~3u) || r->SlotCount > 2 || r->Flags || r->DropCount || r->Reserved || r->Reserved1 || r->Reserved2)
            return;
        for (unsigned i = 0; i < 2; i++)
            if (r->ValidMask & (1u << i)) {
                auto slot = motor_pdo::raw_slot(*r, i);
                encos_frame_t f{};
                f.id = slot.id;
                f.flags = slot.flags;
                f.dlc = slot.dlc;
                memcpy(f.data, slot.data, sizeof(f.data));
                if (!raw_send(slot.channel, &f, now)) {
                    ++state.tx_total;
                    if (slot.channel < ENCOS_CHANNEL_COUNT)
                        ++state.channel_tx_drop[slot.channel];
                }
            }
    } else if (state.config.ActivePdoProfile == 2) {
        for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
            auto raw = motor_pdo::raw_bus(state.candidate, c);
            const auto *p = &raw;
            *object<canfd_bus_pdo_t>(0x7400 + 16 * c) = raw;
            if (state.full_seen[c] && (uint16_t)(p->cycle_seq - state.last_full_cycle[c]) == 0)
                continue;
            state.last_full_cycle[c] = p->cycle_seq;
            state.full_seen[c] = true;
            if (p->valid_mask & 0x80u)
                continue;
            for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS; i++)
                if ((p->valid_mask & (1u << i)) && (p->slot[i].flags & 1)) {
                    const canfd_slot_pdo_t *s = &p->slot[i];
                    encos_frame_t f{};
                    f.id = s->can_id;
                    f.dlc = s->dlc;
                    f.flags = static_cast<uint8_t>(s->flags & ~33u);
                    memcpy(f.data, s->data, 8);
                    if (!raw_send(c, &f, now)) {
                        ++state.tx_total;
                        ++state.channel_tx_drop[c];
                    }
                }
        }
    }
}
void motor_control_process(void)
{
    uint32_t now_us = ethercat_canfd_now_us();
    if ((uint32_t)(now_us - state.last_process_us) < motor_cfg::process_period_us)
        return;
    state.last_process_us = now_us;
    uint32_t now = ethercat_canfd_now_ms();
    accept_candidate(now);
    check_timeouts(now);
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        state.schedule[c].refill(now);
        encos_frame_t f;
        uint32_t stamp;
        for (unsigned n = 0; n < 32 && ethercat_canfd_receive(c, &f, &stamp); n++) {
            consume_feedback(c, &f, stamp, now);
            if (state.channels[c].Role >= 2)
                raw_push(c, &f);
        }
        auto transaction = parameters::inspect(c);
        if (ethercat_canfd_bus_state(c) == 3) {
            state.board_fault |= 8;
            if (transaction.busy) {
                ethercat_canfd_cancel_service(c);
                parameters::cancel(c, RESULT_IO);
            }
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (state.axes[a].configured)
                    stop_axis(a, 5);
            continue;
        }
        if (transaction.busy && now - transaction.started >= motor_cfg::parameter_timeout_ms) {
            ethercat_canfd_cancel_service(c);
            parameters::cancel(c, RESULT_TIMEOUT);
            transaction = parameters::inspect(c);
        }
        /* Do not queue a delayed start after its query snapshot has expired. */
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            if (state.axes[a].active && !state.axes[a].control_sent && !query_ready(a, now))
                stop_axis(a, 8);
        auto controls = controls_on_bus(c);
        if (state.schedule[c].stalled(now, ethercat_canfd_tx_progress(c),
                                      ethercat_canfd_pending(c), state.config.FeedbackTimeoutMs)) {
            /* 只把“队列持续无任何发送完成”视为真正的 TX 卡死。
             * 正常多轴流水发送时 TXBRP 可以跨调度周期保持非零。 */
            ethercat_canfd_cancel(c);
            ++state.channel_tx_drop[c];
            ++state.tx_total;
            state.board_fault |= 32;
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (state.axes[a].active) {
                    ++axis_diagnostics(a)->TxDropCount;
                    stop_axis(a, 6);
                }
            continue;
        }
        if (state.operational && state.session_armed && state.config.ActivePdoProfile != 2) {
            unsigned cost = 2u * frame_us(c, 8);
            bool ready = !ethercat_canfd_pending(c) && ethercat_canfd_tx_free(c) > 0;
            int selected = state.schedule[c].next_control(controls, now_us, cost, ready);
            if (selected >= 0) {
                unsigned slot = static_cast<unsigned>(selected);
                unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                auto *x = &state.axes[a];
                if (dcu::app::encode_motor_command(x->device,
                                                  x->feedback_seen ? x->command : x->startup_command, f)) {
                    f.flags = state.channels[c].FrameFormat ? 24 : 0;
                    if (ethercat_canfd_send(c, &f, a)) {
                        state.schedule[c].control_submitted(slot, now_us, controls.period_us, !x->control_sent, cost);
                        if (!x->control_sent) {
                            x->control_sent = true;
                            x->active_ms = now;
                            controls.first_send_mask &= ~(1u << slot);
                        }
                        axis_feedback(a)->ModeDisplay = x->feedback_seen ? x->command.Mode : 1;
                        ++axis_diagnostics(a)->TxEnqueueCount;
                    } else {
                        ++axis_diagnostics(a)->TxDropCount;
                        ++state.channel_tx_drop[c];
                        ++state.tx_total;
                    }
                } else {
                    stop_axis(a, 2);
                    controls.active_mask &= ~(1u << slot);
                }
            }
        }
        const auto *request = parameters::pending(c);
        if (request && !ethercat_canfd_pending(c) && ethercat_canfd_tx_free(c) &&
            state.schedule[c].guard_elapsed(now_us)) {
            unsigned cost = frame_us(c, request->dlc) + frame_us(c, 8);
            bool query_window = !transaction.internal || state.schedule[c].query_window(controls, now_us, cost, 2u * frame_us(c, 8));
            if (query_window && state.schedule[c].can_afford(cost) && ethercat_canfd_send(c, request, ENCOS_AXIS_COUNT)) {
                state.schedule[c].submitted(now_us, cost);
                parameters::mark_sent(c, now);
            }
        }
        unsigned owner = object<mc_owner_t>(0x2003)->Owner;
        if (!transaction.busy && (owner == OWNER_PDO || owner == OWNER_SDO) && state.config.ActivePdoProfile != 2 && state.channels[c].Role != 0) {
            /* Internal queries are strictly best effort.  Reserve one complete
             * control sweep and enough wall-clock slack before creating the
             * service, otherwise the query waits without stealing a deadline.
             * Query-only axes take priority; active axes need only code 4
             * because Type-1 already carries position, velocity, current,
             * temperatures and the motor fault. */
            unsigned query_cost = frame_us(c, 2) + frame_us(c, 8);
            if (state.schedule[c].query_window(controls, now_us, query_cost, 2u * frame_us(c, 8))) {
                for (unsigned pass = 0; pass < 2 && !parameters::inspect(c).busy; pass++) {
                    for (unsigned n = 0; n < ENCOS_MOTORS_PER_CHANNEL; n++) {
                        unsigned slot = (state.query_slot[c] + n) % ENCOS_MOTORS_PER_CHANNEL;
                        unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                        axis_t *x = &state.axes[a];
                        if (!x->configured || (pass == 0 ? x->active : !x->active))
                            continue;
                        if (!x->active && owner == OWNER_SDO && !x->query_armed)
                            continue;
                        if (x->active && (owner != OWNER_PDO || !x->route.PowerQueryPeriodMs))
                            continue;
                        unsigned first = x->active ? 3u : 0u;
                        for (unsigned k = first; k < motor_cfg::query_period_ms.size(); k++) {
                            unsigned period_ms = k == 3 ? x->route.PowerQueryPeriodMs : motor_cfg::query_period_ms[k];
                            if (!period_ms || ((x->query_requested_mask & (1u << k)) && now - x->query_request_ms[k] < period_ms))
                                continue;
                            if (parameters::start_query(c, a, k + 1, 1, now, x->parameter_dialect, x->route.CanId, state.channels[c].FrameFormat)) {
                                x->query_request_ms[k] = now;
                                x->query_requested_mask |= 1u << k;
                                state.query_slot[c] = (slot + 1) % ENCOS_MOTORS_PER_CHANNEL;
                            }
                            break;
                        }
                        if (parameters::inspect(c).busy)
                            break;
                    }
                }
            }
        }
    }
    process_raw(now);
}
static void publish(uint32_t now)
{
    mc_board_tx_t *b = object<mc_board_tx_t>(0x6000);
    ++b->PublishSeq;
    b->BoardStatus = state.board_fault | (state.config_ready ? 1 : 0) | (state.operational ? 2 : 0) | (object<mc_owner_t>(0x2003)->Owner == OWNER_SDO ? 128 : 0);
    b->AxisOnlineMask = b->AxisFaultMask = 0;
    b->ChannelOnlineMask = b->ChannelFaultMask = 0;
    state.rx_total = 0;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++) {
        mc_channel_tx_t *ch = object<mc_channel_tx_t>(0x6100 + c);
        memset(ch, 0, sizeof(*ch));
        uint32_t drops = ethercat_canfd_rx_drops(c) + state.raw_rx[c].drops;
        state.rx_total += drops;
        ch->ChannelStatus = (state.config_ready ? 1 : 0) | (channel_active(c) ? 2 : 0) | (ethercat_canfd_bus_state(c) == 3 ? 4 : 0) | (drops ? 8 : 0) | (state.channel_tx_drop[c] ? 16 : 0);
        ch->RxDropCount = drops > 65535 ? 65535 : drops;
        ch->TxDropCount = state.channel_tx_drop[c] > 65535 ? 65535 : state.channel_tx_drop[c];
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++) {
            axis_t *x = &state.axes[a];
            mc_axis_tx_t *s = axis_feedback(a);
            bool type1 = x->active && x->feedback_seen;
            bool online = x->online_seen && now - x->online_ms <= (type1 ? unsigned(state.config.FeedbackTimeoutMs) : unsigned(motor_cfg::query_online_ms));
            bool power_stale = !x->route.PowerQueryPeriodMs || !x->power_seen || now - x->power_ms >= x->route.PowerQueryPeriodMs * 3u;
            s->PowerAgeMs = age_ms(now, x->power_ms, x->power_seen);
            bool stale;
            if (type1) {
                s->FeedbackAgeMs = age_ms(now, x->feedback_ms, true);
                stale = s->FeedbackAgeMs >= state.config.FeedbackTimeoutMs;
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
            bool ready = type1 ? (!stale && !s->BridgeFaultCode && !s->MotorFaultRaw && ethercat_canfd_bus_state(c) != 3) : query_ready(a, now);
            s->StatusWord = (s->StatusWord & 0x320) | (x->configured ? 1 : 0) | (online ? 2 : 0) | (ready ? 4 : 0) | (type1 ? 8 : 0) | (x->new_feedback ? 16 : 0) | (s->BridgeFaultCode ? 32 : 0) | (s->MotorFaultRaw ? 64 : 0) | (stale ? 128 : 0) | (x->new_power ? 1024 : 0) | (power_stale ? 2048 : 0) | (!type1 && (x->position_seen || x->velocity_seen || x->current_seen) ? MC_STATUS_QUERY_SOURCE : 0) | (x->query_armed && !type1 ? MC_STATUS_ARMED_QUERY_ONLY : 0);
            x->new_feedback = x->new_power = false;
            if (online) {
                b->AxisOnlineMask |= 1u << a;
                ch->OnlineMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
            if (s->BridgeFaultCode || s->MotorFaultRaw) {
                b->AxisFaultMask |= 1u << a;
                ch->FaultMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
            axis_diagnostics(a)->PositionQueryAgeUs = age(now, x->position_ms, x->position_seen);
            axis_diagnostics(a)->VelocityQueryAgeUs = age(now, x->velocity_ms, x->velocity_seen);
            axis_diagnostics(a)->TxCompleteCount = ethercat_canfd_completions(a);
        }
        if (ch->OnlineMask) {
            b->ChannelOnlineMask |= 1u << c;
        }
        if (ch->FaultMask || (ch->ChannelStatus & 4)) {
            b->ChannelFaultMask |= 1u << c;
        }
    }
    b->RxDropCount = state.rx_total;
    b->TxDropCount = state.tx_total;
    if (state.rx_total) {
        state.board_fault |= 16;
        b->BoardStatus |= 16;
    }
}
namespace dcu::app::motor_application {

bool configuration_writable() { return !state.operational && !any_active(); }
bool owner_change_allowed() { return !any_active() && !any_service(); }
bool transaction_busy(unsigned axis)
{
    return axis < ENCOS_AXIS_COUNT && parameters::inspect(axis / ENCOS_MOTORS_PER_CHANNEL).busy;
}
bool apply_configuration() { return apply_config(); }
bool select_profile(unsigned profile)
{
    if (profile >= pdo_layout::profile_count ||
        (state.operational && profile != state.config.ActivePdoProfile) ||
        (profile == 2 && state.config.AxisConfiguredMask)) return false;
    state.config.ActivePdoProfile = profile;
    state.output_size = pdo_layout::profiles[profile].output;
    return true;
}
PdoPublication prepare_publication()
{
    publish(ethercat_canfd_now_ms());
    PdoPublication snapshot{};
    snapshot.profile = state.config.ActivePdoProfile;
    snapshot.cycle_sequence = state.board_command.CycleSeq;
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; ++c) {
        snapshot.roles[c] = state.channels[c].Role;
        snapshot.raw_drops[c] = state.raw_rx[c].drops;
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; ++a)
        snapshot.type1_count[a] = state.axes[a].type1_count;
    return snapshot;
}
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame) { return raw_pop(bus, &frame); }

void owner_changed() { stop_all(0); state.resume_pdo_owner = false; }
void commit_sdo_command(unsigned axis, mc_sdo_command_t &command, uint32_t now)
{
    if (axis >= ENCOS_AXIS_COUNT) {
        command.CommandResult = RESULT_INVALID;
        return;
    }
    bool gate = state.operational && state.session_armed && state.seen_cycle &&
        (state.board_command.BoardControl & 1) && !(state.board_command.BoardControl & 2) &&
        now - state.last_command_ms < state.board_command.CommandLeaseMs;
    if (object<mc_owner_t>(0x2003)->Owner != OWNER_SDO || !gate) {
        command.CommandResult = RESULT_STATE;
        stop_axis(axis, 8);
    } else if (!command.LeaseMs || command.LeaseMs > state.config.MaxCommandLeaseMs ||
               !newer(command.CommitSeq, command.AcceptedSeq)) {
        command.CommandResult = RESULT_INVALID;
        stop_axis(axis, 2);
    } else {
        mc_axis_rx_t intent;
        memcpy(&intent, &command, sizeof(intent));
        bool ok = accept_axis(axis, &intent, true, now);
        command.CommandResult = ok ? RESULT_OK : RESULT_INVALID;
        if (ok) {
            command.AcceptedSeq = command.CommitSeq;
            state.axes[axis].lease_ms = command.LeaseMs;
            state.axes[axis].commit_ms = now;
        }
    }
}
bool commit_parameter(unsigned axis, bool maintenance, uint32_t now, bool is_preop)
{
    bool permitted = is_preop && configuration_writable() && object<mc_owner_t>(0x2003)->Owner == OWNER_SDO;
    return parameters::commit(axis, maintenance, now, permitted, channel_active(axis / ENCOS_MOTORS_PER_CHANNEL));
}
}
void motor_control_init(void)
{
    state.config = *object<mc_board_config_t>(0x2001);
    state.output_size = pdo_layout::max_output_bytes;
    state.last_process_us = UINT32_MAX;
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++) {
        object<mc_routing_t>(axis_object_base(a))->CanId = a % ENCOS_MOTORS_PER_CHANNEL + 1;
        axis_feedback(a)->FeedbackAgeMs = axis_feedback(a)->PowerAgeMs = UINT16_MAX;
        axis_diagnostics(a)->PositionQueryAgeUs = axis_diagnostics(a)->VelocityQueryAgeUs = UINT32_MAX;
    }
    for (unsigned c = 0; c < ENCOS_CHANNEL_COUNT; c++)
        state.channels[c] = *object<mc_channel_config_t>(0x2010 + c);
    object<mc_board_info_t>(0x2000)->Capabilities = 7; /* Three PDO profiles; hardware validation still required. */
}
