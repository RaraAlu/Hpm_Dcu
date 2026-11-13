#include "motor_control.h"
#include "encos_motor.h"
#include "ethercat_canfd_gateway.h"
#include "ecat_def.h"
#include "ecatslv.h"
#include "sdoserv.h"
#include "objdef.h"
#include "digital_io.h"
#include "hpm_interrupt.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include "encos_storage.inc"

enum
{
    RESULT_OK = 0,
    RESULT_BUSY = 1,
    RESULT_INVALID = 2,
    RESULT_STATE = 3,
    RESULT_TIMEOUT = 4,
    RESULT_UNSUPPORTED = 5,
    RESULT_MISMATCH = 6,
    RESULT_IO = 7
};
enum
{
    OWNER_NONE = 0,
    OWNER_PDO = 1,
    OWNER_SDO = 2
};
enum
{
    CAN_BUDGET_US_PER_MS = 800,
    CAN_BUDGET_CAP_US = 2400
};
enum
{
    CONTROL_RESPONSE_GUARD_US = 100,
    PROCESS_PERIOD_US = 50
};
enum
{
    QUERY_ONLINE_MS = 300,
    QUERY_POSITION_MS = 100,
    QUERY_VELOCITY_MS = 150,
    QUERY_CURRENT_MS = 300
};
static const uint16_t query_period_ms[4] = {100, 100, 200, 1000};
typedef struct
{
    mc_routing_t route;
    mc_basic_t basic;
    mc_range_t range;
    encos_profile_t profile;
    mc_axis_rx_t command;
    mc_axis_rx_t startup_command;
    uint32_t feedback_ms, power_ms, online_ms, position_ms, velocity_ms, current_ms, active_ms, lease_ms, commit_ms;
    uint32_t type1_count;
    uint32_t query_request_ms[4];
    uint8_t query_requested_mask;
    uint16_t previous_control;
    bool configured, active, armed, query_armed, control_sent, feedback_seen, power_seen, online_seen, position_seen, velocity_seen, current_seen, new_feedback, new_power;
} axis_t;
typedef struct
{
    bool busy, sent, writing, verifying, maintenance, internal;
    uint8_t axis, query, group, uuid_mask, operation, read_cursor;
    uint16_t parameter, opcode;
    uint32_t seq, started, hold_until, uuid[3];
    encos_frame_t request;
    uint8_t expected[6], expected_size;
} service_t;
typedef struct
{
    encos_frame_t frames[32];
    uint8_t read, write;
    uint32_t drops;
} raw_queue_t;
static axis_t axes[ENCOS_AXIS_COUNT];
static service_t services[4];
static raw_queue_t raw_rx[4];
static mc_channel_config_t channels[4];
static mc_board_config_t config = {0, 10, 20, 1};
static mc_board_rx_t board_command;
static mc_channel_rx_t channel_command[4];
static uint8_t candidate[784];
static uint16_t input_size = 1072, output_size = 784;
static volatile bool operational, pending, session_armed;
static bool resume_pdo_owner;
static bool seen_cycle, config_ready, raw_pending, raw_seen;
static uint32_t last_command_ms, last_cycle, last_raw_batch, raw_batch, rx_total, tx_total;
static uint32_t last_process_us = UINT32_MAX, last_budget_ms[4], last_publish_ms;
static uint32_t last_bus_tx_us[4], last_axis_control_us[ENCOS_AXIS_COUNT];
static uint16_t budget_us[4];
static uint8_t published_image[1072];
static bool published_once;
static uint32_t last_tx_progress[4], last_tx_progress_ms[4];
static uint32_t last_full_cycle[4], raw_budget_ms[4];
static uint8_t raw_budget[4], round_robin, query_slot[4], control_slot[4];
static uint8_t diagnostic_page;
static bool full_seen[4];
static uint16_t board_fault;
static uint32_t channel_tx_drop[4];

static uint32_t lock(void) { return disable_global_irq(CSR_MSTATUS_MIE_MASK); }
static void unlock(uint32_t irq) { restore_global_irq(irq & CSR_MSTATUS_MIE_MASK); }
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
/* 8-byte FD+BRS at 1/5 Mbps is budgeted at 64 us, classic CAN at 138 us.
 * Includes an engineering stuffing allowance; analyzer measurements remain required. */
static unsigned frame_us(unsigned c, unsigned dlc)
{
    return channels[c].FrameFormat ? 48u + 2u * dlc : 50u + 11u * dlc;
}
static uint32_t control_slack_us(unsigned c, uint32_t now_us)
{
    uint32_t slack = UINT32_MAX;
    unsigned period_us = channel_command[c].Reserved;
    for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
    {
        axis_t *x = &axes[a];
        if (!x->active)
            continue;
        if (!x->control_sent || !period_us)
            return 0;
        uint32_t elapsed = (uint32_t)(now_us - last_axis_control_us[a]);
        if (elapsed >= period_us)
            return 0;
        uint32_t remaining = period_us - elapsed;
        if (remaining < slack)
            slack = remaining;
    }
    return slack;
}
static bool internal_query_window(unsigned c, unsigned active, uint32_t now_us, unsigned query_cost)
{
    if (!active)
        return true;
    unsigned period_us = channel_command[c].Reserved;
    unsigned control_cost = 2u * frame_us(c, 8);
    if (!period_us || (uint64_t)active * control_cost * 1000u > (uint64_t)CAN_BUDGET_US_PER_MS * period_us)
        return false;
    uint32_t since_tx = (uint32_t)(now_us - last_bus_tx_us[c]);
    uint32_t guard_wait = since_tx >= CONTROL_RESPONSE_GUARD_US ? 0u : CONTROL_RESPONSE_GUARD_US - since_tx;
    uint32_t slack = control_slack_us(c, now_us);
    return slack >= guard_wait + PROCESS_PERIOD_US + query_cost && budget_us[c] >= query_cost + active * control_cost;
}
static void budget_tick(unsigned c, uint32_t now)
{
    uint32_t elapsed = now - last_budget_ms[c];
    if (!elapsed)
        return;
    last_budget_ms[c] = now;
    unsigned credit = elapsed >= 3 ? CAN_BUDGET_CAP_US : budget_us[c] + elapsed * CAN_BUDGET_US_PER_MS;
    budget_us[c] = credit > CAN_BUDGET_CAP_US ? CAN_BUDGET_CAP_US : credit;
}
static mc_object_t *find_object(uint16_t index)
{
    unsigned lo = 0, hi = mc_object_count;
    while (lo < hi)
    {
        unsigned m = (lo + hi) / 2;
        if (mc_objects[m].index < index)
            lo = m + 1;
        else
            hi = m;
    }
    return lo < mc_object_count && mc_objects[lo].index == index ? &mc_objects[lo] : NULL;
}
void *mc_object(uint16_t index)
{
    mc_object_t *o = find_object(index);
    return o ? o->data : NULL;
}
#define OD(type, index) ((mc_##type##_t *)mc_object(index))
#define STATE(a) OD(axis_tx, 0x6200 + (a))
#define DIAG(a) OD(diag, 0x2400 + 16 * (a))
#define BASE(a) (0x3000 + 16 * (a))
static bool any_active(void)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        if (axes[a].active)
            return true;
    return false;
}
static bool channel_active(unsigned c)
{
    for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
        if (axes[a].active)
            return true;
    return false;
}
static bool any_service(void)
{
    for (unsigned c = 0; c < 4; c++)
        if (services[c].busy)
            return true;
    return false;
}
static bool preop(void) { return (nAlStatus & STATE_MASK) == STATE_PREOP && !operational && !any_active(); }
static void service_done(unsigned c, uint16_t result);
static void stop_axis(unsigned a, uint16_t fault)
{
    axis_t *x = &axes[a];
    mc_axis_tx_t *s = STATE(a);
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
        DIAG(a)->ActiveFaultBits |= 1u << fault;
}
static void stop_all(uint16_t fault)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        stop_axis(a, axes[a].configured ? fault : 0);
    for (unsigned c = 0; c < 4; c++)
        ethercat_canfd_cancel(c);
    raw_pending = false;
    session_armed = false;
}
void motor_control_operational(bool op)
{
    uint32_t irq = lock();
    mc_owner_t *owner = OD(owner, 0x2003);
    if (!op)
    {
        /* Leaving OP ends this PDO session.  The idle EtherCAT master may
         * keep the slave in PREOP, but that must not keep CAN polling alive. */
        resume_pdo_owner = resume_pdo_owner || owner->Owner == OWNER_PDO;
        owner->Owner = OWNER_NONE;
        owner->RequestedOwner = OWNER_NONE;
        owner->Result = RESULT_OK;
    }
    else if (resume_pdo_owner && owner->Owner == OWNER_NONE)
    {
        /* A transient OP loss in the same HMI session can recover without
         * repeating the PREOP SDO setup.  A new owner write cancels this. */
        owner->Owner = OWNER_PDO;
        owner->RequestedOwner = OWNER_PDO;
        resume_pdo_owner = false;
    }
    operational = op;
    pending = false;
    seen_cycle = false;
    raw_seen = false;
    published_once = false;
    memset(full_seen, 0, sizeof(full_seen));
    memset(&board_command, 0, sizeof(board_command));
    stop_all(0);
    if (!op)
    {
        for (unsigned c = 0; c < 4; c++)
        {
            service_t *t = &services[c];
            if (t->busy)
                service_done(c, RESULT_STATE);
            memset(t, 0, sizeof(*t));
        }
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
    {
        memset(&axes[a].command, 0, sizeof(axes[a].command));
        axes[a].previous_control = 0;
    }
    unlock(irq);
}
void motor_control_tick(uint32_t now)
{
    if (!operational)
        return;
    if (seen_cycle && (uint32_t)(now - last_command_ms) >= board_command.CommandLeaseMs)
    {
        board_fault |= 4;
        stop_all(3);
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        if (axes[a].active || axes[a].query_armed)
        {
            axis_t *x = &axes[a];
            if (OD(owner, 0x2003)->Owner == OWNER_SDO && (uint32_t)(now - x->commit_ms) >= x->lease_ms)
                stop_axis(a, 3);
            /* A previous session's Type-1 timestamp must not expire a new session.
             * Start the first-feedback deadline only after its first control frame
             * has actually entered the CAN transmitter. */
            if (x->control_sent && (uint32_t)(now - (x->feedback_seen ? x->feedback_ms : x->active_ms)) >= config.FeedbackTimeoutMs)
                stop_axis(a, 4);
        }
}
static bool query_ready(unsigned a, uint32_t now)
{
    axis_t *x = &axes[a];
    return x->configured && x->online_seen && now - x->online_ms <= QUERY_ONLINE_MS && x->position_seen && now - x->position_ms <= QUERY_POSITION_MS && x->velocity_seen && now - x->velocity_ms <= QUERY_VELOCITY_MS && x->current_seen && now - x->current_ms <= QUERY_CURRENT_MS && fabs((double)STATE(a)->ActualCurrent) <= x->route.CurrentLimit * 10.0 && !STATE(a)->MotorFaultRaw && !STATE(a)->BridgeFaultCode && ethercat_canfd_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3;
}
static bool accept_axis(unsigned a, const mc_axis_rx_t *command, bool permit, uint32_t now)
{
    axis_t *x = &axes[a];
    mc_axis_tx_t *s = STATE(a);
    uint16_t prev = x->previous_control;
    x->previous_control = command->ControlWord;
    if (!permit || (command->ControlWord & MC_AXIS_STOP) || !(command->ControlWord & MC_AXIS_ARM))
    {
        stop_axis(a, 0);
        if (permit && !(command->ControlWord & MC_AXIS_ARM) && !(command->ControlWord & MC_AXIS_STOP))
        {
            x->armed = true;
            if ((command->ControlWord & MC_AXIS_RESET) && !(prev & MC_AXIS_RESET) && ethercat_canfd_bus_state(a / ENCOS_MOTORS_PER_CHANNEL) != 3 && !s->MotorFaultRaw)
            {
                s->BridgeFaultCode = 0;
                s->StatusWord &= ~((1u << 5) | (1u << 8) | (1u << 9));
                DIAG(a)->ActiveFaultBits = 0;
            }
        }
        memset(&x->command, 0, sizeof(x->command));
        return true;
    }
    if ((command->ControlWord & ~15u) || command->Flags)
    {
        stop_axis(a, 2);
        return false;
    }
    if (!x->query_armed && !x->active)
    {
        if (!x->configured || !x->armed || (prev & MC_AXIS_ARM) || (command->ControlWord & MC_AXIS_START_MOTION))
            return false;
        x->query_armed = true;
        x->armed = false;
    }
    if (!(command->ControlWord & MC_AXIS_START_MOTION))
    {
        if (x->active)
        {
            stop_axis(a, 0);
            x->query_armed = true;
        }
        memset(&x->command, 0, sizeof(x->command));
        return true;
    }
    encos_frame_t f;
    if (!x->configured || !encos_encode(command, &x->route, &x->profile, &f))
    {
        stop_axis(a, x->configured ? 2 : 1);
        s->StatusWord |= 1u << 8;
        return false;
    }
    if (x->active && x->command.Mode != command->Mode)
    {
        stop_axis(a, 8);
        s->StatusWord |= 1u << 8;
        return false;
    }
    if (!x->active)
    {
        /* StartMotion needs a fresh edge after a separate query-only cycle. */
        if ((prev & MC_AXIS_START_MOTION) || !query_ready(a, now) ||
            fabs((double)DIAG(a)->QueriedVelocity) > 20000.0 ||
            ((command->Mode == 1 || command->Mode == 4) &&
             fabs((double)command->TargetPosition - DIAG(a)->QueriedPosition) > 100000.0))
            return false;
        service_t *t = &services[a / ENCOS_MOTORS_PER_CHANNEL];
        /* An inactive-axis polling request must never delay the first motion
         * command. Abort only the internal transaction; explicit SDO service
         * ownership cannot coexist with PDO control ownership. */
        if (t->busy && t->internal)
        {
            uint32_t hold = t->hold_until;
            ethercat_canfd_cancel_service(a / ENCOS_MOTORS_PER_CHANNEL);
            memset(t, 0, sizeof(*t));
            t->hold_until = hold;
        }
        x->startup_command = *command;
        x->startup_command.ControlWord = MC_AXIS_ARM;
        x->startup_command.Mode = 1;
        x->startup_command.TargetPosition = DIAG(a)->QueriedPosition;
        x->startup_command.TargetVelocity = 0;
        x->startup_command.TargetTorque = 0;
        x->startup_command.TargetCurrent = 0;
        x->startup_command.Kp = 0;
        x->startup_command.Kd = 0;
        x->startup_command.CurrentLimit = x->route.CurrentLimit < 4095 ? x->route.CurrentLimit : 4095;
        if (!encos_encode(&x->startup_command, &x->route, &x->profile, &f))
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
    uint32_t irq = lock();
    memcpy(candidate, p, output_size);
    pending = true;
    unlock(irq);
}
static void accept_candidate(uint32_t now)
{
    if (!pending)
    {
        return;
    }
    pending = false;
    mc_board_rx_t b;
    memcpy(&b, candidate, sizeof(b));
    /* Stop intent always wins, including duplicate sequence or malformed targets. */
    bool stop = !(b.BoardControl & 1) || (b.BoardControl & 2);
    if (stop)
        stop_all(0);
    if (config.ActivePdoProfile != 2)
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        {
            mc_axis_rx_t c;
            mc_channel_rx_t ch;
            memcpy(&c, candidate + 64 + 24 * a, 24);
            memcpy(&ch, candidate + 32 + 8 * (a / ENCOS_MOTORS_PER_CHANNEL), 8);
            if (!(c.ControlWord & 1) || (c.ControlWord & 2) || !(ch.ChannelControl & 1) || (ch.ChannelControl & 2) || !(b.ChannelEnableMask & (1u << (a / ENCOS_MOTORS_PER_CHANNEL))) || !(b.AxisEnableMask & (1u << a)) || !(ch.ActiveMask & (1u << (a % ENCOS_MOTORS_PER_CHANNEL))))
                stop_axis(a, 0);
        }
    if (b.InterfaceVersion != MC_INTERFACE_VERSION || (b.BoardControl & ~7u) || (b.ChannelEnableMask & 0xF0) || (b.AxisEnableMask & 0xF0000000u) || b.Reserved0 || b.Reserved1 || b.Reserved2 || b.Reserved3 || b.Reserved4 || !b.CommandLeaseMs || b.CommandLeaseMs > config.MaxCommandLeaseMs)
    {
        board_fault |= 64;
        stop_all(9);
        return;
    }
    if (!operational || (seen_cycle && !newer(b.CycleSeq, last_cycle)))
        return;
    if (!session_armed && !stop)
    {
        stop_all(8);
        return;
    }
    if (stop && !b.AxisEnableMask)
        session_armed = true;
    if ((b.BoardControl & 4) && !(board_command.BoardControl & 4) && stop)
        board_fault = 0;
    last_cycle = b.CycleSeq;
    seen_cycle = true;
    last_command_ms = now;
    board_command = b;
    *OD(board_rx, 0x7000) = b;
    OD(board_tx, 0x6000)->AcceptedCycleSeq = b.CycleSeq;
    if (config.ActivePdoProfile == 2)
    {
        raw_pending = !stop;
        return;
    }
    for (unsigned c = 0; c < 4; c++)
    {
        mc_channel_rx_t ch;
        memcpy(&ch, candidate + 32 + c * 8, 8);
        channel_command[c] = ch;
        *OD(channel_rx, 0x7100 + c) = ch;
        bool valid = !(ch.ChannelControl & ~3u) && !(ch.ActiveMask & 0x80u) && ch.Reserved >= 1000u && ch.Reserved <= 10000u && ch.Reserved % 1000u == 0 && ch.ExpectedRole == channels[c].Role;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++)
        {
            unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
            mc_axis_rx_t cmd;
            memcpy(&cmd, candidate + 64 + 24 * a, 24);
            bool permit = valid && !stop && (b.ChannelEnableMask & (1u << c)) && (b.AxisEnableMask & (1u << a)) && (ch.ChannelControl & 1) && !(ch.ChannelControl & 2) && (ch.ActiveMask & (1u << j));
            if (!valid)
            {
                stop_axis(a, 7);
                continue;
            }
            if (OD(owner, 0x2003)->Owner == OWNER_PDO)
                accept_axis(a, &cmd, permit, now);
        }
    }
    if (config.ActivePdoProfile == 1)
    {
        mc_raw_t *r = OD(raw, 0x7300);
        memcpy(r, candidate + 736, 48);
        if (!raw_seen || newer(r->BatchSeq, last_raw_batch))
        {
            last_raw_batch = r->BatchSeq;
            raw_seen = true;
            raw_pending = !stop;
        }
    }
}
static bool apply_config(void)
{
    mc_board_config_t *b = OD(board_config, 0x2001);
    if (!preop() || any_service() || b->MaxCommandLeaseMs < 2 || b->MaxCommandLeaseMs > 10 || b->FeedbackTimeoutMs < 5 || b->FeedbackTimeoutMs > 20)
        return false;
    uint32_t mask = 0;
    for (unsigned c = 0; c < 4; c++)
    {
        mc_channel_config_t *ch = OD(channel_config, 0x2010 + c);
        if (ch->Role > 3 || ch->FrameFormat > 1 || ch->NominalBitrate != 1000000 || ch->DataBitrate != 5000000 || ch->ScheduleUs != 2000 || ch->RawRateLimit > 8)
            return false;
        if ((ch->Role == 0 || ch->Role == 2) && ch->MotorConfiguredMask)
            return false;
        if (config.ActivePdoProfile == 2 && ch->MotorConfiguredMask)
            return false;
        uint16_t ids = 0;
        if (ch->MotorConfiguredMask & 0x80u)
            return false;
        for (unsigned j = 0; j < ENCOS_MOTORS_PER_CHANNEL; j++)
            if (ch->MotorConfiguredMask & (1u << j))
            {
                unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + j;
                mc_routing_t *r = OD(routing, BASE(a));
                mc_basic_t *basic = OD(basic, BASE(a) + 1);
                encos_profile_t p;
                if (r->CanId < 1 || r->CanId > 8 || (ids & (1u << r->CanId)) || r->ModelProfileId != 0x8000 || r->StopPolicy != 0 || r->FeedbackPolicy != 1 || !encos_profile(OD(range, BASE(a) + 2), basic, &p))
                    return false;
                if (r->PositionLimitMin >= r->PositionLimitMax || r->PositionLimitMin < p.pos[0] * 1e6 || r->PositionLimitMax > p.pos[1] * 1e6 || !r->VelocityLimit || r->VelocityLimit > fmin(-p.vel[0], p.vel[1]) * 1e6 || !r->CurrentLimit || r->CurrentLimit > fmin(-p.current[0], p.current[1]) * 10 || !r->TorqueLimit || r->TorqueLimit > fmin(-p.torque[0], p.torque[1]) * 1000 || (r->PowerQueryPeriodMs && (r->PowerQueryPeriodMs < 20 || r->PowerQueryPeriodMs > 1000)) || basic->CommunicationMode != ch->FrameFormat || basic->CanTimeoutMs > 500)
                    return false;
                ids |= 1u << r->CanId;
                mask |= 1u << a;
            }
    }
    if (mask != b->AxisConfiguredMask)
        return false;
    stop_all(0);
    config = *b;
    for (unsigned c = 0; c < 4; c++)
    {
        channels[c] = *OD(channel_config, 0x2010 + c);
        budget_us[c] = CAN_BUDGET_CAP_US;
        last_budget_ms[c] = ethercat_canfd_now_ms();
    }
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
    {
        axis_t *x = &axes[a];
        memset(x, 0, sizeof(*x));
        memset(STATE(a), 0, sizeof(mc_axis_tx_t));
        x->configured = (mask & (1u << a)) != 0;
        x->route = *OD(routing, BASE(a));
        x->basic = *OD(basic, BASE(a) + 1);
        x->range = *OD(range, BASE(a) + 2);
        if (x->configured)
            encos_profile(&x->range, &x->basic, &x->profile);
    }
    config_ready = true;
    return true;
}
static void service_done(unsigned c, uint16_t result)
{
    service_t *t = &services[c];
    unsigned a = t->axis;
    if (!t->internal)
    {
        if (t->maintenance)
        {
            mc_maintenance_t *m = OD(maintenance, BASE(a) + 8);
            m->DoneSeq = t->seq;
            m->Result = result;
        }
        else
        {
            mc_transaction_t *m = OD(transaction, BASE(a) + 7);
            m->DoneSeq = t->seq;
            m->Result = result;
        }
    }
    DIAG(a)->LastServiceError = result;
    t->busy = false;
    t->sent = false;
}
static bool start_query(unsigned c, unsigned a, uint8_t code, uint8_t group, uint32_t now, bool internal)
{
    service_t *t = &services[c];
    if (t->busy || (int32_t)(now - t->hold_until) < 0)
        return false;
    uint32_t hold = t->hold_until;
    memset(t, 0, sizeof(*t));
    t->hold_until = hold;
    t->axis = a;
    t->query = code;
    t->group = group;
    t->internal = internal;
    t->request.id = axes[a].route.CanId;
    if (!encos_query(code, group, &t->request))
        return false;
    t->request.flags = channels[c].FrameFormat ? 24 : 0;
    t->busy = true;
    t->started = now;
    return true;
}
static bool cache_query(unsigned a, uint8_t code, const encos_frame_t *f, uint32_t now, service_t *t)
{
    mc_query_t *q = OD(query, BASE(a) + 5);
    axis_t *x = &axes[a];
    int32_t value;
    const uint8_t *d = f->data;
    bool ok = false;
    if (code >= 1 && code <= 4 && f->dlc == 6)
    {
        double scale = code == 1 ? 1e6 * 3.141592653589793 / 180 : code == 2 ? 1e6 * 2 * 3.141592653589793 / 60
                                                               : code == 3   ? 100
                                                                             : 1000;
        if (!encos_float(d + 2, scale, &value))
            return false;
        uint32_t bits = encos_be32(d + 2);
        float real;
        memcpy(&real, &bits, 4);
        memcpy((uint8_t *)q + 4 * (code - 1), &real, 4);
        if (code == 1)
        {
            DIAG(a)->QueriedPosition = value;
            x->position_ms = now;
            x->position_seen = true;
            if (!x->active)
                STATE(a)->ActualPosition = value;
        }
        if (code == 2)
        {
            DIAG(a)->QueriedVelocity = value;
            x->velocity_ms = now;
            x->velocity_seen = true;
            if (!x->active)
                STATE(a)->ActualVelocity = value;
        }
        if (code == 3)
        {
            if (value < INT16_MIN || value > INT16_MAX)
                return false;
            x->current_ms = now;
            x->current_seen = true;
            if (!x->active)
                STATE(a)->ActualCurrent = (int16_t)value;
        }
        if (code == 4)
        {
            STATE(a)->ActualPower = value;
            x->power_ms = now;
            x->power_seen = true;
            x->new_power = true;
        }
        q->ValidBits |= 1u << (code - 1);
        ok = true;
    }
    else if ((code == 5 || code == 22 || code == 31 || code == 38) && f->dlc == 4)
    {
        uint16_t v = encos_be16(d + 2);
        mc_basic_t *b = OD(basic, BASE(a) + 1);
        if (code == 5)
        {
            q->AccelerationRaw = v;
            b->AccelerationRaw = v;
            q->ValidBits |= 1u << 4;
        }
        if (code == 22)
            b->TorqueConstantRaw = v;
        if (code == 31)
            b->CanTimeoutMs = v;
        if (code == 38)
        {
            q->ZeroOffsetRaw = (int16_t)v;
            b->ZeroOffsetRaw = (int16_t)v;
            q->ValidBits |= 1u << 9;
        }
        ok = true;
    }
    else if (((code >= 23 && code <= 28) || (code >= 32 && code <= 34)) && f->dlc == 6)
    {
        uint16_t v[2] = {encos_be16(d + 2), encos_be16(d + 4)};
        void *dest = code <= 28 ? (uint8_t *)mc_object(BASE(a) + 2) + (code - 23) * 4 : (uint8_t *)mc_object(BASE(a) + 3) + (code - 32) * 4;
        memcpy(dest, v, 4);
        ok = true;
    }
    else if (code == 29 && f->dlc == 7 && d[2] >= 1 && d[2] <= 3)
    {
        t->uuid[d[2] - 1] = encos_be32(d + 3);
        t->uuid_mask |= 1u << (d[2] - 1);
        if (t->uuid_mask != 7)
        {
            return false;
        }
        memcpy(mc_object(BASE(a) + 6), t->uuid, 12);
        ok = true;
    }
    else if (code == 30 && f->dlc == 8)
    {
        memcpy((uint8_t *)mc_object(BASE(a) + 6) + 12, d + 2, 6);
        ok = true;
    }
    else if ((code == 35 || code == 37) && f->dlc == 3)
    {
        if (code == 35 && d[2] <= 2)
        {
            OD(kt, BASE(a) + 4)->CalibrationMode = d[2];
            ok = true;
        }
        if (code == 37 && d[2] <= 1)
        {
            q->BrakeState = d[2];
            q->ValidBits |= 1u << 5;
            ok = true;
        }
    }
    if (ok)
    {
        ++q->SnapshotSeq;
        q->AgeMs = 0;
    }
    return ok;
}
static void consume_feedback(unsigned c, const encos_frame_t *f, uint32_t stamp, uint32_t now)
{
    if ((f->flags & 2) || f->flags != (OD(channel_config, 0x2010 + c)->FrameFormat ? 24 : 0) || f->dlc < 2)
        return;
    service_t *t = &services[c];
    unsigned a = ENCOS_AXIS_COUNT;
    if (t->busy && t->maintenance && t->sent && f->id == 0x7FF && f->dlc >= 4)
    {
        const mc_routing_t *route = &axes[t->axis].route;
        if (f->data[0] == (route->CanId >> 8) && f->data[1] == (route->CanId & 0xFF) && f->data[2] == 1)
        {
            memcpy(OD(maintenance, BASE(t->axis) + 8)->RawResponse, f->data, 8);
            if (f->data[3] == 3)
                service_done(c, RESULT_OK);
            else if (f->data[3] == 0)
                service_done(c, RESULT_IO);
        }
        return;
    }
    for (unsigned j = c * ENCOS_MOTORS_PER_CHANNEL; j < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; j++)
        if (axes[j].configured && axes[j].route.CanId == f->id)
        {
            a = j;
            break;
        }
    if (t->busy && t->request.id == f->id)
        a = t->axis;
    if (a == ENCOS_AXIS_COUNT)
        return;
    axis_t *x = &axes[a];
    uint8_t type = f->data[0] >> 5;
    bool valid = false;
    if (type == 1 && x->active && x->control_sent && x->configured && encos_feedback(f, &x->profile, STATE(a)))
    {
        if (!x->feedback_seen)
        {
            x->position_seen = x->velocity_seen = x->current_seen = false;
        }
        x->feedback_seen = true;
        x->feedback_ms = stamp;
        x->new_feedback = true;
        x->query_armed = false;
        valid = true;
        ++x->type1_count;
        if (fabs((double)STATE(a)->ActualCurrent) > x->route.CurrentLimit * 10.0)
            stop_axis(a, 2);
    }
    else if ((type == 2 || type == 3) && f->dlc == 8 && (f->data[0] & 31) <= 7)
    {
        int32_t v;
        if (encos_float(f->data + 1, type == 2 ? 1e6 * 3.141592653589793 / 180 : 1e6 * 2 * 3.141592653589793 / 60, &v))
        {
            if (type == 2)
            {
                DIAG(a)->QueriedPosition = v;
                x->position_ms = stamp;
                x->position_seen = true;
            }
            else
            {
                DIAG(a)->QueriedVelocity = v;
                x->velocity_ms = stamp;
                x->velocity_seen = true;
            }
            valid = true;
        }
    }
    else if (t->busy && t->sent && a == t->axis)
    {
        mc_transaction_t *tr = OD(transaction, BASE(a) + 7);
        tr->RawResponseLength = f->dlc;
        memcpy(tr->RawResponse, f->data, 8);
        if (t->maintenance)
            memcpy(OD(maintenance, BASE(a) + 8)->RawResponse, f->data, 8);
        if (t->writing && ((type == 4 && f->dlc == 3 && f->data[1] == t->request.data[1]) || (f->data[0] == 0xFF && f->data[1] == 0xFE && f->dlc >= 4 && f->data[2] == t->request.data[1])))
        {
            if (type == 4 && (f->data[2] != 1 || (f->data[0] & 31)))
            {
                service_done(c, RESULT_IO);
                return;
            }
            t->writing = false;
            if (!t->query)
            {
                service_done(c, RESULT_OK);
                return;
            }
            t->verifying = true;
            t->sent = false;
            t->started = now;
            encos_query(t->query, t->group, &t->request);
            return;
        }
        if (type == 5 && (f->data[0] & 31) <= 7 && f->data[1] == t->query)
        {
            if (f->data[0] & 31)
            {
                service_done(c, RESULT_IO);
                valid = true;
            }
            else if (cache_query(a, t->query, f, stamp, t))
            {
                valid = true;
                if (t->verifying && (f->dlc != t->expected_size + 2 || memcmp(f->data + 2, t->expected, t->expected_size)))
                    service_done(c, RESULT_MISMATCH);
                else
                    service_done(c, RESULT_OK);
            }
        }
    }
    if (valid)
    {
        x->online_seen = true;
        x->online_ms = stamp;
        DIAG(a)->RxValidCount++;
        if (type >= 1 && type <= 5)
        {
            STATE(a)->MotorFaultRaw = f->data[0] & 31;
            if (STATE(a)->MotorFaultRaw)
                stop_axis(a, 0);
        }
    }
    else
        DIAG(a)->ProtocolErrorCount++;
}
static void raw_push(unsigned c, const encos_frame_t *f)
{
    raw_queue_t *q = &raw_rx[c];
    uint8_t next = (q->write + 1) & 31;
    if (next == q->read)
    {
        ++q->drops;
        return;
    }
    q->frames[q->write] = *f;
    q->write = next;
}
static bool raw_pop(unsigned c, encos_frame_t *f)
{
    raw_queue_t *q = &raw_rx[c];
    if (q->read == q->write)
        return false;
    *f = q->frames[q->read];
    q->read = (q->read + 1) & 31;
    return true;
}
static bool raw_send(unsigned c, const encos_frame_t *f, uint32_t now)
{
    if (c >= 4 || !(board_command.ChannelEnableMask & (1u << c)) || (channels[c].Role != 2 && channels[c].Role != 3) || services[c].busy || (int32_t)(now - services[c].hold_until) < 0)
        return false;
    if (config.ActivePdoProfile != 2)
    {
        if (f->id == 0x7FF)
            return false;
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            if (axes[a].configured && f->id == axes[a].route.CanId)
                return false;
    }
    if (now - raw_budget_ms[c] >= 2)
    {
        raw_budget_ms[c] = now;
        raw_budget[c] = 0;
    }
    if (raw_budget[c] >= channels[c].RawRateLimit)
    {
        return false;
    }
    unsigned cost = ((f->flags & 8u) ? 48u + 2u * f->dlc : 50u + 11u * f->dlc) + frame_us(c, 8);
    if (budget_us[c] < cost || !ethercat_canfd_send(c, f, ENCOS_AXIS_COUNT))
        return false;
    budget_us[c] -= cost;
    ++raw_budget[c];
    return true;
}
static void process_raw(uint32_t now)
{
    if (!raw_pending)
    {
        return;
    }
    raw_pending = false;
    if (!operational || !session_armed || !(board_command.BoardControl & 1) || (board_command.BoardControl & 2) || now - last_command_ms >= board_command.CommandLeaseMs)
        return;
    if (config.ActivePdoProfile == 1)
    {
        mc_raw_t *r = OD(raw, 0x7300);
        if ((r->ValidMask & ~3u) || r->SlotCount > 2 || r->Flags || r->DropCount || r->Reserved || r->Reserved1 || r->Reserved2)
            return;
        for (unsigned i = 0; i < 2; i++)
            if (r->ValidMask & (1u << i))
            {
                const uint8_t *p = (const uint8_t *)r + 16 + 16 * i;
                encos_frame_t f = {0};
                memcpy(&f.id, p, 4);
                f.flags = p[5];
                f.dlc = p[6];
                memcpy(f.data, p + 8, 8);
                if (!raw_send(p[4], &f, now))
                {
                    ++tx_total;
                    if (p[4] < 4)
                        ++channel_tx_drop[p[4]];
                }
            }
    }
    else if (config.ActivePdoProfile == 2)
    {
        for (unsigned c = 0; c < 4; c++)
        {
            const canfd_bus_pdo_t *p = (const canfd_bus_pdo_t *)(candidate + 32 + 128 * c);
            memcpy(mc_object(0x7400 + 16 * c), p, 128);
            if (full_seen[c] && (uint16_t)(p->cycle_seq - last_full_cycle[c]) == 0)
                continue;
            last_full_cycle[c] = p->cycle_seq;
            full_seen[c] = true;
            if (p->valid_mask & 0x80u)
                continue;
            for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS; i++)
                if ((p->valid_mask & (1u << i)) && (p->slot[i].flags & 1))
                {
                    const canfd_slot_pdo_t *s = &p->slot[i];
                    encos_frame_t f = {.id = s->can_id, .dlc = s->dlc, .flags = s->flags & ~33u};
                    memcpy(f.data, s->data, 8);
                    if (!raw_send(c, &f, now))
                    {
                        ++tx_total;
                        ++channel_tx_drop[c];
                    }
                }
        }
    }
}
void motor_control_process(void)
{
    uint32_t now_us = ethercat_canfd_now_us();
    if ((uint32_t)(now_us - last_process_us) < PROCESS_PERIOD_US)
        return;
    last_process_us = now_us;
    uint32_t irq = lock(), now = ethercat_canfd_now_ms();
    accept_candidate(now);
    motor_control_tick(now);
    for (unsigned c = 0; c < 4; c++)
    {
        budget_tick(c, now);
        encos_frame_t f;
        uint32_t stamp;
        for (unsigned n = 0; n < 32 && ethercat_canfd_receive(c, &f, &stamp); n++)
        {
            consume_feedback(c, &f, stamp, now);
            if (channels[c].Role >= 2)
                raw_push(c, &f);
        }
        service_t *t = &services[c];
        if (ethercat_canfd_bus_state(c) == 3)
        {
            board_fault |= 8;
            if (t->busy)
            {
                ethercat_canfd_cancel_service(c);
                service_done(c, RESULT_IO);
            }
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (axes[a].configured)
                    stop_axis(a, 5);
            continue;
        }
        if (t->busy && now - t->started >= 100u)
        {
            ethercat_canfd_cancel_service(c);
            service_done(c, RESULT_TIMEOUT);
        }
        /* Do not queue a delayed start after its query snapshot has expired. */
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            if (axes[a].active && !axes[a].control_sent && !query_ready(a, now))
                stop_axis(a, 8);
        unsigned active = 0;
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
            if (axes[a].active)
                ++active;
        uint32_t progress = ethercat_canfd_tx_progress(c);
        if (progress != last_tx_progress[c] || !ethercat_canfd_pending(c))
        {
            last_tx_progress[c] = progress;
            last_tx_progress_ms[c] = now;
        }
        else if (now - last_tx_progress_ms[c] >= config.FeedbackTimeoutMs)
        {
            /* 只把“队列持续无任何发送完成”视为真正的 TX 卡死。
             * 正常多轴流水发送时 TXBRP 可以跨调度周期保持非零。 */
            ethercat_canfd_cancel(c);
            ++channel_tx_drop[c];
            ++tx_total;
            board_fault |= 32;
            for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
                if (axes[a].active)
                {
                    ++DIAG(a)->TxDropCount;
                    stop_axis(a, 6);
                }
            continue;
        }
        if (operational && session_armed && config.ActivePdoProfile != 2)
        {
            /* Only one motor command may enter a CAN transmitter at a time.
             * Wait for that frame to leave MCAN, then reserve 100 us from its
             * enqueue point for the motor's Type-1 response before another
             * motor on the same bus is addressed. Each axis still observes
             * the 1..10 ms period requested by the HMI; a loaded bus can only
             * run as fast as N * 100 us for N active motors. */
            unsigned cost = 2u * frame_us(c, 8);
            bool bus_ready = active && !ethercat_canfd_pending(c) && ethercat_canfd_tx_free(c) > 0 && budget_us[c] >= cost && (uint32_t)(now_us - last_bus_tx_us[c]) >= CONTROL_RESPONSE_GUARD_US;
            if (bus_ready)
                for (unsigned n = 0; n < ENCOS_MOTORS_PER_CHANNEL; n++)
                {
                    unsigned slot = (control_slot[c] + n) % ENCOS_MOTORS_PER_CHANNEL;
                    unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                    axis_t *x = &axes[a];
                    if (!x->active)
                        continue;
                    unsigned period_us = channel_command[c].Reserved;
                    uint32_t elapsed_us = (uint32_t)(now_us - last_axis_control_us[a]);
                    if (x->control_sent && elapsed_us < period_us)
                        continue;
                    if (encos_encode(x->feedback_seen ? &x->command : &x->startup_command, &x->route, &x->profile, &f))
                    {
                        f.flags = channels[c].FrameFormat ? 24 : 0;
                        if (ethercat_canfd_send(c, &f, a))
                        {
                            budget_us[c] -= cost;
                            last_bus_tx_us[c] = now_us;
                            /* Keep the requested rate phase-locked.  Re-basing every
                             * successful send made one scheduler quantum of lateness
                             * accumulate on every 2 ms period (about 490 Hz instead of
                             * 500 Hz).  Do not catch up with bursts after a long stall. */
                            if (!x->control_sent || elapsed_us >= 2u * period_us)
                                last_axis_control_us[a] = now_us;
                            else
                                last_axis_control_us[a] += period_us;
                            control_slot[c] = (slot + 1) % ENCOS_MOTORS_PER_CHANNEL;
                            if (!axes[a].control_sent)
                            {
                                axes[a].control_sent = true;
                                axes[a].active_ms = now;
                            }
                            STATE(a)->ModeDisplay = x->feedback_seen ? x->command.Mode : 1;
                            ++DIAG(a)->TxEnqueueCount;
                        }
                        else
                        {
                            ++DIAG(a)->TxDropCount;
                            ++channel_tx_drop[c];
                            ++tx_total;
                        }
                    }
                    else
                        stop_axis(a, 2);
                    break;
                }
        }
        if (t->busy && !t->sent && !ethercat_canfd_pending(c) && ethercat_canfd_tx_free(c) && (uint32_t)(now_us - last_bus_tx_us[c]) >= CONTROL_RESPONSE_GUARD_US)
        {
            unsigned cost = frame_us(c, t->request.dlc) + frame_us(c, 8);
            bool query_window = !t->internal || internal_query_window(c, active, now_us, cost);
            if (query_window && budget_us[c] >= cost && ethercat_canfd_send(c, &t->request, ENCOS_AXIS_COUNT))
            {
                budget_us[c] -= cost;
                last_bus_tx_us[c] = now_us;
                t->sent = true;
                t->started = now;
                if (t->maintenance || t->parameter == 2)
                    t->hold_until = now + 500;
            }
        }
        unsigned owner = OD(owner, 0x2003)->Owner;
        if (!t->busy && (owner == OWNER_PDO || owner == OWNER_SDO) && config.ActivePdoProfile != 2 && channels[c].Role != 0)
        {
            /* Internal queries are strictly best effort.  Reserve one complete
             * control sweep and enough wall-clock slack before creating the
             * service, otherwise the query waits without stealing a deadline.
             * Query-only axes take priority; active axes need only code 4
             * because Type-1 already carries position, velocity, current,
             * temperatures and the motor fault. */
            unsigned query_cost = frame_us(c, 2) + frame_us(c, 8);
            if (internal_query_window(c, active, now_us, query_cost))
            {
                for (unsigned pass = 0; pass < 2 && !t->busy; pass++)
                {
                    for (unsigned n = 0; n < ENCOS_MOTORS_PER_CHANNEL; n++)
                    {
                        unsigned slot = (query_slot[c] + n) % ENCOS_MOTORS_PER_CHANNEL;
                        unsigned a = c * ENCOS_MOTORS_PER_CHANNEL + slot;
                        axis_t *x = &axes[a];
                        if (!x->configured || (pass == 0 ? x->active : !x->active))
                            continue;
                        if (!x->active && owner == OWNER_SDO && !x->query_armed)
                            continue;
                        if (x->active && (owner != OWNER_PDO || !x->route.PowerQueryPeriodMs))
                            continue;
                        unsigned first = x->active ? 3u : 0u;
                        for (unsigned k = first; k < 4; k++)
                        {
                            unsigned period_ms = k == 3 ? x->route.PowerQueryPeriodMs : query_period_ms[k];
                            if (!period_ms || ((x->query_requested_mask & (1u << k)) && now - x->query_request_ms[k] < period_ms))
                                continue;
                            if (start_query(c, a, k + 1, 1, now, true))
                            {
                                x->query_request_ms[k] = now;
                                x->query_requested_mask |= 1u << k;
                                query_slot[c] = (slot + 1) % ENCOS_MOTORS_PER_CHANNEL;
                            }
                            break;
                        }
                        if (t->busy)
                            break;
                    }
                }
            }
        }
    }
    process_raw(now);
    unlock(irq);
}
static void publish(uint32_t now)
{
    mc_board_tx_t *b = OD(board_tx, 0x6000);
    ++b->PublishSeq;
    b->BoardStatus = board_fault | (config_ready ? 1 : 0) | (operational ? 2 : 0) | (OD(owner, 0x2003)->Owner == OWNER_SDO ? 128 : 0);
    b->AxisOnlineMask = b->AxisFaultMask = 0;
    b->ChannelOnlineMask = b->ChannelFaultMask = 0;
    rx_total = 0;
    for (unsigned c = 0; c < 4; c++)
    {
        mc_channel_tx_t *ch = OD(channel_tx, 0x6100 + c);
        memset(ch, 0, sizeof(*ch));
        uint32_t drops = ethercat_canfd_rx_drops(c) + raw_rx[c].drops;
        rx_total += drops;
        ch->ChannelStatus = (config_ready ? 1 : 0) | (channel_active(c) ? 2 : 0) | (ethercat_canfd_bus_state(c) == 3 ? 4 : 0) | (drops ? 8 : 0) | (channel_tx_drop[c] ? 16 : 0);
        ch->RxDropCount = drops > 65535 ? 65535 : drops;
        ch->TxDropCount = channel_tx_drop[c] > 65535 ? 65535 : channel_tx_drop[c];
        for (unsigned a = c * ENCOS_MOTORS_PER_CHANNEL; a < (c + 1) * ENCOS_MOTORS_PER_CHANNEL; a++)
        {
            axis_t *x = &axes[a];
            mc_axis_tx_t *s = STATE(a);
            bool type1 = x->active && x->feedback_seen;
            bool online = x->online_seen && now - x->online_ms <= (type1 ? config.FeedbackTimeoutMs : QUERY_ONLINE_MS);
            bool power_stale = !x->route.PowerQueryPeriodMs || !x->power_seen || now - x->power_ms >= x->route.PowerQueryPeriodMs * 3u;
            s->PowerAgeMs = age_ms(now, x->power_ms, x->power_seen);
            bool stale;
            if (type1)
            {
                s->FeedbackAgeMs = age_ms(now, x->feedback_ms, true);
                stale = s->FeedbackAgeMs >= config.FeedbackTimeoutMs;
                if (stale)
                    s->ValidFlags &= 64;
            }
            else
            {
                bool pos = x->position_seen && now - x->position_ms <= QUERY_POSITION_MS;
                bool vel = x->velocity_seen && now - x->velocity_ms <= QUERY_VELOCITY_MS;
                bool current = x->current_seen && now - x->current_ms <= QUERY_CURRENT_MS;
                s->ValidFlags = (pos ? 1 : 0) | (vel ? 2 : 0) | (current ? 4 : 0);
                if (current && x->profile.kt > 0)
                {
                    double torque = (double)s->ActualCurrent * x->profile.kt * 10;
                    if (torque >= INT32_MIN && torque <= INT32_MAX)
                    {
                        s->EstimatedTorque = (int32_t)torque;
                        s->ValidFlags |= 8;
                    }
                }
                s->MotorTemperature = 0;
                s->MosTemperature = 0;
                stale = !(pos && vel && current);
                if (stale)
                    s->FeedbackAgeMs = UINT16_MAX;
                else
                {
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
            if (online)
            {
                b->AxisOnlineMask |= 1u << a;
                ch->OnlineMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
            if (s->BridgeFaultCode || s->MotorFaultRaw)
            {
                b->AxisFaultMask |= 1u << a;
                ch->FaultMask |= 1u << (a % ENCOS_MOTORS_PER_CHANNEL);
            }
            DIAG(a)->PositionQueryAgeUs = age(now, x->position_ms, x->position_seen);
            DIAG(a)->VelocityQueryAgeUs = age(now, x->velocity_ms, x->velocity_seen);
            DIAG(a)->TxCompleteCount = ethercat_canfd_completions(a);
        }
        if (ch->OnlineMask)
        {
            b->ChannelOnlineMask |= 1u << c;
        }
        if (ch->FaultMask || (ch->ChannelStatus & 4))
        {
            b->ChannelFaultMask |= 1u << c;
        }
    }
    b->RxDropCount = rx_total;
    b->TxDropCount = tx_total;
    if (rx_total)
    {
        board_fault |= 16;
        b->BoardStatus |= 16;
    }
}
static unsigned mapping_indices(unsigned profile, bool tx, uint16_t *indices)
{
    unsigned n = 0;
    indices[n++] = tx ? 0x1A00 : 0x1600;
    if (profile == 2)
    {
        for (unsigned c = 0; c < 4; c++)
            indices[n++] = (tx ? 0x1A50 : 0x1650) + c;
        return n;
    }
    for (unsigned c = 0; c < 4; c++)
        indices[n++] = (tx ? 0x1A01 : 0x1601) + c;
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
        indices[n++] = (tx ? 0x1A20 : 0x1620) + a;
    if (tx)
    {
        indices[n++] = 0x1A40;
    }
    if (profile == 1)
    {
        indices[n++] = tx ? 0x1A41 : 0x1640;
    }
    return n;
}
uint16_t motor_control_mapping(uint16_t *in, uint16_t *out)
{
    static const uint16_t sizes[3][2] = {{1024, 736}, {1072, 784}, {544, 544}};
    for (unsigned p = 0; p < 3; p++)
    {
        uint16_t rx[39], tx[39];
        unsigned nr = mapping_indices(p, false, rx), nt = mapping_indices(p, true, tx);
        if (gRxAssign.u16SubIndex0 == nr && gTxAssign.u16SubIndex0 == nt && !memcmp(rx, gRxAssign.aEntries, nr * 2) && !memcmp(tx, gTxAssign.aEntries, nt * 2))
        {
            if (operational && p != config.ActivePdoProfile)
                return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            if (p == 2 && config.AxisConfiguredMask)
                return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            config.ActivePdoProfile = p;
            OD(board_config, 0x2001)->ActivePdoProfile = p;
            *in = input_size = sizes[p][0];
            *out = output_size = sizes[p][1];
            published_once = false;
            return 0;
        }
    }
    *in = *out = 0;
    return ALSTATUSCODE_INVALIDOUTPUTMAPPING;
}
void motor_control_input(void *data)
{
    uint32_t irq = lock(), now = ethercat_canfd_now_ms();
    if (published_once && now - last_publish_ms < 2)
    {
        memcpy(data, published_image, input_size);
        unlock(irq);
        return;
    }
    publish(now);
    uint8_t *p = published_image;
    memcpy(p, mc_object(0x6000), 32);
    if (config.ActivePdoProfile == 2)
    {
        for (unsigned c = 0; c < 4; c++)
        {
            canfd_bus_pdo_t *raw = mc_object(0x6500 + c * 16);
            memset(raw, 0, sizeof(*raw));
            raw->cycle_seq = board_command.CycleSeq;
            raw->bus_state = ethercat_canfd_bus_state(c);
            raw->counter0 = ethercat_canfd_rx_drops(c) + raw_rx[c].drops;
            encos_frame_t f;
            for (unsigned i = 0; i < CANFD_GATEWAY_SLOTS_PER_BUS && raw_pop(c, &f); i++)
            {
                raw->valid_mask |= 1u << i;
                raw->mode++;
                raw->slot[i].can_id = f.id;
                raw->slot[i].flags = f.flags | 1;
                raw->slot[i].dlc = f.dlc;
                raw->slot[i].seq = ++raw_batch;
                memcpy(raw->slot[i].data, f.data, 8);
            }
            memcpy(p + 32 + c * 128, raw, 128);
        }
    }
    else
    {
        for (unsigned c = 0; c < 4; c++)
            memcpy(p + 32 + c * 8, mc_object(0x6100 + c), 8);
        for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
            memcpy(p + 64 + a * 32, STATE(a), 32);
        memcpy(p + 960, mc_object(0x6300), 64); /* No connected IMU driver: ValidFlags remain zero. */
        if (config.ActivePdoProfile == 1)
        {
            mc_raw_t *raw = OD(raw, 0x6400);
            memset(raw, 0, 48);
            bool motor_only = true;
            for (unsigned c = 0; c < 4; c++)
                if (channels[c].Role >= 2)
                    motor_only = false;
            if (motor_only)
            {
                uint8_t *d = (uint8_t *)raw;
                uint8_t page = diagnostic_page++ & 1u;
                d[0] = 'T';
                d[1] = '1';
                d[2] = 'R';
                d[3] = '2';
                d[4] = page;
                for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
                    d[5 + a] = (uint8_t)(page ? axes[a].type1_count : ethercat_canfd_completions(a));
            }
            else
            {
                raw->BatchSeq = ++raw_batch;
                raw->DropCount = rx_total;
                for (unsigned i = 0; i < 2; i++)
                    for (unsigned n = 0; n < 4; n++)
                    {
                        unsigned c = round_robin;
                        round_robin = (round_robin + 1) % 4;
                        encos_frame_t f;
                        if (!raw_pop(c, &f))
                            continue;
                        uint8_t *s = (uint8_t *)raw + 16 + 16 * i;
                        memcpy(s, &f.id, 4);
                        s[4] = c;
                        s[5] = f.flags;
                        s[6] = f.dlc;
                        memcpy(s + 8, f.data, 8);
                        raw->ValidMask |= 1u << i;
                        raw->SlotCount++;
                        break;
                    }
            }
            memcpy(p + 1024, raw, 48);
        }
    }
    last_publish_ms = now;
    published_once = true;
    memcpy(data, published_image, input_size);
    unlock(irq);
}
UINT8 MC_AssignWrite(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    if (!preop())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    TMC_ASSIGN *v = index == 0x1C12 ? &gRxAssign : &gTxAssign;
    if (sub > 39)
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (bytes != (sub ? 2u : 1u))
        return ABORTIDX_PARAM_LENGTH_ERROR;
    uint16_t value = 0;
    memcpy(&value, data, bytes);
    if (!sub)
    {
        if (value > 39)
            return ABORTIDX_VALUE_EXCEEDED;
        v->u16SubIndex0 = value;
    }
    else
    {
        if (v->u16SubIndex0)
            return ABORTIDX_ENTRY_CANT_BE_WRITTEN_SI0_NOT_0;
        v->aEntries[sub - 1] = value;
    }
    return 0;
}
UINT8 MC_OD_Read(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    mc_object_t *o = find_object(index);
    if (!o)
        return ABORTIDX_OBJECT_NOT_EXISTING;
    if (!sub)
    {
        if (bytes != 1)
            return ABORTIDX_PARAM_LENGTH_ERROR;
        *((uint8_t *)data) = (uint8_t)o->count;
        return 0;
    }
    for (unsigned i = 0; i < mc_field_count; i++)
    {
        const mc_field_t *f = &mc_fields[i];
        if (f->index != index || f->sub != sub)
            continue;
        if (!(f->access & 1))
            return ABORTIDX_WRITE_ONLY_ENTRY;
        if (bytes != f->bytes)
            return ABORTIDX_PARAM_LENGTH_ERROR;
        uint32_t irq = lock();
        memcpy(data, o->data + f->offset, bytes);
        unlock(irq);
        return 0;
    }
    return ABORTIDX_SUBINDEX_NOT_EXISTING;
}
static UINT8 commit_service(unsigned a, bool maintenance, uint32_t now)
{
    unsigned c = a / ENCOS_MOTORS_PER_CHANNEL;
    service_t *t = &services[c];
    if (t->busy || (int32_t)(now - t->hold_until) < 0 || channel_active(c))
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (maintenance)
    {
        mc_maintenance_t *m = OD(maintenance, BASE(a) + 8);
        if (m->RequestSeq == m->DoneSeq)
            return 0;
        if (!preop() || OD(owner, 0x2003)->Owner != OWNER_SDO)
        {
            m->Result = RESULT_STATE;
            m->DoneSeq = m->RequestSeq;
            return 0;
        }
        if (m->Opcode != 3 || m->Argument0 || m->Argument1)
        {
            m->Result = RESULT_UNSUPPORTED;
            m->DoneSeq = m->RequestSeq;
            return 0;
        }
        mc_routing_t *r = OD(routing, BASE(a));
        if (r->CanId < 1 || r->CanId > 8)
        {
            m->Result = RESULT_INVALID;
            m->DoneSeq = m->RequestSeq;
            return 0;
        }
        uint32_t hold = t->hold_until;
        memset(t, 0, sizeof(*t));
        t->hold_until = hold;
        t->axis = a;
        t->seq = m->RequestSeq;
        t->maintenance = true;
        t->started = now;
        t->request.id = 0x7FF;
        t->request.dlc = 4;
        t->request.flags = OD(channel_config, 0x2010 + c)->FrameFormat ? 24 : 0;
        t->request.data[0] = r->CanId >> 8;
        t->request.data[1] = r->CanId;
        t->request.data[2] = 0;
        t->request.data[3] = 3;
        t->busy = true;
        m->Result = RESULT_BUSY;
        return 0;
    }
    mc_transaction_t *tr = OD(transaction, BASE(a) + 7);
    uint16_t pid = tr->ParameterId;
    uint8_t query = encos_parameter_query(pid), op = tr->Operation;
    if (tr->RequestSeq == tr->DoneSeq)
        return 0;
    if (!preop() || OD(owner, 0x2003)->Owner != OWNER_SDO)
    {
        tr->Result = RESULT_STATE;
        tr->DoneSeq = tr->RequestSeq;
        return 0;
    }
    if (op < 1 || op > 3 || (!query && !(op == 2 && pid == 2)))
    {
        tr->Result = RESULT_UNSUPPORTED;
        tr->DoneSeq = tr->RequestSeq;
        return 0;
    }
    mc_routing_t *r = OD(routing, BASE(a));
    if (r->CanId < 1 || r->CanId > 8)
    {
        tr->Result = RESULT_INVALID;
        tr->DoneSeq = tr->RequestSeq;
        return 0;
    }
    uint32_t hold = t->hold_until;
    memset(t, 0, sizeof(*t));
    t->hold_until = hold;
    t->axis = a;
    t->seq = tr->RequestSeq;
    t->parameter = pid;
    t->query = query;
    t->group = OD(kt, BASE(a) + 4)->Group;
    t->started = now;
    t->operation = op;
    t->request.id = r->CanId;
    bool ok;
    if (op == 2)
    {
        ok = encos_parameter(pid, OD(basic, BASE(a) + 1), OD(range, BASE(a) + 2), OD(gain, BASE(a) + 3), OD(kt, BASE(a) + 4), &t->request);
        t->writing = true;
        t->expected_size = t->request.dlc - 2;
        memcpy(t->expected, t->request.data + 2, t->expected_size);
    }
    else
        ok = encos_query(query, t->group, &t->request);
    if (!ok)
    {
        tr->Result = RESULT_UNSUPPORTED;
        tr->DoneSeq = tr->RequestSeq;
        return 0;
    }
    t->request.flags = OD(channel_config, 0x2010 + c)->FrameFormat ? 24 : 0;
    t->busy = true;
    tr->Result = RESULT_BUSY;
    return 0;
}
UINT8 MC_OD_Write(UINT16 index, UINT8 sub, UINT32 bytes, UINT16 *data, UINT8 complete)
{
    if (complete)
        return ABORTIDX_COMPLETE_ACCESS_NOT_SUPPORTED;
    mc_object_t *o = find_object(index);
    if (!o)
        return ABORTIDX_OBJECT_NOT_EXISTING;
    if (!sub)
        return ABORTIDX_READ_ONLY_ENTRY;
    const mc_field_t *field = NULL;
    for (unsigned i = 0; i < mc_field_count; i++)
        if (mc_fields[i].index == index && mc_fields[i].sub == sub)
        {
            field = &mc_fields[i];
            break;
        }
    if (!field)
        return ABORTIDX_SUBINDEX_NOT_EXISTING;
    if (!(field->access & 2))
        return ABORTIDX_READ_ONLY_ENTRY;
    if (bytes != field->bytes)
        return ABORTIDX_PARAM_LENGTH_ERROR;
    bool command = index >= 0x2200 && index <= 0x23B0, owner = index == 0x2003;
    if (!command && !owner && !preop())
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (owner && (any_active() || any_service()))
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    if (index >= 0x3000 && index <= 0x31B8 && services[((index - 0x3000) / 16) / ENCOS_MOTORS_PER_CHANNEL].busy)
        return ABORTIDX_IN_THIS_STATE_DATA_CANNOT_BE_READ_OR_STORED;
    uint32_t irq = lock();
    memcpy(o->data + field->offset, data, bytes);
    uint32_t now = ethercat_canfd_now_ms();
    UINT8 result = 0;
    if (index == 0x2003)
    {
        mc_owner_t *v = OD(owner, index);
        if (v->RequestedOwner > 2)
        {
            result = ABORTIDX_VALUE_EXCEEDED;
            v->Result = RESULT_INVALID;
        }
        else
        {
            stop_all(0);
            v->Owner = v->RequestedOwner;
            v->Result = 0;
            resume_pdo_owner = false;
        }
    }
    else if (index == 0x2002)
    {
        mc_action_t *v = OD(action, index);
        if (sub == 1)
        {
            v->Result = RESULT_BUSY;
            if (apply_config())
            {
                v->AppliedGeneration = v->ApplyGeneration;
                v->Result = 0;
                OD(board_info, 0x2000)->ConfigGeneration = v->AppliedGeneration;
            }
            else
                v->Result = RESULT_INVALID;
        }
        if (sub == 2)
            v->Result = RESULT_UNSUPPORTED;
    }
    else if (command && sub == 11)
    {
        unsigned a = (index - 0x2200) / 16;
        mc_sdo_command_t *v = OD(sdo_command, index);
        if (v->CommitSeq != v->AcceptedSeq)
        {
            bool gate = operational && session_armed && seen_cycle && (board_command.BoardControl & 1) && !(board_command.BoardControl & 2) && now - last_command_ms < board_command.CommandLeaseMs;
            if (OD(owner, 0x2003)->Owner != OWNER_SDO || !gate)
            {
                v->CommandResult = RESULT_STATE;
                stop_axis(a, 8);
            }
            else if (!v->LeaseMs || v->LeaseMs > config.MaxCommandLeaseMs || !newer(v->CommitSeq, v->AcceptedSeq))
            {
                v->CommandResult = RESULT_INVALID;
                stop_axis(a, 2);
            }
            else
            {
                bool ok = accept_axis(a, (mc_axis_rx_t *)v, true, now);
                v->CommandResult = ok ? RESULT_OK : RESULT_INVALID;
                if (ok)
                {
                    v->AcceptedSeq = v->CommitSeq;
                    axes[a].lease_ms = v->LeaseMs;
                    axes[a].commit_ms = now;
                }
            }
        }
    }
    else if (index >= 0x3000)
    {
        unsigned a = (index - 0x3000) / 16, part = (index - 0x3000) % 16;
        if ((part == 7 && sub == 3) || (part == 8 && sub == 4))
            result = commit_service(a, part == 8, now);
    }
    unlock(irq);
    return result;
}
void motor_control_background(void) { motor_control_process(); }
void motor_control_init(void)
{
    for (unsigned a = 0; a < ENCOS_AXIS_COUNT; a++)
    {
        OD(routing, BASE(a))->CanId = a % ENCOS_MOTORS_PER_CHANNEL + 1;
        STATE(a)->FeedbackAgeMs = STATE(a)->PowerAgeMs = UINT16_MAX;
        DIAG(a)->PositionQueryAgeUs = DIAG(a)->VelocityQueryAgeUs = UINT32_MAX;
    }
    for (unsigned c = 0; c < 4; c++)
        channels[c] = *OD(channel_config, 0x2010 + c);
    OD(board_info, 0x2000)->Capabilities = 7; /* Three PDO profiles; hardware validation still required. */
}
