#include "detail/gateway_application_impl.hpp"
#include "can/detail/raw_can_submit.hpp"
#include "can/cfg/gateway_schedule_policy.hpp"
#include "motor_driver_binding.hpp"
#include "motor_protocol.h"
#include "can_transport.h"
#include "first_pdo_trace.h"
#include "realtime_port.h"

namespace dcu::app::gateway_application::detail {
namespace policy = dcu::app::config::gateway_schedule;
using motor::RESULT_IO;
using motor::RESULT_STATE;

namespace {
#if defined(CONFIG_CAN_BATCH_SAMPLE)
struct BatchSample {
    uint32_t sequence, preview, deadline_us, queued_us, tx_eof_us, rx_eof_us, matched_us, encoded_us;

    void begin(uint32_t seq, bool prepacked, uint32_t deadline) volatile {
        sequence = seq;
        preview = prepacked;
        deadline_us = deadline;
        queued_us = tx_eof_us = 0;
        rx_eof_us = matched_us = 0;
    }
    void match(uint32_t eof_us) volatile {
        rx_eof_us = eof_us;
        matched_us = dcu_time_now_us();
    }
};
volatile BatchSample s_batch_sample[DCU_MOTOR_DEVICE_CAPACITY] __attribute__((section(".noncacheable.bss")));
#define DCU_BATCH_SAMPLE(axis, observation) (s_batch_sample[(axis)].observation)
inline void capture_batch_time(uint32_t &us) { us = dcu_time_now_us(); }
#else
#define DCU_BATCH_SAMPLE(axis, observation) ((void)0)
inline void capture_batch_time(uint32_t &) {}
#endif

bool same_command(const motor::Command &a, const motor::Command &b)
{
    return a.mode == b.mode && a.control_bits == b.control_bits && a.reserved_flags == b.reserved_flags &&
        a.position_urad == b.position_urad && a.velocity_urad_s == b.velocity_urad_s && a.torque_mNm == b.torque_mNm &&
        a.current_cA == b.current_cA && a.current_limit_dA == b.current_limit_dA &&
        a.kp_centi == b.kp_centi && a.kd_milli == b.kd_milli;
}
bool newer(uint32_t a, uint32_t b)
{
    uint32_t d = a - b;
    return d && d < 0x80000000u;
}

motor::MotorConfiguration motor_configuration(const dcu_motor_device_profile_t &device)
{
    motor::MotorConfiguration motor{};
    motor.route = {device.can_id, device.position_min_urad, device.position_max_urad,
                   device.velocity_limit_urad_s, device.torque_limit_mNm, device.current_limit_dA};
    motor.encos = device.encos;
    motor.model_profile_id = device.model_profile_id;
    motor.power_query_period_ms = device.power_query_period_ms;
    motor.bus = device.bus;
    motor.status_poll_period_ms = device.status_poll_period_ms;
    motor.status_reply_timeout_ms = device.status_reply_timeout_ms;
    motor.protocol = device.protocol;
    motor.neutral_mit_only = device.neutral_mit_only;
    motor.zero_mit_loopback = device.zero_mit_loopback;
    motor.type = device.type;
    return motor;
}

}

unsigned GatewayApplication::frame_us(unsigned c, unsigned bytes) const
{
    return dcu::app::BusScheduler::frame_cost(state_.channels[c].frame_format != 0, bytes);
}
ControlSet GatewayApplication::controls_on_bus(unsigned c) const
{
    dcu::app::ControlSet controls{};
    controls.period_us = state_.channel_command[c].period_us;
    for (unsigned slot = 0; slot < bus_device_count_[c]; ++slot) {
        const unsigned a = devices_on_bus_[c][slot];
        if (cyclic_.required & (1U << a)) continue;
        auto &device = devices_[a];
        if (!device.active()) continue;
        controls.active_mask |= 1u << slot;
        if (!device.control_sent()) controls.first_send_mask |= 1u << slot;
    }
    return controls;
}
bool GatewayApplication::any_active() const
{
    for (unsigned a = 0; a < device_count_; a++)
        if (devices_[a].active())
            return true;
    return false;
}
bool GatewayApplication::channel_active(unsigned c) const
{
    for (unsigned j = 0; j < bus_device_count_[c]; ++j)
        if (devices_[devices_on_bus_[c][j]].active())
            return true;
    return false;
}
bool GatewayApplication::command_permitted(uint32_t now) const
{
    return startup_.running() && state_.config_ready && state_.operational && state_.session_armed && state_.seen_cycle &&
        (state_.board_command.control_bits & 1) && !(state_.board_command.control_bits & 2) &&
        now - state_.last_command_ms < state_.board_command.lease_ms;
}
void GatewayApplication::stop_all(uint16_t fault)
{
    preview_.mask = 0;
    cyclic_.required = cyclic_.submitted = cyclic_.completed = cyclic_.matched = 0;
    cyclic_.buses = 0;
    stop_neutral_run(fault != 0);
    for (unsigned a = 0; a < device_count_; a++)
        devices_[a].stop(fault);
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; c++) {
        dcu_can_cancel(c);
    }
    state_.session_armed = false;
}

void GatewayApplication::stop_neutral_run(bool fault)
{
    if (!neutral_run_.active) return;
    for (unsigned a = 0; a < device_count_; ++a) {
        if (!(neutral_run_.active & (1U << a))) continue;
        transactions_.cancel_neutral(neutral_run_.bus, a);
        devices_[a].abort_neutral_mit(fault ? 6 : 0);
    }
    neutral_run_.closed |= neutral_run_.active; // Reconnect does not grant another run.
    neutral_run_.active = neutral_run_.awaiting = 0;
    if (fault) state_.board_fault |= 32U;
}

void GatewayApplication::set_neutral_window(uint32_t period_ns, uint32_t cutoff_us)
{
    neutral_run_.period_ns = period_ns;
    neutral_run_.cutoff_us = cutoff_us;
}

void GatewayApplication::set_cyclic_feedback_window(uint32_t period_us, uint32_t deadline_us)
{
    for (auto &slots : cyclic_.queued) slots = 0;
    cyclic_.period_us = period_us;
    cyclic_.deadline_us = deadline_us;
    cyclic_.required = cyclic_.submitted = cyclic_.completed = cyclic_.matched = 0;
    cyclic_.buses = 0;
}

void GatewayApplication::preview_axis(unsigned a, const motor::Command &command)
{
    if (a < device_count_ && (timestamped_encos_ & (1U << a)) &&
        command.mode == motor::ControlMode::Impedance && devices_[a].preview_control(command, preview_.frames[a])) {
        preview_.commands[a] = command;
        capture_batch_time(preview_.encoded_us[a]);
        preview_.mask |= 1U << a;
    }
}

void GatewayApplication::fault_cyclic_feedback(uint32_t missing)
{
    for (unsigned a = 0; a < device_count_; ++a) {
        if (cyclic_.required & (1U << a)) devices_[a].acknowledge_publication();
        if (missing & (1U << a)) devices_[a].stop(4); // Existing per-axis feedback timeout.
    }
}

bool GatewayApplication::neutral_window_open(uint32_t now_us) const
{
    return neutral_run_.period_ns == policy::neutral_mit_period_us * 1000U &&
           static_cast<int32_t>(neutral_run_.cutoff_us - now_us) > 0;
}

void GatewayApplication::service_neutral_run(unsigned bus, uint32_t now, bool drained)
{
    if (!neutral_run_.active || neutral_run_.bus != bus) return;
    if (!drained || dcu_can_bus_state(bus) != DCU_CAN_BUS_ERROR_ACTIVE || dcu_can_rx_drops(bus)) {
        stop_neutral_run(true);
        return;
    }
    for (unsigned slot = 0; slot < bus_device_count_[bus]; ++slot) {
        const unsigned a = devices_on_bus_[bus][slot];
        const uint32_t bit = 1U << a;
        if (!(neutral_run_.active & bit)) continue;
        if ((neutral_run_.awaiting & bit) && !transactions_.neutral_pending(bus, a)) {
            stop_neutral_run(true);
            return;
        }
        if (!devices_[a].neutral_mit_requested()) continue;
        const uint32_t submit_us = dcu_time_now_us();
        if (!command_permitted(now) || !neutral_window_open(submit_us) ||
            neutral_run_.cutoff_us - submit_us < policy::neutral_mit_reply_us ||
            (neutral_run_.awaiting & bit) || neutral_run_.submitted[a] >= neutral_run_.limit[a]) {
            stop_neutral_run(true);
            return;
        }
        if (!dcu_can_tx_serial_ready(bus)) continue;
        if (!transactions_.submit_neutral(bus, a, devices_[a], schedule_[bus], now, submit_us,
                                          static_cast<uint8_t>(neutral_run_.submitted[a]), true)) {
            stop_neutral_run(true);
            return;
        }
        ++neutral_run_.submitted[a];
        neutral_run_.awaiting |= bit;
        neutral_run_.sent_this_cycle |= bit;
    }
}

bool GatewayApplication::boot_startup(const dcu_gateway_profile_t &profile, uint64_t boot_id)
{
    return state_.config_ready && startup_.boot(profile, boot_id, dcu_time_now_ms());
}

profile_update::Policy GatewayApplication::profile_policy() const
{
    return {state_.config.max_command_lease_ms, state_.config.feedback_timeout_ms, startup_.readiness_ttl_ms()};
}

bool GatewayApplication::apply_profile_policy(uint32_t profile, uint32_t base, uint32_t revision,
                                               const profile_update::Policy &policy)
{
    if (!state_.config_ready || layout_.topology || !preop_access_allowed() || !profile_update::valid(policy) ||
        !startup_.apply_policy(profile, base, revision, policy.readiness_ttl_ms, dcu_time_now_ms()))
        return false;
    revoke_business();
    layout_fault();
    state_.config = {policy.max_command_lease_ms, policy.feedback_timeout_ms};
    return true;
}

void GatewayApplication::revoke_business()
{
    stop_all(0);
    transactions_.reset_session(RESULT_STATE);
    raw_.reset_session();
    state_.owner = OWNER_NONE;
    state_.seen_cycle = false;
    state_.board_command = {};
    state_.last_cycle = 0;
    for (unsigned i = 0; i < device_count_; ++i) devices_[i].reset_session();
}

void GatewayApplication::step_startup(uint32_t now)
{
    if (startup_.running() && state_.seen_cycle &&
        now - state_.last_command_ms >= state_.board_command.lease_ms) {
        layout_fault();
        startup_.invalidate(DCU_GATEWAY_REASON_MASTER_TIMEOUT, now);
        state_.board_fault |= 4;
    }
    bool healthy = true;
    for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus)
        if (state_.channels[bus].enabled && dcu_can_bus_state(bus) >= DCU_CAN_BUS_OFF)
            healthy = false;
    const auto before = startup_.status().state;
    if (state_.operational && layout_.running && before != DCU_GATEWAY_RUNNING) layout_fault();
    if (before == DCU_GATEWAY_HOLD && healthy) revoke_business();
    // A neutral-only profile gates local transport, not readiness to move.
    startup_.step(now, raw_only(state_.mode) ? 0 : readiness_devices_, healthy);
    if (state_.operational && startup_.resume()) state_.owner = OWNER_PDO;
    const auto status = startup_.status();
    const auto after = status.state;
    if (state_.operational && (layout_.state == RawState::Active || layout_.switching) &&
        (status.session_id != layout_.session || after == DCU_GATEWAY_FAULT ||
         after == DCU_GATEWAY_HOLD || (before == DCU_GATEWAY_RUNNING && after != before)))
        layout_fault();
    layout_.running = state_.operational && after == DCU_GATEWAY_RUNNING;
    if ((state_.owner != OWNER_NONE && after != DCU_GATEWAY_RUNNING) ||
        (before != after && (after == DCU_GATEWAY_HOLD || after == DCU_GATEWAY_FAULT)))
        revoke_business();
}

void GatewayApplication::operational(bool op)
{
    if (!op && state_.operational) {
        layout_.motor_owner = 0;
        // 计划内退 OP 不取消协商；其他退站视为旧透传会话终结。
        if (!layout_.switching) layout_fault();
        raw_.discard_frames();
        startup_.invalidate(DCU_GATEWAY_REASON_MASTER_TIMEOUT, dcu_time_now_ms());
    } else if (op && !state_.operational) {
        layout_.reset = false;
        layout_.running = false;
        layout_.started = true;
        if (layout_.state == RawState::Armed) activate_layout();
    }
    state_.operational = op;
    if (!op) revoke_business();
}
void GatewayApplication::cycle_fault()
{
    layout_fault();
    if (startup_.enforced()) startup_.invalidate(DCU_GATEWAY_REASON_PROTOCOL_MISMATCH, dcu_time_now_ms());
    operational(false);
    /* Keep the existing invalid-PDO bit understood by the master's 0xfc mask.
     * Detailed cycle causes remain in cycle_diagnostics. */
    state_.board_fault |= 64U;
}
void GatewayApplication::check_timeouts(uint32_t now)
{
    if (!state_.operational) return;
    if (state_.seen_cycle && uint32_t(now - state_.last_command_ms) >= state_.board_command.lease_ms) {
        layout_fault();
        state_.board_fault |= 4;
        stop_all(3);
    }
    if (raw_only(state_.mode)) return;
    const bool sdo_owner = state_.owner == OWNER_SDO;
    for (unsigned i = 0; i < device_count_; ++i)
        devices_[i].check_timeouts(now, sdo_owner, state_.config.feedback_timeout_ms);
}
DCU_FAST_CODE bool GatewayApplication::accept_candidate(const CycleCommand &candidate, uint32_t now, ProcessResult &result)
{
    if (state_.mode == OperatingMode::SingleFd) {
        const auto &batch = candidate.raw[0];
        // 无板级控制头：先用新的空批次建会话，再按 BatchSeq 续租。
        if (!state_.operational || !startup_.running() || layout_.state != RawState::Active ||
            batch.kind != RawCanBatch::Kind::SingleFd || !batch.valid || batch.count > batch.capacity() ||
            (!state_.session_armed && (!batch.sequence || batch.count)) ||
            (state_.seen_cycle && (!newer(batch.sequence, state_.last_cycle) ||
             now - state_.last_command_ms >= state_.config.max_command_lease_ms))) return false;
        state_.seen_cycle = state_.session_armed = true;
        state_.last_cycle = batch.sequence;
        state_.last_command_ms = now;
        state_.board_command.lease_ms = state_.config.max_command_lease_ms;
        return true;
    }
    const auto &b = candidate.board;
    if (startup_.enforced()) {
        if (!state_.operational || !startup_.running()) return false;
        // A stale enabled PDO must not establish the sequence baseline after reconnect.
        if (!state_.session_armed &&
            (!b.layout_valid || !b.sequence || !b.lease_ms || b.lease_ms > state_.config.max_command_lease_ms ||
             (b.control_bits & ~6u) || b.enabled_channels || b.enabled_devices || b.unmapped_device_requested))
            return false;
    }
    /* Stop intent always wins, including duplicate sequence or malformed targets. */
    bool stop = !(b.control_bits & 1) || (b.control_bits & 2);
    const bool next_cycle = state_.seen_cycle && b.sequence == state_.last_cycle + 1U;
    if (stop)
        stop_all(0);
    if (neutral_run_.active && (!next_cycle || neutral_feedback_pending())) {
        stop_neutral_run(true);
        return false;
    }
    if (state_.mode != OperatingMode::Raw)
        for (unsigned a = 0; a < device_count_; a++) {
            const auto &command = candidate.devices[a];
            const unsigned bus = devices_[a].bus();
            const auto &channel = candidate.channels[bus];
            if (!(command.control_bits & motor::Arm) || (command.control_bits & motor::Stop) ||
                !(channel.control_bits & 1) || (channel.control_bits & 2) ||
                !(b.enabled_channels & (1u << bus)) || !(b.enabled_devices & candidate.active_devices & (1u << a))) {
                devices_[a].stop(0);
                if (neutral_run_.active & (1U << a)) stop_neutral_run(true);
            }
        }
    unsigned enabled_channels = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c)
        if (state_.channels[c].enabled) enabled_channels |= 1u << c;
    if (!b.layout_valid || (b.control_bits & ~7u) || (b.enabled_channels & ~enabled_channels) || !b.lease_ms || b.lease_ms > state_.config.max_command_lease_ms) {
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
    if (stop && !b.enabled_devices && !b.unmapped_device_requested)
        state_.session_armed = true;
    if ((b.control_bits & 4) && !(state_.board_command.control_bits & 4) && stop)
        state_.board_fault = 0;
    state_.last_cycle = b.sequence;
    state_.seen_cycle = true;
    state_.last_command_ms = now;
    state_.board_command = b;
    if (state_.mode == OperatingMode::Raw || (startup_.enforced() && stop)) {
        return true;
    }
    neutral_run_.sent_this_cycle = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; c++) {
        const auto &ch = candidate.channels[c];
        state_.channel_command[c] = ch;
        bool valid = !(ch.control_bits & ~3u) && ch.active_mask_valid && ch.period_us >= 1000u && ch.period_us <= 10000u && ch.period_us % 1000u == 0 && ch.expected_role == state_.channels[c].role;
        for (unsigned j = 0; j < bus_device_count_[c]; ++j) {
            const unsigned a = devices_on_bus_[c][j];
            const auto &cmd = candidate.devices[a];
            bool permit = valid && !stop && (b.enabled_channels & (1u << c)) && (b.enabled_devices & (1u << a)) && (ch.control_bits & 1) && !(ch.control_bits & 2) && (candidate.active_devices & (1u << a));
            if (devices_[a].neutral_mit_only()) {
                const uint32_t bit = 1U << a;
                if (neutral_run_.closed & bit) continue;
                const bool allowed = next_cycle && state_.owner == OWNER_PDO && permit && !b.unmapped_device_requested &&
                    ch.period_us == policy::neutral_mit_period_us && neutral_window_open(dcu_time_now_us());
                const bool accepted = devices_[a].accept_neutral_mit(cmd, allowed, neutral_run_.active & bit);
                if (accepted && devices_[a].neutral_mit_requested())
                    neutral_run_.active |= bit;
                else if (neutral_run_.active & bit)
                    stop_neutral_run(true);
                continue;
            }
            if (!valid) {
                devices_[a].stop(7);
                continue;
            }
            if (state_.owner == OWNER_PDO) {
                if (!raw_only(state_.mode) && permit && cyclic_.period_us == ch.period_us &&
                    (timestamped_encos_ & (1U << a)) && cmd.mode == motor::ControlMode::Impedance &&
                    (cmd.control_bits & (motor::Arm | motor::StartMotion | motor::Stop)) ==
                        (motor::Arm | motor::StartMotion)) {
                    cyclic_.required |= 1U << a;
                    cyclic_.buses |= 1U << c;
                    cyclic_.tx_eof[a] = {};
                }
                const bool accepted = devices_[a].accept(cmd, permit, now);
                const bool active = devices_[a].active();
                if (accepted && active && (cyclic_.required & (1U << a))) {
                    auto &frame = cyclic_.frames[a];
                    // A first motion request may have been replaced by a position hold.
                    const bool prepacked = preview_.sequence == b.sequence && (preview_.mask & (1U << a)) &&
                        same_command(preview_.commands[a], devices_[a].effective_control());
                    DCU_BATCH_SAMPLE(a, begin(b.sequence, prepacked, cyclic_.deadline_us));
                    if (prepacked)
                        frame = preview_.frames[a];
                    else if (!devices_[a].encode_control(frame)) {
                        devices_[a].stop(2);
                        continue;
                    }
                    frame.flags = state_.channels[c].frame_format ? DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS : 0;
                    DCU_BATCH_SAMPLE(a, encoded_us = prepacked ? preview_.encoded_us[a] : dcu_time_now_us());
                    cyclic_.queued[c] |= 1U << j;
                }
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
                if (c == 2)
                    DCU_FIRST_TRACE(DCU_FIRST_AXIS_ACCEPT, devices_[a].can_id(), permit, accepted, active);
#endif
                if (accepted && active) result.mirrored_devices |= 1u << a;
            }
        }
    }
    return true;
}
bool GatewayApplication::initialize(const Configuration &configuration)
{
    const auto *b = &configuration.board;
    if (state_.config_ready || b->max_command_lease_ms < 2 || b->max_command_lease_ms > 10 ||
        b->feedback_timeout_ms < 5 || b->feedback_timeout_ms > 20)
        return false;
    if (configuration.device_count > DCU_MOTOR_DEVICE_CAPACITY ||
        (configuration.device_count && !configuration.devices))
        return false;
    for (const auto &channel : configuration.channels) {
        if (channel.role > 3 || channel.frame_format > 1 || channel.raw_rate_limit > 8 ||
            (!channel.enabled && channel.role))
            return false;
    }
    unsigned neutral_bus = DCU_DEVICE_CAN_BUS_COUNT;
    uint8_t counts[DCU_DEVICE_CAN_BUS_COUNT]{};
    for (unsigned a = 0; a < configuration.device_count; ++a) {
        const auto &device = configuration.devices[a];
        const unsigned bus = device.bus;
        if (bus >= DCU_DEVICE_CAN_BUS_COUNT || ++counts[bus] > DCU_MOTOR_DEVICES_PER_BUS)
            return false;
        if (device.zero_mit_loopback) {
            const auto *applied = dcu_can_channel_config(bus);
            if (!applied || !applied->timestamp_eof) return false;
        }
        if (device.neutral_mit_only) {
            const auto *applied = dcu_can_channel_config(bus);
            if ((neutral_bus != DCU_DEVICE_CAN_BUS_COUNT && bus != neutral_bus) || !applied || !applied->timestamp_eof ||
                device.neutral_mit_frame_limit < 2 || device.neutral_mit_frame_limit > DCU_NEUTRAL_MIT_MAX_FRAMES)
                return false;
            neutral_bus = bus;
        } else if (device.neutral_mit_frame_limit)
            return false;
        const auto &channel = configuration.channels[bus];
        if (!channel.enabled || !(channel.role & 1) ||
            !motor::validate_motor_configuration(motor_configuration(device), channel.frame_format))
            return false;
        for (unsigned previous = 0; previous < a; ++previous)
            if (configuration.devices[previous].bus == bus &&
                configuration.devices[previous].can_id == device.can_id)
                return false;
    }
    neutral_run_.bus = neutral_bus;
    readiness_devices_ = 0;
    state_.last_process_us = UINT32_MAX;
    state_.config = *b;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        state_.channels[c] = catalogue_channels_[c] = configuration.channels[c];
        layout_.limits[c] = configuration.channels[c].raw_rate_limit;
        schedule_[c].reset_budget(dcu_time_now_ms());
    }
    for (unsigned a = 0; a < configuration.device_count; ++a) {
        const auto &device = configuration.devices[a];
        if (!devices_[a].bind(a, motor_configuration(device), state_.channels[device.bus].frame_format)) {
            stop_all(1);
            for (unsigned i = 0; i < device_count_; ++i) devices_[i].disable();
            device_count_ = 0;
            for (auto &count : bus_device_count_) count = 0;
            return false;
        }
        transactions_.bind_slot(a, bus_device_count_[device.bus]);
        devices_on_bus_[device.bus][bus_device_count_[device.bus]++] = a;
        const auto *channel = dcu_can_channel_config(device.bus);
        if (device.protocol == DCU_MOTOR_PROTOCOL_ENCOS && channel && channel->timestamp_eof)
            timestamped_encos_ |= 1U << a;
        if (device.zero_mit_loopback) loopback_devices_ |= 1U << a;
        if (device.neutral_mit_only) neutral_run_.limit[a] = device.neutral_mit_frame_limit;
        else readiness_devices_ |= 1U << a;
        ++device_count_;
    }
    state_.config_ready = true;
    return true;
}

bool GatewayApplication::commit_topology(const dcu_production_profile_t &catalogue, uint32_t buses, uint32_t enabled)
{
    if (!layout_.topology || !layout_.switching || device_count_ != catalogue.device_count ||
        (buses >> DCU_DEVICE_CAN_BUS_COUNT) || (enabled >> device_count_)) return false;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c)
        if ((buses & (1U << c)) && !catalogue_channels_[c].enabled) return false;
    for (unsigned a = 0; a < device_count_; ++a)
        if ((enabled & (1U << a)) && !(buses & (1U << catalogue.devices[a].bus))) return false;
    if (!finish_layout(OperatingMode::MotorWithRaw, true)) return false;
    // Only this task can submit CAN. Rebind the approved drivers before releasing it;
    // the neutral command budget/closed bits belong to the boot, not to the selection.
    readiness_devices_ = timestamped_encos_ = 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        auto &channel = state_.channels[c];
        channel = (buses & (1U << c)) ? catalogue_channels_[c] : ChannelConfiguration{};
        if (channel.role == 1) channel.role = 0;
        layout_.limits[c] = channel.raw_rate_limit;
        bus_device_count_[c] = state_.query_cursor[c] = 0;
        schedule_[c].reset_budget(dcu_time_now_ms());
    }
    for (unsigned a = 0; a < device_count_; ++a) {
        if (!(enabled & (1U << a))) { devices_[a].disable(); continue; }
        const auto &d = catalogue.devices[a];
        auto &channel = state_.channels[d.bus];
        channel.role = catalogue_channels_[d.bus].role;
        if (!devices_[a].bind(a, motor_configuration(d), channel.frame_format)) {
            layout_.topology = true;
            layout_fault();
            return false;
        }
        transactions_.bind_slot(a, bus_device_count_[d.bus]);
        devices_on_bus_[d.bus][bus_device_count_[d.bus]++] = a;
        if (!d.neutral_mit_only) readiness_devices_ |= 1U << a;
        if (d.protocol == DCU_MOTOR_PROTOCOL_ENCOS && dcu_can_channel_config(d.bus)->timestamp_eof)
            timestamped_encos_ |= 1U << a;
    }
    startup_.invalidate(DCU_GATEWAY_REASON_MASTER_TIMEOUT, dcu_time_now_ms());
    return true;
}
DCU_FAST_CODE bool GatewayApplication::consume_feedback(unsigned c, const dcu_can_frame_t &frame,
                                          const dcu_can_rx_stamp_t &stamp, uint32_t now, uint32_t now_us)
{
    const auto format = transactions_.reply_format(c, frame.id, state_.channels[c].frame_format);
    constexpr uint8_t fd_brs = DCU_CAN_FLAG_FD | DCU_CAN_FLAG_BRS;
    if (frame.flags != (format ? fd_brs : 0) || !dcu_can_frame_valid(&frame)) return false;
    const auto bytes = dcu_can_dlc_to_bytes(frame.dlc, format != 0);
    if (transactions_.consume_broadcast_reply(c, frame, bytes)) return true;

    for (unsigned j = 0; j < bus_device_count_[c]; ++j) {
        const unsigned index = devices_on_bus_[c][j];
        auto &device = devices_[index];
        if (!device.matches(frame.id) || !device.accepts_payload(bytes)) continue;
        const uint32_t bit = 1U << index;
        if (cyclic_.required & bit) {
            const auto tx = cyclic_.tx_eof[index];
            const uint32_t delta = stamp.eof.tick - tx.tick;
            const uint32_t lead_us = cyclic_.deadline_us - stamp.eof.us;
            const auto before = device.cyclic_feedback_count();
            const bool matched = !(cyclic_.matched & bit) && tx.valid && stamp.eof.valid &&
                delta && delta < 0x80000000U && lead_us != 0 && lead_us <= cyclic_.period_us &&
                device.consume_feedback(frame, stamp.observed_ms, now) && device.active() &&
                device.cyclic_feedback_count() != before;
            if (matched) {
                cyclic_.matched |= bit;
                DCU_BATCH_SAMPLE(index, match(stamp.eof.us));
            }
            if (matched && startup_.enforced()) startup_.observe(index, stamp.observed_ms, now);
            if (c == 2)
                DCU_FIRST_TRACE(DCU_FIRST_RX_CONSUMED, frame.id, matched,
                                device.cyclic_feedback_count() - before, stamp.eof.tick);
            return matched;
        }
        if (device.neutral_mit_only() && neutral_run_.active &&
            (!(neutral_run_.awaiting & bit) || !neutral_window_open(now_us))) return false;
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
        const auto before = device.cyclic_feedback_count();
#endif
        const bool verified = transactions_.consume_reply(c, index, device, frame, bytes, stamp, now, now_us);
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
        if (c == 2)
            DCU_FIRST_TRACE(DCU_FIRST_RX_CONSUMED, frame.id, verified,
                            device.cyclic_feedback_count() - before, stamp.eof.tick);
#endif
        if (device.neutral_mit_only() && neutral_run_.active) {
            if (!verified) return false;
            neutral_run_.awaiting &= ~bit;
            if (neutral_run_.submitted[index] == neutral_run_.limit[index]) {
                neutral_run_.active &= ~bit;
                neutral_run_.closed |= bit;
            }
            return true;
        }
        if (startup_.enforced() && verified && !device.neutral_mit_only()) {
            motor::MotorFeedback feedback{};
            motor::MotorDiagnostics diagnostics{};
            const auto status = device.read_feedback(now, state_.config.feedback_timeout_ms, feedback, diagnostics);
            // Device alarms block readiness; local motion faults remain latched for PDO ResetFault.
            if (feedback.sample.motor_fault) startup_.invalidate(DCU_GATEWAY_REASON_PROTOCOL_MISMATCH, now);
            else if (status.online && (feedback.sample.valid_fields & motor::Position))
                startup_.observe(index, stamp.observed_ms, now);
        }
        return true;
    }
    return false;
}

DCU_FAST_CODE void GatewayApplication::collect_tx_events(unsigned c)
{
    dcu_can_tx_event_t event;
    while (dcu_can_take_tx_event(c, &event)) {
        if (transactions_.finish_tx(c, event)) continue;
        for (unsigned j = 0; j < bus_device_count_[c]; ++j) {
            const unsigned a = devices_on_bus_[c][j];
            if (!devices_[a].finish_tx(event)) continue;
            if ((cyclic_.submitted & (1U << a)) && event.status == DCU_CAN_TX_COMPLETED) {
                cyclic_.completed |= 1U << a;
                cyclic_.tx_eof[a] = event.eof;
                if (event.eof.valid) DCU_BATCH_SAMPLE(a, tx_eof_us = event.eof.us);
            }
            break;
        }
    }
}

inline __attribute__((always_inline)) void GatewayApplication::route_raw_rx(
    unsigned c, const dcu_can_frame_t &frame, bool routed, bool single_fd)
{
    const bool raw_enabled = raw_channel(c);
    if (frame.dlc > DCU_CAN_CLASSIC_MAX_PAYLOAD && !single_fd) {
        if (!routed || raw_enabled) ++state_.legacy_rx_drop[c];
    } else if (raw_enabled) {
        if (layout_.state == RawState::Active) raw_.receive(c, frame);
        else if (layout_.state == RawState::Draining || layout_.state == RawState::Requested)
            ++layout_.drops;
    }
}

DCU_FAST_CODE void GatewayApplication::service_cyclic_feedback(uint32_t now, uint32_t now_us)
{
    if (!command_permitted(now) || dcu_tx_inhibited()) return;
    const bool tx_open = static_cast<int32_t>(cyclic_.deadline_us - now_us) > 0;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        if (!(cyclic_.buses & (1U << c))) {
            if (tx_open && neutral_run_.active && neutral_run_.bus == c) process_bus(c, now, now_us);
            continue;
        }
        if (dcu_can_bus_state(c) == DCU_CAN_BUS_OFF) continue;
        collect_tx_events(c);
        auto &scheduler = schedule_[c];
        scheduler.refill(now);
        const unsigned queued = cyclic_.queued[c];
        const unsigned cost = 2U * frame_us(c, 8);
        if (tx_open && queued && scheduler.can_afford(__builtin_popcount(queued) * cost) &&
            dcu_can_tx_serial_ready(c)) {
            dcu_can_batch_item_t items[DCU_MOTOR_DEVICES_PER_BUS];
            unsigned count = 0;
            bool busy = false;
            for (unsigned mask = queued; mask; mask &= mask - 1U) {
                const unsigned slot = __builtin_ctz(mask);
                const unsigned a = devices_on_bus_[c][slot];
                if (!devices_[a].active()) { cyclic_.queued[c] &= ~(1U << slot); continue; }
                if (transactions_.busy(c, a)) { busy = true; break; }
                items[count++] = {slot, &cyclic_.frames[a]};
            }
            if (count && !busy) {
                uint32_t queued_us{};
                capture_batch_time(queued_us);
                const auto first = dcu_can_submit_batch(c, items, count, cyclic_.deadline_us);
                cyclic_.queued[c] = 0;
                for (unsigned i = 0; i < count; ++i) {
                    const auto slot = items[i].owner;
                    const unsigned a = devices_on_bus_[c][slot];
                    auto &device = devices_[a];
                    if (first) {
                        DCU_BATCH_SAMPLE(a, queued_us = queued_us);
                        scheduler.control_submitted(slot, now_us, cyclic_.period_us, !device.control_sent(), cost);
                        device.control_submitted(now, first + i);
                        cyclic_.submitted |= 1U << a;
                    } else if (static_cast<int32_t>(cyclic_.deadline_us - dcu_time_now_us()) > 0) {
                        device.record_tx_drop();
                        ++state_.channel_tx_drop[c];
                        ++state_.tx_total;
                        device.stop(6);
                    }
                }
            }
        }
        dcu_can_frame_t frame;
        dcu_can_rx_stamp_t stamp;
        for (unsigned n = 0; n < DCU_MOTOR_DEVICES_PER_BUS && dcu_can_receive(c, &frame, &stamp); ++n) {
            // A reply can arrive between the initial retirement check and this dequeue.
            collect_tx_events(c);
            const bool routed = consume_feedback(c, frame, stamp, now, dcu_time_now_us());
            route_raw_rx(c, frame, routed, false);
        }
    }
}
ProcessResult GatewayApplication::process(const CycleCommand *candidate)
{
    ProcessResult result{};
    if (layout_.topology) {
        advance_layout(dcu_time_now_ms());
        result.consumed = candidate != nullptr;
        return result;
    }
    uint32_t now_us = dcu_time_now_us();
    if (!candidate && cyclic_feedback_missing()) {
        service_cyclic_feedback(dcu_time_now_ms(), now_us);
        return result;
    }
    if ((uint32_t)(now_us - state_.last_process_us) < policy::process_period_us)
        return result;
    state_.last_process_us = now_us;
    uint32_t now = dcu_time_now_ms();
    if (startup_.enforced()) step_startup(now);
    result.consumed = candidate != nullptr;
    if (candidate && (layout_.state == RawState::Draining || layout_.state == RawState::Requested))
        reject_raw(*candidate);
    result.accepted = candidate && accept_candidate(*candidate, now, result);
    check_timeouts(now);
    if (cyclic_feedback_missing()) {
        const auto reserved_buses = cyclic_.buses;
        service_cyclic_feedback(now, dcu_time_now_us());
        if (result.accepted && layout_.state == RawState::Active)
            result.raw_mirrored = process_raw(*candidate, now, reserved_buses);
        return result;
    }
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; c++) {
        if (!state_.channels[c].enabled) continue;
        process_bus(c, now, now_us);
    }
    if (startup_.enforced() && !startup_.running() && state_.owner != OWNER_NONE) {
        layout_fault();
        revoke_business();
        result.accepted = false;
        result.mirrored_devices = 0;
    }
    advance_layout(now);
    if (result.accepted && startup_.running() && layout_.state == RawState::Active)
        result.raw_mirrored = process_raw(*candidate, now);
    return result;
}

void GatewayApplication::process_bus(unsigned c, uint32_t now, uint32_t now_us)
{
    const bool pure = raw_only(state_.mode);
    const bool handed_off = layout_.switching && (layout_.pending_motor_owner & (1U << c));
    const unsigned motors = pure || handed_off ? 0 : bus_device_count_[c];
    if (neutral_run_.active && c == neutral_run_.bus && (dcu_can_bus_state(c) != DCU_CAN_BUS_ERROR_ACTIVE || dcu_can_rx_drops(c)))
        stop_neutral_run(true);
    collect_tx_events(c);
    for (unsigned j = 0; j < motors; ++j)
        transactions_.expire_poll(c, devices_on_bus_[c][j], now, now_us);
    schedule_[c].refill(now);
    dcu_can_frame_t f;
    dcu_can_rx_stamp_t stamp;
    unsigned n = 0;
    for (; n < 32 && dcu_can_receive(c, &f, &stamp); n++) {
        if (transactions_.awaiting_tx(c))
            collect_tx_events(c);
        const bool routed = !pure && consume_feedback(c, f, stamp, now, dcu_time_now_us());
        if (!routed && neutral_run_.active && c == neutral_run_.bus) stop_neutral_run(true);
        route_raw_rx(c, f, routed, state_.mode == OperatingMode::SingleFd);
    }
    if (!pure) service_neutral_run(c, now, n < 32);
    if (dcu_can_bus_state(c) == 3) {
        state_.board_fault |= 8;
        transactions_.cancel(c, RESULT_IO);
        for (unsigned j = 0; j < motors; ++j)
            devices_[devices_on_bus_[c][j]].stop(5);
        return;
    }
    if (!pure) {
        transactions_.expire_legacy(c, now);
        for (unsigned j = 0; j < motors; ++j)
            devices_[devices_on_bus_[c][j]].check_startup(now);
    }
    if (schedule_[c].stalled(now, dcu_can_tx_progress(c),
                             dcu_can_pending(c, DCU_CAN_DEVICE_ANY), state_.config.feedback_timeout_ms)) {
        /* 只把“队列持续无任何发送完成”视为真正的 TX 卡死。
         * 正常多轴流水发送时 TXBRP 可以跨调度周期保持非零。 */
        dcu_can_cancel(c);
        ++state_.channel_tx_drop[c];
        ++state_.tx_total;
        state_.board_fault |= 32;
        for (unsigned j = 0; j < motors; ++j)
            if (auto &device = devices_[devices_on_bus_[c][j]]; device.active()) {
                device.record_tx_drop();
                device.stop(6);
            }
        return;
    }
    // 纯透传仍消费终态事件并检查总线，但不创建电机控制、探测或参数事务。
    if (pure || handed_off) return;
    auto controls = controls_on_bus(c);
    if (startup_.running() && state_.operational && state_.session_armed) {
        unsigned cost = 2u * frame_us(c, 8);
        const int selected = schedule_[c].next_control(controls, now_us, cost, dcu_can_tx_serial_ready(c));
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
        if (c == 2)
            DCU_FIRST_TRACE(DCU_FIRST_SCHEDULE, controls.active_mask | (uint32_t{controls.first_send_mask} << 8) |
                            (schedule_[c].can_afford(cost) ? 1U << 16 : 0U),
                            static_cast<uint32_t>(selected), dcu_first_pdo_can_gate(), now_us);
#endif
        if (selected >= 0) {
            unsigned slot = static_cast<unsigned>(selected);
            auto &device = devices_[devices_on_bus_[c][slot]];
            if (device.encode_control(f)) {
                f.flags = state_.channels[c].frame_format ? 24 : 0;
                if (const auto id = dcu_can_submit(c, slot, &f)) {
                    schedule_[c].control_submitted(slot, now_us, controls.period_us, !device.control_sent(), cost);
                    device.control_submitted(now, id);
                    controls.first_send_mask &= ~(1u << slot);
                } else {
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
                    if (c == 2) DCU_FIRST_TRACE(DCU_FIRST_SUBMIT_FAILED, device.can_id(), dcu_tx_inhibited(), 0, 0);
#endif
                    device.record_tx_drop();
                    ++state_.channel_tx_drop[c];
                    ++state_.tx_total;
                }
            } else {
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
                if (c == 2) DCU_FIRST_TRACE(DCU_FIRST_ENCODE_FAILED, device.can_id(), 0, 0, 0);
#endif
                device.stop(2);
                controls.active_mask &= ~(1u << slot);
            }
        }
    }
#if defined(CONFIG_CAN2_FIRST_PDO_TRACE)
    else if (c == 2)
        DCU_FIRST_TRACE(DCU_FIRST_NOT_RUNNING, startup_.running(), state_.operational, state_.session_armed,
                        dcu_tx_inhibited());
#endif
    const bool fd = state_.channels[c].frame_format != 0;
    if (!(cyclic_.buses & (1U << c)) && (startup_.running() || startup_.may_probe()))
        transactions_.submit_queued(c, schedule_[c], controls, fd, now, now_us);
    const bool preparing = startup_.may_probe();
    const Owner owner = state_.owner;
    if (!(cyclic_.buses & (1U << c)) && (preparing || (startup_.running() &&
        (owner == OWNER_PDO || owner == OWNER_SDO))) &&
        state_.channels[c].role != 0) {
        /* Internal queries are strictly best effort.  Reserve one complete
         * control sweep and enough wall-clock slack before creating the
         * service, otherwise the query waits without stealing a deadline.
         * Non-active devices get the first query opportunity. Each driver
         * decides whether and what to poll. */
        unsigned query_cost = frame_us(c, 2) + frame_us(c, 8);
        if (schedule_[c].query_window(controls, now_us, query_cost, 2u * frame_us(c, 8))) {
            for (unsigned pass = 0; pass < 2; pass++) {
                for (unsigned n = 0; n < bus_device_count_[c]; n++) {
                    unsigned slot = (state_.query_cursor[c] + n) % bus_device_count_[c];
                    const unsigned index = devices_on_bus_[c][slot];
                    auto &device = devices_[index];
                    if (pass == 0 ? device.active() : !device.active())
                        continue;
                    if (transactions_.start_query(c, index, device, schedule_[c], controls,
                                                  preparing || owner == OWNER_PDO, fd, now, now_us)) {
                        state_.query_cursor[c] = (slot + 1) % bus_device_count_[c];
                        return; // One new query per worker pass; siblings need not wait for its reply.
                    }
                }
            }
        }
    }
}

bool GatewayApplication::process_raw(const CycleCommand &candidate, uint32_t now, uint8_t reserved_buses)
{
    const auto mode = state_.mode;
    if (mode == OperatingMode::Motor) return false;
    RawCanPolicy policy{};
    const bool single_fd = mode == OperatingMode::SingleFd;
    policy.permitted = single_fd ? state_.session_armed && state_.operational && startup_.running() :
                                   command_permitted(now);
    if (mode == OperatingMode::Raw && !policy.permitted) return false;
    for (unsigned c = 0; c < DCU_DEVICE_CAN_BUS_COUNT; ++c) {
        auto &bus = policy.buses[c];
        const auto &channel = state_.channels[c];
        bus.can_send = !(reserved_buses & (1U << c)) && channel.enabled &&
            (single_fd || (state_.board_command.enabled_channels & (1u << c))) &&
            raw_channel(c) && transactions_.raw_send_allowed(c, now);
        bus.rate_limit = raw_only(mode) ? layout_.limits[c] : channel.raw_rate_limit;
        bus.protect_full_bus = bus_device_count_[c] != 0;
        bus.fd_response = channel.frame_format != 0;
        if (!raw_only(mode) || bus.protect_full_bus)
            bus.protected_ids[bus.protected_count++] = legacy_broadcast_id;
        if (!single_fd || !(layout_.motor_owner & (1U << c)))
            for (unsigned j = 0; j < bus_device_count_[c]; ++j)
                bus.protected_ids[bus.protected_count++] = devices_[devices_on_bus_[c][j]].can_id();
    }
    const unsigned batches = mode == OperatingMode::Raw ? DCU_DEVICE_CAN_BUS_COUNT : 1;
    for (unsigned c = 0; c < batches; ++c) {
        const auto submit = single_fd ? raw_submit<true>(schedule_) :
            (raw_only(mode) ? raw_submit(schedule_) : raw_submit<false, true>(schedule_));
        const auto drops = raw_.process(candidate.raw[c], policy, now, submit);
        state_.tx_total += drops.total;
        layout_.drops += drops.total;
        for (unsigned bus = 0; bus < DCU_DEVICE_CAN_BUS_COUNT; ++bus)
            state_.channel_tx_drop[bus] += drops.channel[bus];
        if (drops.total && raw_drops() - layout_.baseline > config::raw_can::drop_limit) {
            layout_fault();
            return false;
        }
    }
    return true;
}
bool GatewayApplication::preop_access_allowed() const { return !state_.operational && !any_active(); }
bool GatewayApplication::owner_change_allowed() const
{
    return !any_active() && !transactions_.any_busy();
}
bool GatewayApplication::read_device(unsigned device, motor::MotorFeedback &feedback, motor::MotorDiagnostics &diagnostics) const
{
    if (device >= device_count_ || !devices_[device].configured()) return false;
    devices_[device].read_feedback(dcu_time_now_ms(), state_.config.feedback_timeout_ms, feedback, diagnostics);
    return true;
}
bool GatewayApplication::set_owner(Owner owner)
{
    if (owner != OWNER_NONE) return false;
    if (startup_.running()) {
        startup_.invalidate(DCU_GATEWAY_REASON_MASTER_TIMEOUT, dcu_time_now_ms());
        revoke_business();
    }
    return true;
}
CommandResult GatewayApplication::commit_sdo_command(unsigned device, const SdoCommand &command, uint32_t now)
{
    if (raw_only(state_.mode) || device >= device_count_ || !devices_[device].configured()) return {};
    CommandResult result{};
    if (command.sequence == 0) {
        result.changed = false;
        return result;
    }
    // SDO motion is never authorized; the permission check still ticks readiness before stopping.
    (void)command_permitted(now);
    result.result = RESULT_STATE;
    devices_[device].stop(8);
    return result;
}
ParameterReadResult GatewayApplication::read_parameter(unsigned device, const ParameterRead &request,
                                                        uint32_t now, bool is_preop)
{
    if (raw_only(state_.mode) || device >= device_count_ || !devices_[device].configured()) return {};
    const auto &instance = devices_[device];
    const unsigned bus = instance.bus();
    bool permitted = !startup_.enforced() && is_preop && preop_access_allowed() && state_.owner == OWNER_SDO &&
                     state_.channels[bus].enabled;
    return transactions_.read_parameter(device, instance, request, state_.channels[bus].frame_format,
                                        now, permitted, channel_active(bus));
}
}
namespace {
// Static storage is intentional; the composed objects must not use the task stack.
dcu::app::gateway_application::detail::GatewayApplication application;
}

namespace dcu::app::gateway_application {

bool initialize_profile(const dcu_production_profile_t &profile)
{
    if (!profile.channels || !profile.channel_count || profile.channel_count > DCU_DEVICE_CAN_BUS_COUNT ||
        profile.device_count > DCU_MOTOR_DEVICE_CAPACITY || (profile.device_count && !profile.devices) ||
        profile.passthrough_device_count > DCU_DEVICE_CAN_BUS_COUNT * 8U ||
        (profile.passthrough_device_count && !profile.passthrough_devices)) return false;
    detail::Configuration config{};
    config.board = {profile.max_command_lease_ms, profile.feedback_timeout_ms};
    config.devices = profile.devices;
    config.device_count = profile.device_count;
    for (size_t i = 0; i < profile.channel_count; ++i) {
        const auto &channel = profile.channels[i];
        if (channel.bus >= DCU_DEVICE_CAN_BUS_COUNT || config.channels[channel.bus].enabled) return false;
        const auto *active = dcu_can_channel_config(channel.bus);
        if (!active || active->fd != channel.fd || active->nominal_bitrate_bps != channel.nominal_bitrate_bps ||
            active->data_bitrate_bps != channel.data_bitrate_bps || active->timestamp_eof != channel.timestamp_eof)
            return false;
        config.channels[channel.bus].enabled = true;
        config.channels[channel.bus].frame_format = channel.fd;
        if (const auto limit = profile.raw_rate_limit[channel.bus]) {
            if (profile.device_count || limit > 8) return false;
            config.channels[channel.bus].role = 2;
            config.channels[channel.bus].raw_rate_limit = limit;
        }
    }
    for (size_t i = 0; i < profile.device_count; ++i) {
        const unsigned bus = profile.devices[i].bus;
        if (bus >= DCU_DEVICE_CAN_BUS_COUNT || !config.channels[bus].enabled)
            return false;
        config.channels[bus].role = 1;
    }
    uint8_t registered_ids[DCU_DEVICE_CAN_BUS_COUNT]{};
    for (size_t i = 0; i < profile.passthrough_device_count; ++i) {
        const auto &device = profile.passthrough_devices[i];
        if (device.bus >= DCU_DEVICE_CAN_BUS_COUNT || !device.can_id || device.can_id > 8 ||
            device.protocol != DCU_MOTOR_PROTOCOL_ENCOS ||
            !motor::known_motor_model(device.protocol, device.type)) return false;
        const auto &channel = config.channels[device.bus];
        if (!channel.enabled || channel.frame_format != 1 || channel.role) return false;
        const uint8_t bit = 1U << (device.can_id - 1);
        if (registered_ids[device.bus] & bit) return false;
        registered_ids[device.bus] |= bit;
    }
    return application.initialize(config);
}
ChannelConfiguration channel_configuration(unsigned bus) { return application.channel_configuration(bus); }
bool boot_startup(const dcu_gateway_profile_t &profile, uint64_t boot_id)
{ return application.boot_startup(profile, boot_id); }
bool startup_enabled() { return application.startup_enabled(); }
profile_update::Policy profile_policy() { return application.profile_policy(); }
bool apply_profile_policy(uint32_t profile, uint32_t base, uint32_t revision, const profile_update::Policy &policy)
{ return application.apply_profile_policy(profile, base, revision, policy); }
dcu_gateway_readiness_t readiness() { return application.readiness(); }
ProcessResult process(const CycleCommand *candidate) { return application.process(candidate); }
void operational(bool operational) { application.operational(operational); }
void cycle_fault() { application.cycle_fault(); }
bool preop_access_allowed() { return application.preop_access_allowed(); }
bool owner_change_allowed() { return application.owner_change_allowed(); }
bool select_mode(OperatingMode mode) { return application.select_mode(mode); }
RawState raw_state() { return application.raw_state(); }
uint32_t raw_drops() { return application.raw_drops(); }
uint32_t pure_raw_limits() { return application.pure_raw_limits(); }
bool set_pure_raw_limits(uint32_t packed) { return application.set_pure_raw_limits(packed); }
bool stage_raw_motor_owner(uint8_t buses) { return application.stage_raw_motor_owner(buses); }
uint8_t raw_motor_owner() { return application.raw_motor_owner(); }
bool request_layout(OperatingMode target, uint32_t profile, uint32_t revision,
                    const uint8_t (&digest)[DCU_GATEWAY_DIGEST_BYTES])
{ return application.request_layout(target, profile, revision, digest); }
bool map_layout(OperatingMode mode) { return application.map_layout(mode); }
bool reset_layout() { return application.reset_layout(); }
bool request_topology() { return application.request_topology(); }
bool topology_pending() { return application.topology_pending(); }
bool commit_topology(const dcu_production_profile_t &catalogue, uint32_t buses, uint32_t devices)
{ return application.commit_topology(catalogue, buses, devices); }
void layout_fault() { application.layout_fault(); }
OperatingMode mode() { return application.mode(); }
bool set_owner(Owner owner) { return application.set_owner(owner); }
Owner owner() { return application.owner(); }
CommandResult commit_sdo_command(unsigned device, const SdoCommand &command, uint32_t now_ms)
{
    return application.commit_sdo_command(device, command, now_ms);
}
bool parameter_service_busy(unsigned bus) { return application.parameter_service_busy(bus); }
ParameterReadResult read_parameter(unsigned device, const ParameterRead &request, uint32_t now_ms, bool preop)
{
    return application.read_parameter(device, request, now_ms, preop);
}
bool read_device(unsigned device, motor::MotorFeedback &feedback, motor::MotorDiagnostics &diagnostics)
{
    return application.read_device(device, feedback, diagnostics);
}
void prepare_publication(Publication &snapshot) { application.prepare_publication(snapshot); }
bool neutral_feedback_pending() { return application.neutral_feedback_pending(); }
void set_cyclic_feedback_window(uint32_t period, uint32_t deadline) { application.set_cyclic_feedback_window(period, deadline); }
void reset_preview(uint32_t seq) { application.reset_preview(seq); }
void preview_axis(unsigned a, const motor::Command &command) { application.preview_axis(a, command); }
uint32_t cyclic_feedback_required() { return application.cyclic_feedback_required(); }
uint32_t cyclic_feedback_missing() { return application.cyclic_feedback_missing(); }
void fault_cyclic_feedback(uint32_t missing) { application.fault_cyclic_feedback(missing); }
void set_neutral_window(uint32_t period_ns, uint32_t cutoff_us) { application.set_neutral_window(period_ns, cutoff_us); }
void neutral_feedback_deadline() { application.neutral_feedback_deadline(); }
bool take_raw_frame(unsigned bus, dcu_can_frame_t &frame) { return application.take_raw_frame(bus, frame); }
}
