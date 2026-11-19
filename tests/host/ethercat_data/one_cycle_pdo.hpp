#pragma once

#include "cycle_config.h"
#include <cstdint>

#ifndef DCU_ONE_CYCLE_SYNC0_PHASE_US
#define DCU_ONE_CYCLE_SYNC0_PHASE_US 120U
#endif
#ifndef DCU_ONE_CYCLE_SM3_READ_PHASE_US
#define DCU_ONE_CYCLE_SM3_READ_PHASE_US 885U
#endif
#ifndef DCU_ONE_CYCLE_SM3_PREPARE_US
#define DCU_ONE_CYCLE_SM3_PREPARE_US 0U
#endif

namespace one_cycle {
constexpr uint64_t ns_per_us = 1000;
constexpr uint64_t period_ns = DCU_SYNC_MIN_PERIOD_NS;
constexpr uint64_t sync0_phase_ns = DCU_ONE_CYCLE_SYNC0_PHASE_US * ns_per_us;
constexpr uint64_t read_phase_ns = DCU_ONE_CYCLE_SM3_READ_PHASE_US * ns_per_us;
constexpr uint64_t sm3_prepare_ns = DCU_ONE_CYCLE_SM3_PREPARE_US * ns_per_us;
static_assert(sm3_prepare_ns <= 50 * ns_per_us && sm3_prepare_ns < read_phase_ns);
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
constexpr uint64_t sm2_prepare_ns = DCU_ONE_CYCLE_SM2_PREPARE_US * ns_per_us;
static_assert(sm2_prepare_ns <= 50 * ns_per_us && sm3_prepare_ns == 50 * ns_per_us);
#ifndef DCU_ONE_CYCLE_SOF_TRACE
#error "SM2 timing requires the same-frame DC trace"
#endif
#endif
constexpr uint64_t collect_phase_ns = 975 * ns_per_us;
enum class CollectWait { legacy, absolute, early };
constexpr uint64_t collect_advance_ns = 50 * ns_per_us;
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
static_assert(read_phase_ns < collect_phase_ns - collect_advance_ns);
#endif
constexpr uint64_t publish_deadline_ns = sync0_phase_ns + period_ns - DCU_TX_PUBLISH_ADVANCE_US * ns_per_us;

// One logical datagram on 100 Mb/s Ethernet, including preamble and IFG.
// This is a serialization estimate, not a bound on IgH, PHY or ESC latency.
constexpr uint64_t serialization_ns(unsigned payload_bytes)
{
    return (payload_bytes + 48U) * 80ULL;
}

constexpr bool nominal_window_fits(unsigned output_bytes, unsigned input_bytes)
{
    return serialization_ns(output_bytes) < sync0_phase_ns &&
           publish_deadline_ns < read_phase_ns &&
           read_phase_ns + serialization_ns(input_bytes) < collect_phase_ns &&
           collect_phase_ns < period_ns;
}

constexpr uint64_t next_tick(uint64_t origin_ns, uint64_t now_ns)
{
    return origin_ns + ((now_ns - origin_ns) / period_ns + 1) * period_ns;
}

constexpr uint64_t dc_time_at(uint64_t dc_tick, uint64_t due, uint64_t now)
{
    return now < due ? dc_tick - (due - now) : dc_tick + (now - due);
}

#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
struct CollectTiming {
    uint64_t wait_entry_ns{}, wake_ns{}, first_receive_begin_ns{}, first_receive_end_ns{}, first_domain_end_ns{};
    uint64_t first_state_end_ns{}, last_state_end_ns{}, wkc_valid_ns{}, ack_checked_ns{}, match_ns{}, op_checked_ns{};
    uint64_t receive_total_ns{}, domain_total_ns{}, state_total_ns{};
    uint32_t polls{};
};
#endif

struct Sample {
    uint64_t due_ns{}, deadline_ns{}, dc_tick_ns{}, write_ns{}, read_ns{}, feedback_ns{};
    uint32_t sequence{}, ack{}, output_wkc{}, input_wkc{}, al{};
    bool operational{};
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    CollectTiming collect{};
#endif
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    uint32_t dc_sof_ns{};
    uint64_t sm2_frames{}, sm2_frame_bytes{}, sm3_wait_return_ns{};
    uint64_t sm3_ready_ns{}, sm3_send_return_ns{};
    bool dc_sof_valid{}, servo_used{};
#endif
#ifdef DCU_ONE_CYCLE_ENDURANCE
    bool slave_op{};
#endif
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    uint64_t sm2_entry_ns{}, sm2_wake_ns{}, sm2_prepared_ns{}, sm2_queued_ns{}, sm2_send_return_ns{};
#endif
};

constexpr bool on_time(const Sample &s)
{
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    if (!s.collect.polls || s.collect.first_receive_begin_ns < s.due_ns + collect_phase_ns ||
        s.feedback_ns < s.collect.first_receive_begin_ns) return false;
#endif
    return s.operational && s.al == 8 && s.output_wkc == 1 && s.input_wkc == 1 &&
           s.sequence != 0 && s.ack == s.sequence &&
           s.due_ns <= s.write_ns && s.write_ns < s.due_ns + sync0_phase_ns &&
           s.read_ns >= s.due_ns + read_phase_ns && s.feedback_ns >= s.read_ns &&
           s.feedback_ns <= s.deadline_ns && s.deadline_ns <= s.due_ns + period_ns;
}

#ifdef DCU_ONE_CYCLE_ENDURANCE
static_assert(sm3_prepare_ns == 50 * ns_per_us);
// Observation may continue after a missed return; this never makes that sample pass.
constexpr bool may_observe_next(const Sample &s)
{
    return s.slave_op && s.al == 8 && s.output_wkc == 1 && s.servo_used &&
           s.sm3_send_return_ns && s.due_ns <= s.write_ns && s.write_ns < s.due_ns + sync0_phase_ns &&
           s.feedback_ns < s.deadline_ns + sync0_phase_ns;
}
#endif

template<class Port>
void collect_input(Port &port, Sample &s)
{
    bool complete;
    // Only poll the read already on the wire. No requeue, extra frame or deadline shift.
    do {
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
        auto &t = s.collect;
        const auto begin = port.now_ns();
#endif
        port.receive_input();
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
        const auto rx = port.now_ns();
#endif
        port.process_input();
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
        const auto domain = port.now_ns();
#endif
        complete = port.input_wkc(s);
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
        const auto state = port.now_ns();
        if (++t.polls == 1) {
            t.first_receive_begin_ns = begin;
            t.first_receive_end_ns = rx;
            t.first_domain_end_ns = domain;
            t.first_state_end_ns = state;
        }
        t.last_state_end_ns = state;
        t.receive_total_ns += rx - begin;
        t.domain_total_ns += domain - rx;
        t.state_total_ns += state - domain;
        if (complete && s.input_wkc == 1) t.wkc_valid_ns = state;
#endif
    } while (!complete && port.now_ns() < s.deadline_ns);
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    // Peek the stable process image before the AL ioctl, without advancing the acceptance timestamp.
    const auto ack = port.input_ack();
    const bool match = s.collect.wkc_valid_ns && s.output_wkc == 1 && s.sequence && ack == s.sequence;
    s.collect.ack_checked_ns = port.now_ns();
    if (match) s.collect.match_ns = s.collect.ack_checked_ns;
#endif
    s.operational = s.operational && complete;
    port.check_op(s);
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    s.collect.op_checked_ns = port.now_ns();
#endif
    s.ack = port.input_ack();
    s.feedback_ns = port.now_ns();
}

template<class Port>
Sample exchange(Port &port, uint64_t due_ns, uint32_t sequence, uint64_t next_due_ns, uint64_t dc_tick_ns)
{
    Sample sample{};
    sample.due_ns = due_ns;
    sample.deadline_ns = next_due_ns < due_ns + period_ns ? next_due_ns : due_ns + period_ns;
    sample.dc_tick_ns = dc_tick_ns;
    sample.sequence = sequence;
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    // The caller has finished the preceding SM3 exchange. Never borrow its receive window.
    const auto prepare_at = due_ns > sm2_prepare_ns ? due_ns - sm2_prepare_ns : 0;
    sample.sm2_entry_ns = port.now_ns();
    if (sample.sm2_entry_ns < prepare_at) port.wait(prepare_at);
    sample.sm2_wake_ns = port.now_ns();
#else
    port.wait(due_ns);
#endif
    port.prepare_output(sample);
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    sample.sm2_prepared_ns = port.now_ns();
#endif
    port.queue_output();
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    sample.sm2_queued_ns = port.now_ns();
    if constexpr (sm2_prepare_ns != 0) port.spin_until(due_ns);
#endif
    sample.write_ns = port.now_ns();
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    // The queued LWR must not be flushed by a later send after its latch window.
    if (sample.write_ns >= due_ns + sync0_phase_ns) return sample;
#endif
    port.send_output(sample);
    const auto send_at = due_ns + read_phase_ns;
    port.wait(send_at - sm3_prepare_ns);
    port.prepare_feedback(sample);
    if constexpr (sm3_prepare_ns != 0) port.spin_until(send_at);
    sample.read_ns = port.now_ns();
    if constexpr (sm3_prepare_ns != 0) {
        if (sample.read_ns >= sample.deadline_ns) {
            // Deactivation is next: never roll this queued read into the following cycle.
            sample.operational = false;
            sample.feedback_ns = sample.read_ns;
            return sample;
        }
    }
    port.request_feedback(sample);
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    const auto collect_at = due_ns + collect_phase_ns;
    const bool early = port.collect_mode == CollectWait::early;
    sample.collect.wait_entry_ns = port.now_ns();
    port.wait_collect(early ? collect_at - collect_advance_ns : collect_at);
    sample.collect.wake_ns = port.now_ns();
    // Only wait on the RK clock here; do not poll IgH or inspect the input image before the gate.
    if (early && sample.collect.wake_ns < collect_at) port.spin_until(collect_at);
#else
    port.wait(due_ns + collect_phase_ns);
#endif
    port.collect_feedback(sample);
    return sample;
}

template<class Port>
Sample exchange(Port &port, uint64_t due_ns, uint32_t sequence)
{
    return exchange(port, due_ns, sequence, due_ns + period_ns, due_ns);
}
}
