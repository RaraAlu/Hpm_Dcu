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
constexpr uint64_t collect_phase_ns = 975 * ns_per_us;
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

struct Sample {
    uint64_t due_ns{}, deadline_ns{}, dc_tick_ns{}, write_ns{}, read_ns{}, feedback_ns{};
    uint32_t sequence{}, ack{}, output_wkc{}, input_wkc{}, al{};
    bool operational{};
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    uint32_t dc_sof_ns{};
    uint64_t sm2_frames{}, sm2_frame_bytes{}, sm3_wait_return_ns{};
    uint64_t sm3_ready_ns{}, sm3_send_return_ns{};
    bool dc_sof_valid{}, servo_used{};
#endif
#ifdef DCU_ONE_CYCLE_ENDURANCE
    bool slave_op{};
#endif
};

constexpr bool on_time(const Sample &s)
{
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
Sample exchange(Port &port, uint64_t due_ns, uint32_t sequence, uint64_t next_due_ns, uint64_t dc_tick_ns)
{
    Sample sample{};
    sample.due_ns = due_ns;
    sample.deadline_ns = next_due_ns < due_ns + period_ns ? next_due_ns : due_ns + period_ns;
    sample.dc_tick_ns = dc_tick_ns;
    sample.sequence = sequence;
    port.wait(due_ns);
    port.write(sample);
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
    port.wait(due_ns + collect_phase_ns);
    port.collect_feedback(sample);
    return sample;
}

template<class Port>
Sample exchange(Port &port, uint64_t due_ns, uint32_t sequence)
{
    return exchange(port, due_ns, sequence, due_ns + period_ns, due_ns);
}
}
