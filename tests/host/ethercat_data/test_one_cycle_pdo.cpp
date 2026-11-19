#include "one_cycle_pdo.hpp"
#include "probe_dc_clock.hpp"
#include <cassert>

namespace {
using namespace one_cycle;
struct BusModel {
    uint64_t now{}, staged_ns{}, response_ns{};
    uint32_t last_ack{}, response{};
    unsigned phase{}, sends{}, output_phase{}, output_sends{};
    uint64_t wake_delay{}, preparation_cost{};
    uint64_t output_wake_delay{}, output_prep{3000}, output_queue{2000}, app_stamp{};

    void wait(uint64_t ns) {
        if (now < ns) { now = ns; if (phase == 0) now += output_wake_delay; }
        if (phase == 1) now += wake_delay;
    }
    uint64_t now_ns() const { return now; }
    void spin_until(uint64_t target) {
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
        if (phase == 3) {
            assert(collect_mode == CollectWait::early && target == sleep_target + 50000 && now < target);
            now = target + gate_delay;
            return;
        }
#endif
        assert(phase == 2 || (phase == 0 && output_phase == 2));
        if (now < target) now = target;
    }

    void prepare_output(const Sample &s)
    {
        assert(phase == 0 && output_phase++ == 0);
        now += output_prep;
        app_stamp = dc_time_at(s.dc_tick_ns, s.due_ns, now);
    }
    void queue_output() { assert(output_phase++ == 1); now += output_queue; }
    void send_output(Sample &s)
    {
        assert(phase++ == 0 && output_phase == 2 && now == s.write_ns && now >= s.due_ns);
        output_phase = 0;
        ++output_sends;
        staged_ns = s.write_ns + serialization_ns(784) + 20 * ns_per_us;
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
        s.sm2_send_return_ns = now + 1000;
#endif
    }

    void prepare_feedback(Sample &s)
    {
        assert(phase++ == 1 && now == s.due_ns + read_phase_ns - sm3_prepare_ns + wake_delay);
        s.output_wkc = 1;
        s.operational = true;
        now += preparation_cost;
    }

    void request_feedback(Sample &s)
    {
        assert(phase++ == 2 && now >= s.due_ns + read_phase_ns && now == s.read_ns);
        ++sends;
        response = staged_ns < s.due_ns + sync0_phase_ns && now >= s.due_ns + publish_deadline_ns
                       ? s.sequence : last_ack;
        response_ns = now + serialization_ns(1072);
    }

#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    CollectWait collect_mode{CollectWait::legacy};
    uint64_t sleep_target{}, first_receive_call{}, gate_delay{};
    uint64_t wake_cost{}, rx_cost{2000}, domain_cost{2000}, state_cost{1000}, al_cost{1000};
    unsigned incomplete{};
    bool stale{}, op{true};
    void wait_collect(uint64_t target) {
        assert(phase == 3);
        sleep_target = target;
        if (now < target) now = target + wake_cost;
    }
    void receive_input() {
        if (!first_receive_call) first_receive_call = now;
        now += rx_cost;
    }
    void process_input() { now += domain_cost; }
    bool input_wkc(Sample &s) {
        now += state_cost;
        s.input_wkc = !incomplete && now >= response_ns;
        if (incomplete) --incomplete;
        return s.input_wkc == 1;
    }
    uint32_t input_ack() const { return stale ? last_ack : response; }
    void check_op(Sample &s) {
        now += al_cost;
        s.al = op ? 8 : 4;
        s.operational = s.operational && op;
    }
    void collect_feedback(Sample &s) {
        assert(phase++ == 3);
        collect_input(*this, s);
        last_ack = s.ack;
        phase = 0;
    }
#else
    void collect_feedback(Sample &s)
    {
        assert(phase++ == 3 && now >= s.due_ns + collect_phase_ns);
        s.input_wkc = now >= response_ns;
        s.operational = true;
        s.al = 8;
        s.ack = s.input_wkc ? response : last_ack;
        now += 6 * ns_per_us;
        s.feedback_ns = now;
        last_ack = s.ack;
        phase = 0;
    }
#endif
};
}

int main()
{
    using namespace one_cycle;
    static_assert(nominal_window_fits(784, 1072));
    static_assert(!nominal_window_fits(784, 1450));
    static_assert(next_tick(30, 30) == 30 + period_ns);
    static_assert(next_tick(30, 30 + 2 * period_ns + 1) == 30 + 3 * period_ns);
    static_assert(dc_time_at(7000000, 5000000, 4980000) == 6980000);
    BusModel bus;
    for (uint32_t seq = 1; seq <= 4; ++seq)
        assert(on_time(exchange(bus, seq * period_ns, seq)));

    const auto sample = exchange(bus, 5 * period_ns, 5);
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    const auto target = period_ns + collect_phase_ns;
    for (uint64_t delay : {0U, 23167U, 40000U, 50000U, 60000U, 80000U}) {
        BusModel early_bus;
        early_bus.collect_mode = CollectWait::early;
        early_bus.wake_cost = delay;
        const auto deadline = 2 * period_ns - 33;
        const auto s = exchange(early_bus, period_ns, 1, deadline, period_ns);
        const auto wake = target - 50000 + delay;
        const auto receive = wake < target ? target : wake;
        assert(early_bus.sleep_target == target - 50000 && s.collect.wake_ns == wake);
        assert(early_bus.first_receive_call == receive && s.collect.first_receive_begin_ns == receive);
        assert(s.collect.match_ns >= receive && s.feedback_ns == receive + 6000 && s.deadline_ns == deadline);
        assert(on_time(s) == (s.feedback_ns <= deadline) && s.ack == 1 && s.collect.polls == 1);
        assert(early_bus.output_sends == 1 && early_bus.sends == 1);
        auto too_early = s;
        too_early.collect.first_receive_begin_ns = target - 1;
        assert(!on_time(too_early));
    }
    for (int fault = 0; fault <= 3; ++fault) {
        BusModel timed;
        if (fault == 1) timed.wake_cost = 40000;
        if (fault == 2) timed.rx_cost += 40000;
        if (fault == 3) timed.domain_cost += 40000;
        const auto a = exchange(timed, period_ns, 1);
        const auto &t = a.collect;
        assert(t.wait_entry_ns < target && t.wake_ns == target + timed.wake_cost);
        assert(t.polls == 1 && t.receive_total_ns == timed.rx_cost && t.domain_total_ns == timed.domain_cost);
        assert(t.first_receive_end_ns == t.wake_ns + timed.rx_cost);
        assert(t.first_domain_end_ns == t.first_receive_end_ns + timed.domain_cost);
        assert(t.wkc_valid_ns == t.first_state_end_ns && t.wkc_valid_ns == t.first_domain_end_ns + timed.state_cost);
        assert(t.match_ns == t.ack_checked_ns && t.op_checked_ns == t.match_ns + timed.al_cost);
        assert(a.feedback_ns == t.op_checked_ns && a.ack == 1 && on_time(a) == (fault == 0));
        const auto b = exchange(timed, 2 * period_ns, 2);
        assert(b.write_ns - b.due_ns == (fault ? 26000U : 0U));
    }
    BusModel interrupted;
    interrupted.collect_mode = CollectWait::early;
    interrupted.gate_delay = 30000;
    const auto paused = exchange(interrupted, period_ns, 1);
    assert(paused.collect.wake_ns == target - 50000 && paused.collect.first_receive_begin_ns == target + 30000);
    assert(paused.ack == 1 && paused.feedback_ns > paused.deadline_ns && !on_time(paused));
    BusModel retry;
    retry.incomplete = 1;
    auto twice = exchange(retry, period_ns, 1);
    assert(on_time(twice) && twice.collect.polls == 2 && twice.collect.receive_total_ns == 4000 &&
           twice.collect.domain_total_ns == 4000 && twice.collect.state_total_ns == 2000 &&
           twice.collect.wkc_valid_ns > twice.collect.first_state_end_ns);
    retry.stale = true;
    auto stale = exchange(retry, 2 * period_ns, 2);
    assert(!on_time(stale) && stale.collect.wkc_valid_ns && !stale.collect.match_ns);
    retry.stale = false;
    retry.op = false;
    auto no_op = exchange(retry, 3 * period_ns, 3);
    assert(!on_time(no_op) && no_op.collect.match_ns && no_op.collect.op_checked_ns);
    retry.op = true;
    retry.al_cost += 40000;
    auto late_op = exchange(retry, 4 * period_ns, 4);
    assert(late_op.collect.match_ns < late_op.deadline_ns && late_op.feedback_ns > late_op.deadline_ns && !on_time(late_op));
    retry.incomplete = 100;
    auto no_wkc = exchange(retry, 5 * period_ns, 5);
    assert(!on_time(no_wkc) && no_wkc.collect.polls > 1 && !no_wkc.collect.wkc_valid_ns && !no_wkc.collect.match_ns);
#endif
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    {
        BusModel timed;
        timed.now = period_ns - 18254;  // Previous feedback has already been collected.
        timed.output_wake_delay = 57831;
        auto s = exchange(timed, period_ns, 21, 2 * period_ns - 30, 7 * period_ns);
        assert(s.sm2_entry_ns == period_ns-18254 && s.sm2_prepared_ns-s.sm2_wake_ns == 3000 &&
               s.sm2_queued_ns-s.sm2_prepared_ns == 2000 && s.deadline_ns == 2*period_ns-30);
        if constexpr (sm2_prepare_ns) {
            assert(s.sm2_wake_ns == s.sm2_entry_ns && s.sm2_queued_ns < s.due_ns && s.write_ns == s.due_ns);
            assert(timed.app_stamp == 7 * period_ns - 18254 + 3000);
        } else {
            assert(s.sm2_wake_ns == period_ns + 57831 && s.write_ns == period_ns + 62831);
        }
        assert(on_time(s) && timed.output_sends == 1 && timed.sends == 1);
        timed.now = 2 * period_ns + 17771;
        auto next = exchange(timed, 2 * period_ns, 22);
        assert(next.sm2_wake_ns == next.sm2_entry_ns && next.write_ns == 2*period_ns+22771);
        timed.now = 3 * period_ns + sync0_phase_ns - 5000;
        auto expired = exchange(timed, 3 * period_ns, 23);
        assert(!expired.sm2_send_return_ns && !expired.sm3_send_return_ns && !on_time(expired));
        assert(timed.output_sends == 2 && timed.sends == 2);
    }
#endif
    auto bad = sample;
    bad.ack = 4;
    assert(!on_time(bad));
    bad = sample;
    bad.ack = 6;
    assert(!on_time(bad));
    bad = sample;
    bad.feedback_ns = sample.due_ns + period_ns;
    assert(on_time(bad));
    ++bad.feedback_ns;
    assert(!on_time(bad));
    bad = sample;
    bad.feedback_ns = sample.due_ns + 3 * period_ns;
    assert(!on_time(bad));
    bad = sample;
    bad.output_wkc = 0;
    assert(!on_time(bad));
    bad = sample;
    bad.input_wkc = 0;
    assert(!on_time(bad));
    bad = sample;
    bad.al = 4;
    assert(!on_time(bad));
    bad = sample;
    bad.write_ns = sample.due_ns + sync0_phase_ns;
    assert(!on_time(bad));

    const auto early_next = 7 * period_ns - 350;
    auto early = exchange(bus, 6 * period_ns, 6, early_next, 77 * period_ns);
    assert(early.deadline_ns == early_next && early.dc_tick_ns == 77 * period_ns);
#ifdef DCU_ONE_CYCLE_ENDURANCE
    auto missed = early;
    missed.slave_op = missed.servo_used = true;
    missed.sm3_send_return_ns = early.read_ns + 1000;
    missed.input_wkc = 0;
    assert(!on_time(missed) && may_observe_next(missed));
    for (unsigned reason = 0; reason < 6; ++reason) {
        auto broken = missed;
        if (reason == 0) broken.slave_op = false;
        if (reason == 1) broken.servo_used = false;
        if (reason == 2) broken.sm3_send_return_ns = 0;
        if (reason == 3) broken.output_wkc = 0;
        if (reason == 4) broken.al = 4;
        if (reason == 5) broken.feedback_ns = broken.deadline_ns + sync0_phase_ns;
        assert(!may_observe_next(broken));
    }
#endif
    early.feedback_ns = early_next + 1;
    assert(!on_time(early));
    auto late = exchange(bus, 7 * period_ns, 7, 8 * period_ns + 450, 78 * period_ns);
    assert(late.deadline_ns == 8 * period_ns);

    if constexpr (sm3_prepare_ns == 50 * ns_per_us) {
        BusModel timed;
        timed.wake_delay = 30 * ns_per_us;
        timed.preparation_cost = 9 * ns_per_us;
        auto hidden = exchange(timed, period_ns, 1);
        assert(timed.sends == 1 && hidden.read_ns == period_ns + read_phase_ns && on_time(hidden));
        timed.wake_delay = 45 * ns_per_us;
        auto over = exchange(timed, 2 * period_ns, 2);
        assert(timed.sends == 2 && over.read_ns == 2 * period_ns + read_phase_ns + 4 * ns_per_us);
        assert(over.deadline_ns == 3 * period_ns && on_time(over));
        timed.wake_delay = period_ns - read_phase_ns + sm3_prepare_ns - timed.preparation_cost;
        auto expired = exchange(timed, 3 * period_ns, 3);
        assert(timed.sends == 2 && expired.read_ns == expired.deadline_ns && !on_time(expired));
    }

    for (int drift : {-85, 72}) {
        ReferenceFollower filter;
        uint64_t grid = (uint64_t{1} << 32) - 77 * period_ns;
        int64_t arrival = int64_t(grid + filter.target_sof_ns) + 130000 + int64_t(period_ns);
        for (uint32_t n = 1; n <= 6500; ++n) {
            const int step = filter.step_ns(); // N+1 is fixed before observing N.
            const bool acquiring = n <= 600;
            const auto before = grid;
            const int noise = (n % 7 == 0 ? 450 : -75) + (n == 800 ? 12000 : 0);
            const auto sof = uint32_t(arrival + noise);
            assert(filter.observe(grid, n, sof, acquiring));
            if (n == 1)
                assert(grid == before + period_ns);
            else
                assert(grid == before);
            if (n == 1000) {
                const int previous_step = filter.step_ns();
                assert(!filter.observe(grid, n + 1, sof, false));
                assert(filter.step_ns() == previous_step);
            }
            if (!acquiring) {
                assert(filter.locked() && std::abs(step) <= filter.tracking_limit_ns);
                assert(std::abs(arrival - int64_t(grid) - filter.target_sof_ns) < 1300);
            }
            arrival += int64_t(period_ns) + drift + step;
            grid += period_ns;
        }
    }
}
