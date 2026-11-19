#include "one_cycle_pdo.hpp"
#include "probe_dc_clock.hpp"
#include <cassert>

namespace {
using namespace one_cycle;
struct BusModel {
    uint64_t now{}, staged_ns{}, response_ns{};
    uint32_t last_ack{}, response{};
    unsigned phase{}, sends{};
    uint64_t wake_delay{}, preparation_cost{};

    void wait(uint64_t ns) {
        if (now < ns) now = ns;
        if (phase == 1) now += wake_delay;
    }
    uint64_t now_ns() const { return now; }
    void spin_until(uint64_t target) {
        assert(phase == 2);
        if (now < target) now = target;
    }

    void write(Sample &s)
    {
        assert(phase++ == 0 && now == s.due_ns);
        s.write_ns = now + 5 * ns_per_us;
        staged_ns = s.write_ns + serialization_ns(784) + 20 * ns_per_us;
    }

    void prepare_feedback(Sample &s)
    {
        assert(phase++ == 1 && now == s.due_ns + read_phase_ns - sm3_prepare_ns + wake_delay);
        s.output_wkc = 1;
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

    void collect_feedback(Sample &s)
    {
        assert(phase++ == 3 && now >= s.due_ns + collect_phase_ns);
        s.input_wkc = now >= response_ns;
        s.operational = true;
        s.al = 8;
        s.ack = s.input_wkc ? response : last_ack;
        s.feedback_ns = now + 6 * ns_per_us;
        last_ack = s.ack;
        phase = 0;
    }
};
}

int main()
{
    using namespace one_cycle;
    static_assert(nominal_window_fits(784, 1072));
    static_assert(!nominal_window_fits(784, 1450));
    static_assert(next_tick(30, 30) == 30 + period_ns);
    static_assert(next_tick(30, 30 + 2 * period_ns + 1) == 30 + 3 * period_ns);
    BusModel bus;
    for (uint32_t seq = 1; seq <= 4; ++seq)
        assert(on_time(exchange(bus, seq * period_ns, seq)));

    const auto sample = exchange(bus, 5 * period_ns, 5);
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
