#include "igh_cycle.hpp"
#include "one_cycle_pdo.hpp"
#ifdef DCU_ONE_CYCLE_SOF_TRACE
#include "igh_probe_frame_stats.hpp"
#include "probe_dc_clock.hpp"
#endif
#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <memory>
#include <sched.h>
#include <string_view>
#include <sys/mman.h>
#include <sys/resource.h>
#include <thread>
#include <type_traits>
#include <vector>

namespace ig = x2_deploy::driver_module::dcu;
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {
constexpr uint32_t vendor = 0x10000, product = 2, revision = 2, interface_version = 0x10002;
constexpr auto default_period = 4ms;
constexpr unsigned rx_lengths[] = {736, 784, 544}, tx_lengths[] = {1024, 1072, 544};
constexpr unsigned timing_drain_cycles = 24;
constexpr unsigned max_timing_cycles = 30 * 60 * 1000;
constexpr auto max_spin = 50us;
constexpr uint64_t slow_receive_ns = 8000;
using Image = std::array<uint8_t, 784>;

struct Ticket {
    uint32_t epoch, generation, sequence;
};

struct TimingSample {
    uint64_t due_ns, wake_ns, send_ns, send_return_ns, feedback_ns;
    uint32_t sent, ack, wkc, al;
    bool valid;
    uint64_t receive_start_ns, receive_end_ns, domain_end_ns;
    long voluntary_switches, involuntary_switches;
};

void require(bool check, const char *message)
{
    if (!check) throw std::runtime_error(message);
}

uint64_t monotonic_ns(Clock::time_point time)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}

Clock::time_point wait_for_tick(Clock::time_point tick, std::chrono::microseconds spin)
{
    if (!spin.count()) {
        std::this_thread::sleep_until(tick);
        return Clock::now();
    }
    const auto early_ns = monotonic_ns(tick - spin);
    const timespec early{static_cast<time_t>(early_ns / 1000000000),
                         static_cast<long>(early_ns % 1000000000)};
    int rc;
    do {
        rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &early, nullptr);
    } while (rc == EINTR);
    require(rc == 0, "absolute deadline wait failed");
    // Give the receive path CPU time at the deadline, not just a timer wakeup.
    auto now = Clock::now();
    while (now < tick) now = Clock::now();
    return now;
}

bool write_timing(const char *path, const std::vector<TimingSample> &samples)
{
    auto *file = std::fopen(path, "w");
    if (!file) return false;
    std::fputs("due_ns,wake_ns,send_ns,send_return_ns,feedback_ns,sent,ack,wkc,al,valid,"
               "receive_start_ns,receive_end_ns,domain_end_ns,voluntary_switches,involuntary_switches\n", file);
    for (const auto &s : samples) {
        if (!s.sent) break;
        std::fprintf(file, "%llu,%llu,%llu,%llu,%llu,%u,%u,%u,%u,%d,%llu,%llu,%llu,%ld,%ld\n",
                     static_cast<unsigned long long>(s.due_ns), static_cast<unsigned long long>(s.wake_ns),
                     static_cast<unsigned long long>(s.send_ns), static_cast<unsigned long long>(s.send_return_ns),
                     static_cast<unsigned long long>(s.feedback_ns), s.sent, s.ack, s.wkc, s.al, s.valid,
                     static_cast<unsigned long long>(s.receive_start_ns),
                     static_cast<unsigned long long>(s.receive_end_ns),
                     static_cast<unsigned long long>(s.domain_end_ns), s.voluntary_switches, s.involuntary_switches);
    }
    const bool ok = !std::ferror(file);
    return std::fclose(file) == 0 && ok;
}

bool write_one_cycle_timing(const char *path, const std::vector<one_cycle::Sample> &samples)
{
    auto *file = std::fopen(path, "w");
    if (!file) return false;
    std::fputs("due_ns,write_ns,read_ns,feedback_ns,sent,ack,output_wkc,input_wkc,al,on_time", file);
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    std::fputs(",sm2_frames,sm2_frame_bytes,dc_sof_ns,dc_sof_valid,dc_tick_ns,deadline_ns,servo_used,sm3_wait_return_ns", file);
    if constexpr (one_cycle::sm3_prepare_ns)
        std::fputs(",sm3_ready_ns,sm3_send_return_ns", file);
#endif
#ifdef DCU_ONE_CYCLE_ENDURANCE
    std::fputs(",slave_op", file);
#endif
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    std::fputs(",sm2_entry_ns,sm2_wake_ns,sm2_prepared_ns,sm2_queued_ns,sm2_send_return_ns", file);
#endif
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    std::fputs(",collect_wait_entry_ns,collect_wake_ns,first_receive_begin_ns,first_receive_end_ns,first_domain_end_ns,"
               "first_state_end_ns,last_state_end_ns,wkc_valid_ns,ack_checked_ns,match_ns,op_checked_ns,"
               "receive_total_ns,domain_total_ns,state_total_ns,collect_polls", file);
#endif
    std::fputc('\n', file);
    for (const auto &s : samples) {
        if (!s.sequence) break;
        std::fprintf(file, "%llu,%llu,%llu,%llu,%u,%u,%u,%u,%u,%d",
                     static_cast<unsigned long long>(s.due_ns), static_cast<unsigned long long>(s.write_ns),
                     static_cast<unsigned long long>(s.read_ns), static_cast<unsigned long long>(s.feedback_ns),
                     s.sequence, s.ack, s.output_wkc, s.input_wkc, s.al, one_cycle::on_time(s));
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        std::fprintf(file, ",%llu,%llu,%u,%d,%llu,%llu,%d,%llu", static_cast<unsigned long long>(s.sm2_frames),
                     static_cast<unsigned long long>(s.sm2_frame_bytes),
                     s.dc_sof_ns, s.dc_sof_valid, static_cast<unsigned long long>(s.dc_tick_ns),
                     static_cast<unsigned long long>(s.deadline_ns), s.servo_used,
                     static_cast<unsigned long long>(s.sm3_wait_return_ns));
        if constexpr (one_cycle::sm3_prepare_ns)
            std::fprintf(file, ",%llu,%llu", static_cast<unsigned long long>(s.sm3_ready_ns),
                         static_cast<unsigned long long>(s.sm3_send_return_ns));
#endif
#ifdef DCU_ONE_CYCLE_ENDURANCE
        std::fprintf(file, ",%d", s.slave_op);
#endif
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
        std::fprintf(file, ",%llu,%llu,%llu,%llu,%llu", static_cast<unsigned long long>(s.sm2_entry_ns),
                     static_cast<unsigned long long>(s.sm2_wake_ns), static_cast<unsigned long long>(s.sm2_prepared_ns),
                     static_cast<unsigned long long>(s.sm2_queued_ns), static_cast<unsigned long long>(s.sm2_send_return_ns));
#endif
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
        const auto &t = s.collect;
        for (auto n : {t.wait_entry_ns, t.wake_ns, t.first_receive_begin_ns, t.first_receive_end_ns, t.first_domain_end_ns,
                       t.first_state_end_ns, t.last_state_end_ns, t.wkc_valid_ns, t.ack_checked_ns, t.match_ns, t.op_checked_ns,
                       t.receive_total_ns, t.domain_total_ns, t.state_total_ns})
            std::fprintf(file, ",%llu", static_cast<unsigned long long>(n));
        std::fprintf(file, ",%u", t.polls);
#endif
        std::fputc('\n', file);
    }
    const bool ok = !std::ferror(file);
    return std::fclose(file) == 0 && ok;
}

template<typename Function, typename... Args>
void check_igh(const char *operation, Function function, Args... args)
{
    if constexpr (std::is_same_v<std::invoke_result_t<Function, Args...>, int>) {
        require(function(args...) == 0, operation);
    } else {
        errno = 0;
        function(args...);
        require(errno == 0, operation);
    }
}

// The historical diagnostics keep exchanging frames while requesting PREOP.
// This is an optional vendor SDK extension, not needed by the one-cycle probe.
template<class Master>
auto request_legacy_preop(Master *master) -> decltype(ecrt_master_deactivate_slaves(master), void())
{
    ecrt_master_deactivate_slaves(master);
}

[[maybe_unused]] void request_legacy_preop(...)
{
    throw std::runtime_error("legacy graceful exit requires the IgH SDK PREOP extension");
}

uint32_t number(std::string_view value)
{
    uint32_t result{};
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    require(parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size(), "invalid probe argument");
    return result;
}

uint32_t upload(ec_master_t *master, uint16_t index, uint8_t sub, size_t width)
{
    uint8_t value[4]{};
    size_t size{};
    uint32_t abort{};
    const int rc = ecrt_master_sdo_upload(master, 0, index, sub, value, sizeof(value), &size, &abort);
    if (rc || abort || size != width) {
        std::fprintf(stderr, "SDO %04x:%02x rc=%d abort=%08x size=%zu expected=%zu\n",
                     index, sub, rc, abort, size, width);
        throw std::runtime_error("unexpected SDO read result");
    }
    return EC_READ_U32(value);
}

Image ordinary(uint32_t seq, uint16_t lease, bool enabled = false)
{
    Image image{};
    EC_WRITE_U32(image.data(), seq);
    EC_WRITE_U16(image.data() + 4, enabled ? 1 : 0);
    EC_WRITE_U16(image.data() + 8, lease);
    EC_WRITE_U32(image.data() + 12, interface_version);
    return image;
}

struct IghOneCyclePort {
    ec_master_t *master;
    ec_slave_config_t *slave;
    ec_domain_t *output, *input;
    uint8_t *output_data, *input_data;
    unsigned output_bytes;
    uint16_t lease;
    bool follow_hpm;
    one_cycle::CollectWait collect_mode;
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    const one_cycle::FrameCounter &counter;
    one_cycle::FrameStats after_sm3{};
    bool recent_sof{};
#endif

    void wait(uint64_t ns)
    {
        std::this_thread::sleep_until(Clock::time_point(std::chrono::nanoseconds(ns)));
    }

    uint64_t now_ns() const { return ig::IghMonotonicTimeNs(); }

#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
    void wait_collect(uint64_t ns)
    {
        if (collect_mode != one_cycle::CollectWait::absolute) return wait(ns);
        if (now_ns() >= ns) return;
        const timespec target{static_cast<time_t>(ns / 1000000000), static_cast<long>(ns % 1000000000)};
        int rc;
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &target, nullptr);
        } while (rc == EINTR);
        require(rc == 0, "absolute SM3 collect wait failed");
    }
#endif

    void spin_until(uint64_t ns)
    {
        while (now_ns() < ns) {}
    }

    void prepare_output(const one_cycle::Sample &sample)
    {
        const auto image = ordinary(sample.sequence, lease);
        std::copy_n(image.data(), output_bytes, output_data);
        const auto app_time = ig::IghMonotonicTimeNs();
        check_igh("IgH application time", ecrt_master_application_time, master,
                  follow_hpm ? one_cycle::dc_time_at(sample.dc_tick_ns, sample.due_ns, app_time) : app_time);
    }

    void queue_output()
    {
        if (follow_hpm) {
            // Replace the four-byte FPWR with a four-byte read, not another frame.
            check_igh("IgH read-only DC monitor", ecrt_master_sync_monitor_queue, master);
        } else {
            check_igh("IgH reference sync", ecrt_master_sync_reference_clock_to, master, ig::IghMonotonicTimeNs());
        }
        check_igh("IgH slave sync", ecrt_master_sync_slave_clocks, master);
        check_igh("IgH queue output write", ecrt_domain_queue, output);
    }

    void send_output([[maybe_unused]] one_cycle::Sample &sample)
    {
        check_igh("IgH send output write", ecrt_master_send, master);
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
        sample.sm2_send_return_ns = ig::IghMonotonicTimeNs();
#endif
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        const auto now = counter.read();
        if (now.valid && after_sm3.valid && now.frames >= after_sm3.frames && now.bytes >= after_sm3.bytes) {
            sample.sm2_frames = now.frames - after_sm3.frames;
            sample.sm2_frame_bytes = now.bytes - after_sm3.bytes;
        }
#endif
    }

    void prepare_feedback(one_cycle::Sample &sample)
    {
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        sample.sm3_wait_return_ns = ig::IghMonotonicTimeNs();
#endif
        check_igh("IgH receive output return", ecrt_master_receive, master);
        check_igh("IgH process output domain", ecrt_domain_process, output);
        ec_domain_state_t state{};
        check_igh("IgH output domain state", ecrt_domain_state, output, &state);
        sample.output_wkc = state.working_counter;
        sample.operational = state.wc_state == EC_WC_COMPLETE;
        check_igh("IgH queue input read", ecrt_domain_queue, input);
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        if constexpr (one_cycle::sm3_prepare_ns) sample.sm3_ready_ns = ig::IghMonotonicTimeNs();
#endif
    }

    void request_feedback([[maybe_unused]] one_cycle::Sample &sample)
    {
        check_igh("IgH send input read", ecrt_master_send, master);
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        if constexpr (one_cycle::sm3_prepare_ns) sample.sm3_send_return_ns = ig::IghMonotonicTimeNs();
        after_sm3 = counter.read();
        // Read the returned SM2-send FRMW after both sends, before the next DC queue.
        if (sample.output_wkc == 1 || recent_sof || (sample.sequence % 32 == 0 && sample.sm2_frames == 1)) {
            recent_sof = ecrt_master_reference_clock_time(master, &sample.dc_sof_ns) == 0;
            sample.dc_sof_valid = recent_sof;
        }
#endif
    }

    void receive_input() { check_igh("IgH receive input return", ecrt_master_receive, master); }
    void process_input() { check_igh("IgH process input domain", ecrt_domain_process, input); }
    bool input_wkc(one_cycle::Sample &sample)
    {
        ec_domain_state_t state{};
        check_igh("IgH input domain state", ecrt_domain_state, input, &state);
        sample.input_wkc = state.working_counter;
        return state.wc_state == EC_WC_COMPLETE;
    }
    uint32_t input_ack() const { return EC_READ_U32(input_data + 4); }
    void check_op(one_cycle::Sample &sample)
    {
        ec_slave_config_state_t status{};
        check_igh("IgH AL state", ecrt_slave_config_state, slave, &status);
        sample.al = status.al_state;
#ifdef DCU_ONE_CYCLE_ENDURANCE
        sample.slave_op = status.online && status.operational;
#endif
        sample.operational = sample.operational && status.online && status.operational;
    }
    void collect_feedback(one_cycle::Sample &sample) { one_cycle::collect_input(*this, sample); }
};

void run_one_cycle(ec_master_t *master, ec_slave_config_t *slave, unsigned mode, uint16_t lease,
                   std::vector<one_cycle::Sample> &trace, bool follow_hpm, one_cycle::CollectWait collect_mode)
{
    const unsigned rx_bytes = rx_lengths[mode], tx_bytes = tx_lengths[mode];
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
    require(follow_hpm, "SM2 timing requires the HPM follower");
#endif
#ifdef DCU_ONE_CYCLE_ENDURANCE
    require(follow_hpm, "endurance observation requires a locked HPM reference");
    unsigned misses = 0, consecutive = 0;
#endif
    require(one_cycle::nominal_window_fits(rx_bytes, tx_bytes), "PDO layout exceeds nominal one-cycle window");
    auto *output = ecrt_master_create_domain(master);
    auto *input = ecrt_master_create_domain(master);
    require(output && input, "could not create separate write and read domains");
    require(ecrt_slave_config_reg_pdo_entry_pos(slave, 2, 0, 0, output, nullptr) == 0 &&
            ecrt_slave_config_reg_pdo_entry_pos(slave, 3, 0, 0, input, nullptr) == 0 &&
            ecrt_domain_size(output) == rx_bytes && ecrt_domain_size(input) == tx_bytes,
            "split domains do not match the existing SM2/SM3 PDO layout");
    check_igh("IgH send interval", ecrt_master_set_send_interval, master,
              (one_cycle::period_ns - one_cycle::read_phase_ns) / one_cycle::ns_per_us);
    check_igh("IgH DC Sync0", ecrt_slave_config_dc, slave, 0x0300, one_cycle::period_ns,
              one_cycle::sync0_phase_ns, 0, 0);
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    one_cycle::FrameCounter counter;
#endif
    const auto origin = ig::IghMonotonicTimeNs();
    ig::ActivateIgh(master, slave, true, origin);
    IghOneCyclePort port{master, slave, output, input, ecrt_domain_data(output), ecrt_domain_data(input),
                         rx_bytes, lease, follow_hpm, collect_mode
#ifdef DCU_ONE_CYCLE_SOF_TRACE
                         , counter
#endif
    };
    require(port.output_data && port.input_data, "cannot access split domain data");

    auto tick = one_cycle::next_tick(origin, ig::IghMonotonicTimeNs());
    auto dc_tick = tick;
    uint32_t sequence = 1;
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    one_cycle::ReferenceFollower follower;
#endif
    const auto exchange = [&](bool acquisition) {
        int step = 0;
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        if (follow_hpm) {
            const int bound = acquisition ? follower.acquire_limit_ns : follower.tracking_limit_ns;
            step = std::clamp(follower.step_ns(), -bound, bound);
        }
#else
        (void)acquisition;
#endif
        const auto next = uint64_t(int64_t(tick + one_cycle::period_ns) + step);
        auto sample = one_cycle::exchange(port, tick, sequence++, next, dc_tick);
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        if (follow_hpm && sample.dc_sof_valid && sample.sm2_frames == 1 &&
            sample.sm2_frame_bytes >= rx_bytes + 48U && (acquisition || sample.output_wkc == 1))
            sample.servo_used = follower.observe(dc_tick, sample.sequence, sample.dc_sof_ns, acquisition);
#endif
        tick = next;
        dc_tick += one_cycle::period_ns;
        return sample;
    };
    const auto stop = [&] { check_igh("IgH deactivate", ecrt_master_deactivate, master); };
    unsigned stable = 0;
    one_cycle::Sample warmup{};
    const auto start_limit = Clock::now() + 8s;
    while (Clock::now() < start_limit && stable < 8) {
        warmup = exchange(true);
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
        if (!warmup.sm2_send_return_ns) break;
#endif
        if constexpr (one_cycle::sm3_prepare_ns) {
            if (warmup.read_ns >= warmup.deadline_ns) break;
        }
        const bool online = warmup.operational && warmup.output_wkc == 1 && warmup.input_wkc == 1;
        stable = online ? stable + 1 : 0;
    }
    if (stable != 8) {
        std::fprintf(stderr, "ONE_CYCLE_WARMUP_FAILED seq=%u ack=%u wkc=%u/%u al=%u\n",
                     warmup.sequence, warmup.ack, warmup.output_wkc, warmup.input_wkc, warmup.al);
        stop();
        throw std::runtime_error("split cycle did not stabilize in OP");
    }
#ifdef DCU_ONE_CYCLE_SOF_TRACE
    const auto report_warmup = [&, locked=follower.locked(), step=follower.step_ns()] {
        std::printf("DC_MODE=%s warmup_seq=%u dc_tick=%llu sof=%u valid=%d locked=%d step_ns=%d\n",
                follow_hpm ? "hpm-reference" : "rk-reference", warmup.sequence,
                static_cast<unsigned long long>(warmup.dc_tick_ns), warmup.dc_sof_ns,
                warmup.dc_sof_valid, locked, step);
    };
#ifndef DCU_ONE_CYCLE_SM2_PREPARE_US
    report_warmup();
#endif
    if (follow_hpm && (!follower.locked() || !warmup.servo_used)) {
        stop();
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
        report_warmup();
#endif
        throw std::runtime_error("RK did not lock to HPM by the end of the common OP warmup");
    }
#endif
    const auto finish = [&] {
        stop();
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
        report_warmup(); // No stdout in the gap between warmup feedback and the first scored SM2.
#endif
    };
    for (auto &sample : trace) {
        sample = exchange(false);
        if (!one_cycle::on_time(sample)) {
#ifdef DCU_ONE_CYCLE_ENDURANCE
            ++misses;
            if (++consecutive < 3 && one_cycle::may_observe_next(sample)) continue;
#endif
            std::fprintf(stderr, "ONE_CYCLE_MISS seq=%u ack=%u wkc=%u/%u al=%u\n",
                         sample.sequence, sample.ack, sample.output_wkc, sample.input_wkc, sample.al);
            finish();
            throw std::runtime_error("command did not have matching valid feedback by the next tick");
        }
#ifdef DCU_ONE_CYCLE_ENDURANCE
        consecutive = 0;
#endif
#ifdef DCU_ONE_CYCLE_SOF_TRACE
        if (follow_hpm && !sample.servo_used) {
            finish();
            throw std::runtime_error("lost same-frame HPM reference sample");
        }
#endif
    }
    finish();
#ifdef DCU_ONE_CYCLE_ENDURANCE
    std::printf("ONE_CYCLE_ENDURANCE_COMPLETE commands=%zu misses=%u\n", trace.size(), misses);
    require(!misses, "completed observation with missed one-cycle deadlines");
#endif
    std::printf("ONE_CYCLE_PASS commands=%zu\n", trace.size());
}

void run(Ticket old, unsigned hold_cycles, std::chrono::microseconds period,
         std::chrono::microseconds spin, bool defer_dc_time, bool split_rx_timing, bool check_rx_switches,
         unsigned ack_lag_limit,
         std::string_view exit_mode, std::vector<TimingSample> &trace,
         std::vector<one_cycle::Sample> &one_cycle_trace, bool follow_hpm, one_cycle::CollectWait collect_mode)
{
    std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> master(ecrt_request_master(0), ecrt_release_master);
    require(bool(master), "could not reserve IgH master");
    ec_master_info_t info{};
    check_igh("master info", ecrt_master, master.get(), &info);
    require(info.link_up && !info.scan_busy && info.slave_count == 1, "expected exactly one scanned slave");
    ec_slave_info_t slave{};
    check_igh("slave identity", ecrt_master_get_slave, master.get(), 0, &slave);
    require(slave.vendor_id == vendor && slave.product_code == product &&
            slave.revision_number == revision, "unexpected production slave identity");
    require(upload(master.get(), 0x2001, 1, 4) == 0, "motors are configured, stopping no-motion probe");
    const auto mode = upload(master.get(), 0x2001, 4, 1);
    const auto lease = upload(master.get(), 0x2001, 2, 2);
    require(lease >= 2 && lease <= 10, "unexpected active Profile command lease");
    const auto pdo = [lease](uint32_t seq, bool enabled = false) { return ordinary(seq, lease, enabled); };
    require(mode < std::size(rx_lengths), "unexpected PDO mapping mode");
    Ticket ticket{upload(master.get(), 0x2002, 1, 4), upload(master.get(), 0x2002, 2, 4), 0};
    const auto state = upload(master.get(), 0x2002, 3, 2);
    require(ticket.epoch && ticket.generation && state == 2 && upload(master.get(), 0x2002, 4, 4) == 0,
            "expected READY and no session before entering OP");
    require(!old.epoch || ticket.epoch > old.epoch ||
            (ticket.epoch == old.epoch && ticket.generation > old.generation), "previous generation is still active");
    std::printf("READY serial=%08x epoch=%u generation=%u mode=%u\n", slave.serial_number,
                ticket.epoch, ticket.generation, mode);

    auto *sc = ecrt_master_slave_config(master.get(), 0, 0, vendor, product);
    require(sc, "could not bind slave configuration");
    if (!one_cycle_trace.empty()) {
        run_one_cycle(master.get(), sc, mode, lease, one_cycle_trace, follow_hpm, collect_mode);
        return;
    }
    auto *domain = ecrt_master_create_domain(master.get());
    require(domain, "could not bind SM domain");
    auto *al_request = ecrt_slave_config_create_reg_request(sc, 6);
    require(al_request, "could not allocate AL diagnostic request");
    const int rx = ecrt_slave_config_reg_pdo_entry_pos(sc, 2, 0, 0, domain, nullptr);
    const int tx = ecrt_slave_config_reg_pdo_entry_pos(sc, 3, 0, 0, domain, nullptr);
    const auto bytes = ecrt_domain_size(domain);
    require(rx >= 0 && tx >= 0 && unsigned(rx) + rx_lengths[mode] <= bytes &&
            unsigned(tx) + tx_lengths[mode] <= bytes && bytes == rx_lengths[mode] + tx_lengths[mode] &&
            (rx + rx_lengths[mode] <= unsigned(tx) || tx + tx_lengths[mode] <= unsigned(rx)),
            "PDO domain length or offset differs from production layout");
    check_igh("send interval", ecrt_master_set_send_interval, master.get(),
                 std::chrono::duration_cast<std::chrono::microseconds>(period).count());
    check_igh("DC Sync0", ecrt_slave_config_dc, sc, 0x0300,
                 std::chrono::duration_cast<std::chrono::nanoseconds>(period).count(), 0, 0, 0);
    ig::ActivateIgh(master.get(), sc, true, ig::IghMonotonicTimeNs());
    auto *data = ecrt_domain_data(domain);
    require(data, "cannot access domain");
    auto tick = Clock::now();
    bool online = false;
    unsigned ack = 0, al = 0, wkc = 0, wc_state = 0;
    int64_t wake_max_us = 0;
    uint64_t last_send_us = 0;
    const auto exchange = [&](const Image &image, TimingSample *sample = nullptr) {
        const auto due = tick;
        const auto wake = wait_for_tick(due, spin);
        const auto wake_us = std::chrono::duration_cast<std::chrono::microseconds>(wake - tick).count();
        wake_max_us = std::max(wake_max_us, wake_us);
        tick += period;
        const auto app_time = ig::IghMonotonicTimeNs();
        const bool defer = defer_dc_time && sample;
        uint64_t receive_start_ns = 0, receive_end_ns = 0, domain_end_ns = 0;
        struct rusage before;
        if (split_rx_timing && sample) {
            check_igh("IgH application time", ecrt_master_application_time, master.get(), app_time);
            if (check_rx_switches)
                require(getrusage(RUSAGE_THREAD, &before) == 0, "could not read starting thread switch counts");
            receive_start_ns = ig::IghMonotonicTimeNs();
            check_igh("IgH receive", ecrt_master_receive, master.get());
            receive_end_ns = ig::IghMonotonicTimeNs();
            check_igh("IgH domain process", ecrt_domain_process, domain);
            domain_end_ns = ig::IghMonotonicTimeNs();
        } else if (defer) {
            check_igh("IgH receive", ecrt_master_receive, master.get());
            check_igh("IgH domain process", ecrt_domain_process, domain);
        } else {
            ig::ReceiveIghCycle(master.get(), domain, app_time);
        }
        ack = EC_READ_U32(data + tx + 4);
        const auto feedback_ns = sample ? ig::IghMonotonicTimeNs() : 0;
        long voluntary_switches = -1, involuntary_switches = -1;
        if (check_rx_switches && sample && receive_end_ns - receive_start_ns >= slow_receive_ns) {
            // Snapshot outside the timed calls: zero rules out a switch across the whole window.
            struct rusage after;
            require(getrusage(RUSAGE_THREAD, &after) == 0, "could not read ending thread switch counts");
            voluntary_switches = after.ru_nvcsw - before.ru_nvcsw;
            involuntary_switches = after.ru_nivcsw - before.ru_nivcsw;
        }
        // Keep the cycle-start timestamp and update it before DC sync/send, off the receive path.
        if (defer) check_igh("IgH application time", ecrt_master_application_time, master.get(), app_time);
        ec_slave_config_state_t status{};
        ec_domain_state_t domain_state{};
        check_igh("AL state", ecrt_slave_config_state, sc, &status);
        check_igh("PDO working counter", ecrt_domain_state, domain, &domain_state);
        al = status.al_state;
        wkc = domain_state.working_counter;
        wc_state = domain_state.wc_state;
        online = status.online && status.operational && domain_state.wc_state == EC_WC_COMPLETE;
        std::copy_n(image.data(), rx_lengths[mode], data + rx);
        const auto send_ns = ig::IghMonotonicTimeNs();
        ig::SendIghCycle(master.get(), domain, true, send_ns);
        const auto send_return_ns = ig::IghMonotonicTimeNs();
        last_send_us = send_return_ns / 1000;
        if (sample) {
            *sample = {monotonic_ns(due), monotonic_ns(wake), send_ns, send_return_ns,
                       feedback_ns, EC_READ_U32(image.data()), ack, wkc, al, online,
                       receive_start_ns, receive_end_ns, domain_end_ns, voluntary_switches, involuntary_switches};
        }
    };
    const auto report_al = [&](const char *label, uint32_t neutral_sequence) {
        std::printf("%s first_observed_al=%u wkc=%u last_send_us=%llu\n",
                    label, al, wkc, static_cast<unsigned long long>(last_send_us));
        ecrt_reg_request_read(al_request, 0x0130, 6);
        const auto limit = Clock::now() + 100ms;
        while (ecrt_reg_request_state(al_request) == EC_REQUEST_BUSY && Clock::now() < limit)
            exchange(pdo(neutral_sequence++));
        if (ecrt_reg_request_state(al_request) == EC_REQUEST_SUCCESS) {
            const auto *reg = ecrt_reg_request_data(al_request);
            std::printf("%s al_status=%04x al_code=%04x\n", label,
                        unsigned(EC_READ_U16(reg)), unsigned(EC_READ_U16(reg + 4)));
        } else {
            std::printf("%s AL register read did not complete\n", label);
        }
    };
    unsigned stable = 0;
    uint32_t warmup_sequence = 1;
    const auto op_timeout = Clock::now() + 8s;
    while (Clock::now() < op_timeout && stable < 8) {
        exchange(trace.empty() ? pdo(100, true) : pdo(warmup_sequence++));
        stable = online ? stable + 1 : 0;
    }
    if (stable != 8 || (trace.empty() && ack)) {
        std::fprintf(stderr, "OP check: stable=%u al=%u wkc=%u wc_state=%u ack=%u\n", stable, al, wkc, wc_state, ack);
        report_al("START_FAILURE", 101);
        throw std::runtime_error("OP did not stabilize or an enabled PDO was accepted before neutral");
    }
    std::printf("OP period_us=%lld wkc=%u rx=%u tx=%u %s=%u\n",
                static_cast<long long>(period.count()), wkc, rx_lengths[mode], tx_lengths[mode],
                trace.empty() ? "before_neutral_ack" : "warmup_ack", ack);

    if (old.epoch) {
        for (unsigned i = 0; i < 8; ++i)
            exchange(pdo(old.sequence, true));
        require(online && ack == 0, "old enabled PDO established a baseline");
    }
    if (lease < 10 && trace.empty()) {
        for (unsigned i = 0; i < 6; ++i)
            exchange(ordinary(100, 10));
        require(online && ack == 0, "old lease was accepted");
        std::printf("OLD_LEASE_REJECTED requested=10 active=%u\n", lease);
    }
    const uint32_t first = !trace.empty() ? warmup_sequence : old.epoch ? old.sequence + 1 : 101;
    require(first && first <= UINT32_MAX - 20, "probe sequence would wrap");
    if (hold_cycles) {
        require(!old.epoch && hold_cycles >= 4 && hold_cycles < UINT32_MAX - first - 1000,
                "hold requires at least four cycles and no replay ticket");
        // Stream fresh zero-enable PDOs from the first frame. Repeating the
        // baseline sequence while awaiting its ACK can itself expire a lease.
        wake_max_us = 0;
        const auto first_ack_timeout = Clock::now() + 12ms;
        unsigned ack_lag_max = 0, first_ack_cycle = 0, previous_ack = 0;
        for (uint32_t i = 0; i < hold_cycles; ++i) {
            const uint32_t sent = first + i;
            exchange(pdo(sent), trace.empty() ? nullptr : &trace[i]);
            if (!first_ack_cycle && online && ack >= first && ack < sent)
                first_ack_cycle = i + 1;
            const bool awaiting_first = !first_ack_cycle && ack == 0 && (i < 3 || Clock::now() < first_ack_timeout);
            const bool invalid_ack = !awaiting_first && (ack < first || ack < previous_ack ||
                                                         ack >= sent || sent - ack > ack_lag_limit);
            if (trace.empty() && (!online || invalid_ack)) {
                std::fprintf(stderr, "OP hold: cycle=%u al=%u wkc=%u ack=%u sent=%u wake_max_us=%lld\n",
                             i, al, wkc, ack, sent, static_cast<long long>(wake_max_us));
                report_al("RUN_FAILURE", sent + 1);
                throw std::runtime_error("ordinary PDO did not stay operational");
            }
            if (online && ack >= first && ack < sent) {
                ack_lag_max = std::max(ack_lag_max, sent - ack);
                previous_ack = ack;
            }
        }
        if (trace.empty()) require(first_ack_cycle, "zero-enable stream was never acknowledged");
        std::printf("HOLD cycles=%u wkc=%u ack=%u first_ack_cycle=%u ack_lag_max=%u wake_max_us=%lld\n",
                    hold_cycles, wkc, ack, first_ack_cycle, ack_lag_max, static_cast<long long>(wake_max_us));
        uint32_t next = first + hold_cycles;
        // Keep the zero-enable stream running while collecting the in-flight tail.
        for (unsigned i = hold_cycles; i < trace.size(); ++i)
            exchange(pdo(next++), &trace[i]);
        if (exit_mode == "graceful") {
            std::printf("STOP_REQUEST preop last_send_us=%llu\n", static_cast<unsigned long long>(last_send_us));
            request_legacy_preop(master.get());
            const auto limit = Clock::now() + 2s;
            while (al != 2 && Clock::now() < limit)
                exchange(pdo(next++));
            require(al == 2, "graceful stop did not reach PREOP while cyclic sending continued");
            report_al("STOP_PREOP", next);
        } else if (exit_mode == "gap") {
            std::printf("STOP_GAP cycles=4 last_send_us=%llu\n", static_cast<unsigned long long>(last_send_us));
            std::this_thread::sleep_for(4 * period);
            tick = Clock::now();
            exchange(pdo(next++));
            report_al("STOP_GAP_RESULT", next);
        }
        std::printf("PASS epoch=%u generation=%u ack=%u\n", ticket.epoch, ticket.generation, ack);
        return;
    }
    for (unsigned i = 0; i < 6 && ack != first; ++i)
        exchange(pdo(first));
    require(online && ack == first, "ordinary zero-enable PDO was not accepted");
    uint32_t next = first + 1;
    if (old.epoch) {
        // Keep the lease active while making the old frame the only candidate
        // in its cycle. Waiting on stale traffic alone is also a lease test.
        exchange(pdo(next++));
        exchange(pdo(old.sequence, true));
        for (unsigned i = 0; i < 3; ++i) {
            exchange(pdo(next++));
            if (!online || ack < first)
                std::fprintf(stderr, "replay window: online=%d old=%u first=%u ack=%u\n",
                             online, old.sequence, first, ack);
            require(online && ack >= first, "old sequence accepted or PDO lease lost during replay");
        }
        std::printf("OLD_ENABLED_PDO_REJECTED old=%u baseline=%u ack=%u\n", old.sequence, first, ack);
    }
    for (uint32_t seq = next; seq <= first + 20; ++seq)
        exchange(pdo(seq, seq == first + 20));
    for (unsigned i = 0; i < 6 && ack != first + 20; ++i)
        exchange(pdo(first + 20, true));
    require(online && ack == first + 20, "final PDO not acknowledged");
    std::printf("PASS epoch=%u generation=%u ack=%u\n", ticket.epoch, ticket.generation, ack);
}
}

int main(int argc, char **argv)
{
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    const char *timing_file = nullptr;
    std::vector<TimingSample> trace;
    std::vector<one_cycle::Sample> one_cycle_trace;
    try {
        unsigned hold_cycles = 0, ack_lag_limit = 3, timing_cpu = 4;
        std::chrono::microseconds period = default_period, spin{};
        bool defer_dc_time = false, split_rx_timing = false, check_rx_switches = false, strict_one_cycle = false;
        std::string_view exit_mode = "release";
        std::string_view dc_mode, collect_wait;
        while (argc >= 3) {
            const auto option = std::string_view(argv[1]);
            if (option == "--hold-cycles")
                hold_cycles = number(argv[2]);
            else if (option == "--period-us")
                period = std::chrono::microseconds(number(argv[2]));
            else if (option == "--ack-lag-cycles")
                ack_lag_limit = number(argv[2]);
            else if (option == "--exit")
                exit_mode = argv[2];
            else if (option == "--timing-file")
                timing_file = argv[2];
            else if (option == "--timing-cpu")
                timing_cpu = number(argv[2]);
            else if (option == "--spin-us")
                spin = std::chrono::microseconds(number(argv[2]));
            else if (option == "--defer-dc-time") {
                const auto value = number(argv[2]);
                require(value <= 1, "defer-dc-time must be 0 or 1");
                defer_dc_time = value;
            } else if (option == "--split-rx-timing") {
                const auto value = number(argv[2]);
                require(value <= 1, "split-rx-timing must be 0 or 1");
                split_rx_timing = value;
            } else if (option == "--check-rx-switches") {
                const auto value = number(argv[2]);
                require(value <= 1, "check-rx-switches must be 0 or 1");
                check_rx_switches = value;
            } else if (option == "--collect-wait") {
                require(collect_wait.empty(), "choose exactly one collect wait");
                collect_wait = argv[2];
                require(collect_wait == "legacy" || collect_wait == "absolute" || collect_wait == "early", "unknown collect wait");
            } else if (option == "--dc-mode") {
                require(dc_mode.empty(), "choose exactly one DC mode");
                dc_mode = argv[2];
                require(dc_mode == "rk-reference" || dc_mode == "hpm-reference", "unknown DC mode");
            } else if (option == "--one-cycle") {
                const auto value = number(argv[2]);
                require(value <= 1, "one-cycle must be 0 or 1");
                strict_one_cycle = value;
            } else
                break;
            argc -= 2;
            argv += 2;
        }
        require(argc == 1 || argc == 4,
                "usage: probe [--period-us N] [--hold-cycles N] [--ack-lag-cycles N] [--exit release|graceful|gap] "
                "[--timing-file CSV] [--timing-cpu N] [--spin-us N] [--defer-dc-time 0|1] [--split-rx-timing 0|1] "
                "[--check-rx-switches 0|1] [--one-cycle 0|1] [--dc-mode rk-reference|hpm-reference] "
                "[--collect-wait legacy|absolute|early] [OLD_EPOCH OLD_GENERATION OLD_CYCLE_SEQ]");
        require(period >= 1ms && period <= 5ms, "period must be 1000..5000 us");
        require(spin <= max_spin && (!spin.count() || timing_file), "spin requires timing mode and at most 50 us");
        require(!defer_dc_time || timing_file, "deferred DC time update requires timing mode");
        require(!split_rx_timing || (timing_file && !spin.count() && !defer_dc_time),
                "split receive timing requires the original receive path and a timing file");
        require(!check_rx_switches || split_rx_timing, "thread switch check requires split receive timing");
        require(dc_mode.empty() || strict_one_cycle, "DC comparison requires the one-cycle probe");
        require(collect_wait.empty() || strict_one_cycle, "collect wait requires the one-cycle probe");
#ifndef DCU_ONE_CYCLE_COLLECT_TRACE
        require(collect_wait.empty(), "collect wait requires DCU_ONE_CYCLE_COLLECT_TRACE");
#endif
        const auto collect_mode = collect_wait == "early" ? one_cycle::CollectWait::early :
                                  collect_wait == "absolute" ? one_cycle::CollectWait::absolute : one_cycle::CollectWait::legacy;
        if (strict_one_cycle && dc_mode.empty()) dc_mode = "hpm-reference";
#ifndef DCU_ONE_CYCLE_SOF_TRACE
        require(dc_mode != "hpm-reference", "following HPM requires a probe built with DCU_ONE_CYCLE_SOF_TRACE");
#endif
        require(!strict_one_cycle || (period == 1ms && timing_file && !spin.count() &&
                                     !defer_dc_time && !split_rx_timing && !check_rx_switches),
                "one-cycle requires 1 ms timing mode without legacy timing options");
        require(timing_cpu < CPU_SETSIZE && (timing_cpu == 4 || timing_file), "invalid timing CPU");
        require(ack_lag_limit >= 1 && ack_lag_limit <= 12, "ack lag limit must be 1..12 cycles");
        require(exit_mode == "release" || (hold_cycles && (exit_mode == "graceful" || exit_mode == "gap")),
                "graceful and gap exits require --hold-cycles");
        if (timing_file) {
            require(argc == 1 && hold_cycles >= 4 && hold_cycles <= max_timing_cycles && exit_mode == "graceful",
                    "timing capture requires a bounded hold and graceful exit");
            if (strict_one_cycle)
                one_cycle_trace.resize(hold_cycles);
            else
                trace.resize(hold_cycles + timing_drain_cycles);
            cpu_set_t cpus;
            sched_param scheduling{};
            require(sched_getscheduler(0) == SCHED_FIFO && sched_getparam(0, &scheduling) == 0 &&
                    scheduling.sched_priority == 80 && sched_getaffinity(0, sizeof(cpus), &cpus) == 0 &&
                    CPU_COUNT(&cpus) == 1 && CPU_ISSET(timing_cpu, &cpus),
                    "timing capture requires the selected CPU and FIFO 80");
            require(mlockall(MCL_CURRENT | MCL_FUTURE) == 0, "memory lock failed");
            std::printf("TIMING cpu=%u fifo=80 mlockall=1 samples=%u spin_us=%lld defer_dc_time=%d split_rx_timing=%d\n",
                        timing_cpu, hold_cycles, static_cast<long long>(spin.count()), defer_dc_time, split_rx_timing);
            if (check_rx_switches)
                std::printf("RX_SWITCH_CHECK slow_receive_ns=%llu\n", static_cast<unsigned long long>(slow_receive_ns));
            if (strict_one_cycle)
                std::printf("ONE_CYCLE Sync0_us=%llu read_us=%llu collect_us=%llu publish_deadline_us=%llu "
                            "publish_advance_us=%u build_budget_us=%u sm3_prepare_us=%llu\n",
                            static_cast<unsigned long long>(one_cycle::sync0_phase_ns / one_cycle::ns_per_us),
                            static_cast<unsigned long long>(one_cycle::read_phase_ns / one_cycle::ns_per_us),
                            static_cast<unsigned long long>(one_cycle::collect_phase_ns / one_cycle::ns_per_us),
                            static_cast<unsigned long long>(one_cycle::publish_deadline_ns / one_cycle::ns_per_us),
                            static_cast<unsigned>(DCU_TX_PUBLISH_ADVANCE_US),
                            static_cast<unsigned>(DCU_TX_BUILD_BUDGET_US),
                            static_cast<unsigned long long>(one_cycle::sm3_prepare_ns / one_cycle::ns_per_us));
#ifdef DCU_ONE_CYCLE_SM2_PREPARE_US
            if (strict_one_cycle)
                std::printf("SM2_TIMING prepare_us=%llu send_phase_us=0\n",
                            static_cast<unsigned long long>(one_cycle::sm2_prepare_ns / one_cycle::ns_per_us));
#endif
#ifdef DCU_ONE_CYCLE_COLLECT_TRACE
            if (strict_one_cycle) {
                std::printf("COLLECT_TIMING wait=%s clock=monotonic\n", collect_wait.empty() ? "legacy" : collect_wait.data());
                if (collect_mode == one_cycle::CollectWait::early)
                    std::printf("COLLECT_PREWAKE wake_us=%llu gate_us=%llu\n",
                                static_cast<unsigned long long>((one_cycle::collect_phase_ns - one_cycle::collect_advance_ns) / one_cycle::ns_per_us),
                                static_cast<unsigned long long>(one_cycle::collect_phase_ns / one_cycle::ns_per_us));
            }
#endif
        }
        run(argc == 4 ? Ticket{number(argv[1]), number(argv[2]), number(argv[3])} : Ticket{},
            hold_cycles, period, spin, defer_dc_time, split_rx_timing, check_rx_switches, ack_lag_limit, exit_mode,
            trace, one_cycle_trace, dc_mode == "hpm-reference", collect_mode);
        if (timing_file) {
            const bool saved = strict_one_cycle ? write_one_cycle_timing(timing_file, one_cycle_trace) :
                                                  write_timing(timing_file, trace);
            require(saved, "could not save timing CSV");
        }
        return 0;
    } catch (const std::exception &error) {
        std::fprintf(stderr, "probe: %s\n", error.what());
        if (timing_file && !trace.empty() && trace[0].sent && !write_timing(timing_file, trace))
            std::fprintf(stderr, "probe: could not save partial timing CSV\n");
        if (timing_file && !one_cycle_trace.empty() && one_cycle_trace[0].sequence &&
            !write_one_cycle_timing(timing_file, one_cycle_trace))
            std::fprintf(stderr, "probe: could not save partial one-cycle timing CSV\n");
        return 1;
    }
}
