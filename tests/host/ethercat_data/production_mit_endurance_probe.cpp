#include <ecrt.h>
#include "dcu_igh_layout.hpp"
#include "encos_interface.h"
#include "igh_probe_frame_stats.hpp"
#include "probe_dc_clock.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <sys/mman.h>
#include <time.h>
#include <vector>

namespace {
#if defined(DCU_PROBE_CAN2_ENCOS)
constexpr unsigned bus = 2, first_slot = 14, axes = 4, first_can_id = 1;
constexpr bool encos_axes = true;
#else
constexpr unsigned bus = 3, first_slot = 21, axes = 1, first_can_id = 2;
constexpr bool encos_axes = false;
#endif
constexpr auto &wire = dcu_profile_pdo::modes[1];
constexpr unsigned rx_size = wire.rx.bytes, tx_size = wire.tx.bytes;
constexpr int channel_offset = offset(wire.rx, 0x7100 + bus);
constexpr int axis_rx_offset = offset(wire.rx, 0x7200 + first_slot);
constexpr int axis_tx_offset = offset(wire.tx, 0x6200 + first_slot);
constexpr bool target_mapped()
{
    if (channel_offset < 0 || axis_tx_offset < 0 || axis_rx_offset < 0) return false;
    for (unsigned i = 0; i < axes; ++i)
        if (offset(wire.rx, 0x7200 + first_slot + i) != axis_rx_offset + int(i * sizeof(mc_axis_rx_t)) ||
            offset(wire.tx, 0x6200 + first_slot + i) != axis_tx_offset + int(i * sizeof(mc_axis_tx_t))) return false;
    return true;
}
static_assert(target_mapped(), "probe targets are absent from the selected Profile contract");
constexpr unsigned local_mask = (1U << axes) - 1, axis_mask = local_mask << first_slot;
constexpr uint32_t configured_mask = axis_mask | (1U << 21);
constexpr unsigned period_us = 2000, period_ns = period_us * 1000;
constexpr unsigned planned_frames = 30000;
constexpr int sync0_ns = 375000;
constexpr uint64_t receive_phase_ns = 550000;
constexpr uint16_t start = MC_AXIS_ARM | MC_AXIS_START_MOTION;
constexpr uint16_t fresh = 16U, board_faults = 0x7cU;
using Follower = one_cycle::BasicReferenceFollower<period_ns>;

uint64_t now()
{
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return uint64_t(t.tv_sec) * 1000000000 + t.tv_nsec;
}
void wait(uint64_t end)
{
    const auto early = end - 100000;
    const timespec t{time_t(early / 1000000000), long(early % 1000000000)};
    int rc;
    do { rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, nullptr); } while (rc == EINTR);
    if (rc) throw std::runtime_error("clock_nanosleep failed");
    while (now() < end) {}
}
void need(bool condition, const char *message)
{
    if (!condition) throw std::runtime_error(message);
}
uint32_t upload(ec_master_t *master, uint16_t index, uint8_t sub, size_t width)
{
    uint8_t value[4]{};
    size_t size{};
    uint32_t abort{};
    need(!ecrt_master_sdo_upload(master, 0, index, sub, value, sizeof(value), &size, &abort) &&
         !abort && size == width, "production Profile readback failed");
    return EC_READ_U32(value);
}
struct Sample {
    uint64_t sent{}, received{}, frames{}, bytes{}, due{};
    uint32_t seq{}, sof{}, grid{};
    unsigned wkc{}, al{};
    bool ok{};
    mc_board_tx_t board{};
    std::array<mc_axis_tx_t, axes> axis{};
};
struct Session {
    ec_master_t *master;
    ec_domain_t *domain;
    ec_slave_config_t *config;
    one_cycle::FrameCounter &counter;
    uint8_t *pdo;
    uint64_t due, dc;
    Follower follower{};
    uint32_t sequence{};

    Sample cycle(uint16_t op = 0, bool exact = false, bool prepare = false, bool reset_board = false)
    {
        Sample result{};
        result.seq = ++sequence;
        result.due = due;
        const uint64_t next = uint64_t(int64_t(due + period_ns) + follower.step_ns());
        wait(due - 50000);
        const auto before = counter.read();
        if (!before.valid) return result;
        std::memset(pdo, 0, rx_size);
        mc_board_rx_t board{};
        board.CycleSeq = result.seq;
        board.CommandLeaseMs = 10;
        board.InterfaceVersion = MC_INTERFACE_VERSION;
        if (reset_board && !op && !prepare) board.BoardControl = 4;
        if (op || prepare) {
            board.BoardControl = 1;
            board.ChannelEnableMask = 1U << bus;
            board.AxisEnableMask = axis_mask;
            const mc_channel_rx_t channel{1, local_mask, 1, period_us};
            mc_axis_rx_t axis{};
            axis.Mode = 4;
            axis.ControlWord = op;
            std::memcpy(pdo + channel_offset, &channel, sizeof(channel));
            for (unsigned n = 0; n < axes; ++n)
                std::memcpy(pdo + axis_rx_offset + n * sizeof(axis), &axis, sizeof(axis));
        }
        std::memcpy(pdo, &board, sizeof(board));
        ecrt_master_application_time(master, dc + now() - due);
        ecrt_master_sync_monitor_queue(master);
        ecrt_master_sync_slave_clocks(master);
        while (now() < due) {}
        if ((op || prepare) && now() - due > 45000) return result;
        ecrt_domain_queue(domain);
        result.sent = now();
        ecrt_master_send(master);
        wait(due + receive_phase_ns);
        ecrt_master_receive(master);
        ecrt_domain_process(domain);
        result.received = now();
        ec_domain_state_t ds{};
        ec_slave_config_state_t sc{};
        ecrt_domain_state(domain, &ds);
        ecrt_slave_config_state(config, &sc);
        const auto after = counter.read();
        if (after.valid && after.frames >= before.frames && after.bytes >= before.bytes) {
            result.frames = after.frames - before.frames;
            result.bytes = after.bytes - before.bytes;
        }
        result.wkc = ds.working_counter;
        result.al = sc.al_state;
        const bool ref = result.frames == 1 && !ecrt_master_reference_clock_time(master, &result.sof) &&
                         follower.observe(dc, result.seq, result.sof, !exact);
        result.grid = uint32_t(dc);
        const auto phase = int32_t(result.sof - result.grid);
        const bool in_phase = std::abs(int64_t(phase) - Follower::target_sof_ns) <= 10000;
        result.ok = result.frames == 1 && result.bytes >= std::max(rx_size, tx_size) + 28 && result.bytes <= 1514 && ref && in_phase &&
            ds.wc_state == EC_WC_COMPLETE && result.wkc == 3 && sc.operational &&
            result.sent < due + 45000 && result.received < due + period_ns;
        std::memcpy(&result.board, pdo, sizeof(result.board));
        std::memcpy(result.axis.data(), pdo + axis_tx_offset, sizeof(mc_axis_tx_t) * axes);
        due = next;
        dc += period_ns;
        return result;
    }
};
void show(const char *tag, const Sample &x)
{
    printf("%s seq=%u accepted=%u publish=%u due=%llu send=%llu receive=%llu frames=%llu bytes=%llu "
           "WKC=%u AL=%u ok=%d status=0x%x axis=0x%x valid=0x%x position=%d age=%u sof=%u grid=%u\n",
           tag, x.seq, x.board.AcceptedCycleSeq, x.board.PublishSeq, (unsigned long long)x.due,
           (unsigned long long)x.sent, (unsigned long long)x.received, (unsigned long long)x.frames,
           (unsigned long long)x.bytes, x.wkc, x.al, x.ok, x.board.BoardStatus, x.axis[0].StatusWord,
           x.axis[0].ValidFlags, x.axis[0].ActualPosition, x.axis[0].FeedbackAgeMs, x.sof, x.grid);
    if (encos_axes)
        for (unsigned i = 0; i < axes; ++i) {
            const auto &a = x.axis[i];
            printf("can%u ID=%u mode=%u axis=0x%x valid=0x%x age=%u motor_fault=%u bridge_fault=%u\n",
                   bus, first_can_id + i, a.ModeDisplay, a.StatusWord, a.ValidFlags, a.FeedbackAgeMs,
                   a.MotorFaultRaw, a.BridgeFaultCode);
        }
}

constexpr bool matches(const Sample &sample, uint32_t expected, bool expect_feedback)
{
    if (!sample.ok || sample.board.AcceptedCycleSeq != expected || (sample.board.BoardStatus & board_faults))
        return false;
    for (const auto &axis : sample.axis) {
        if (expect_feedback) {
            if (!(axis.StatusWord & fresh) || (axis.ValidFlags & 7U) != 7U ||
                axis.MotorFaultRaw || axis.BridgeFaultCode || axis.FeedbackAgeMs > period_us / 1000 ||
                (encos_axes && (axis.ModeDisplay != 4 || !(axis.StatusWord & 8U)))) return false;
        } else if (encos_axes ? (axis.MotorFaultRaw || axis.BridgeFaultCode || (axis.StatusWord & 8U)) :
                               (axis.ValidFlags || (axis.StatusWord & fresh))) return false;
    }
    return true;
}

struct RunRow {
    Sample frame;
    uint32_t commanded{}, expected{};
    bool verified{};
};

void write_run(FILE *file, const std::vector<RunRow> &rows)
{
    fputs("seq,commanded,expected,accepted,publish,due_ns,send_ns,receive_ns,frames,bytes,wkc,al,"
          "sof_ns,grid_ns,lrw_ok,next_lrw_ok,board,axis,valid,age_ms,position,velocity,current,motor_fault,bridge_fault,can_id,mode\n",
          file);
    for (const auto &row : rows) {
        const auto &r = row.frame;
        for (unsigned i = 0; i < axes; ++i) {
            const auto &a = r.axis[i];
            fprintf(file, "%u,%u,%u,%u,%u,%llu,%llu,%llu,%llu,%llu,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%d,%d,%d,%u,%u,%u,%u\n",
                r.seq, row.commanded, row.expected, r.board.AcceptedCycleSeq, r.board.PublishSeq,
                (unsigned long long)r.due, (unsigned long long)r.sent, (unsigned long long)r.received,
                (unsigned long long)r.frames, (unsigned long long)r.bytes, r.wkc, r.al, r.sof, r.grid,
                r.ok, row.verified, r.board.BoardStatus, a.StatusWord, a.ValidFlags,
                a.FeedbackAgeMs, a.ActualPosition, a.ActualVelocity, a.ActualCurrent,
                a.MotorFaultRaw, a.BridgeFaultCode, first_can_id + i, a.ModeDisplay);
        }
    }
    need(fflush(file) == 0 && !ferror(file), "could not write endurance evidence");
}
}

int main(int argc, char **argv)
try {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    const bool once = argc == 3 && encos_axes && !std::strcmp(argv[1], "--mit-zero-once");
    const bool fire = once || (argc == 3 && !std::strcmp(argv[1], "--mit-zero-60s"));
    need(fire || (argc == 2 && !std::strcmp(argv[1], "--dry-run")),
         "explicit --dry-run, --mit-zero-once CSV (CAN2), or --mit-zero-60s CSV required");
    const unsigned goal = once ? 1U : planned_frames;
    const unsigned warmup_goal = once ? 32U : 64U;
    std::unique_ptr<FILE, decltype(&fclose)> evidence(fire ? fopen(argv[2], "wx") : nullptr, fclose);
    need(!fire || bool(evidence), "cannot create exclusive endurance CSV");
    std::vector<RunRow> rows;
    if (fire) rows.reserve(goal + 1U);
    need(mlockall(MCL_CURRENT | MCL_FUTURE) == 0, "mlockall failed");
    one_cycle::FrameCounter counter;
    std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> m(ecrt_request_master(0), ecrt_release_master);
    need(bool(m), "cannot request IgH master");
    ec_master_info_t mi{};
    ec_slave_info_t si{};
    need(!ecrt_master(m.get(), &mi) && mi.link_up && !mi.scan_busy && mi.slave_count == 1, "not one isolated slave");
    need(!ecrt_master_get_slave(m.get(), 0, &si) && si.vendor_id == 0x10000 && si.product_code == 2 &&
         si.revision_number == 2 && si.al_state == 2, "production HPM in PREOP was not identified");
    need(upload(m.get(), 0x2100, 4, 4) == dcu_profile_pdo::profile_id &&
         upload(m.get(), 0x2100, 5, 4) == dcu_profile_pdo::profile_revision &&
         upload(m.get(), 0x2001, 1, 4) == configured_mask, "unexpected production Profile or axis mask");
    need(dcu_profile_pdo::verify(m.get(), 0, 1), "Profile layout digest, assignments or IgH entries differ");
    for (unsigned i = 0; i < axes; ++i)
        need(upload(m.get(), 0x3000 + 16 * (first_slot + i), 1, 2) == first_can_id + i &&
             upload(m.get(), 0x3000 + 16 * (first_slot + i), 2, 2) == (encos_axes ? 0x8000U : 0U),
             "unexpected production CAN route");
    auto *s = ecrt_master_slave_config(m.get(), 0, 0, 0x10000, 2);
    auto *d = ecrt_master_create_domain(m.get());
    need(s && d, "cannot create slave configuration and domain");
    ecrt_slave_config_overlapping_pdos(s, 1);
    auto out = dcu_profile_pdo::register_image(s, d, 2, wire.rx);
    auto in = dcu_profile_pdo::register_image(s, d, 3, wire.tx);
    need(out == 0 && in == 0 && ecrt_domain_size(d) == std::max(rx_size, tx_size), "unexpected Profile domain offsets");
    ecrt_slave_config_dc(s, 0x300, period_ns, sync0_ns, 0, 0);
    need(!ecrt_master_select_reference_clock(m.get(), s), "cannot select DC reference");
    const auto origin = now();
    ecrt_master_application_time(m.get(), origin);
    printf("period_us=%u sync0_shift_ns=%d\n", period_us, sync0_ns);
    need(!ecrt_master_set_send_interval(m.get(), period_us) && !ecrt_master_activate(m.get()), "cannot activate master");
    auto *pdo = ecrt_domain_data(d);
    need(pdo, "missing PDO image");
    const auto base = origin + ((now() - origin) / period_ns + 2) * period_ns;
    Session session{m.get(), d, s, counter, pdo, base, base};
    unsigned good = 0;
    unsigned stable_before_reset = 0;
    Sample last{};
    unsigned startup_fault_clears = 0;
    for (unsigned i = 0; i < 5000 && good < warmup_goal; ++i) {
        last = session.cycle();
        auto readiness = last;
        readiness.board.BoardStatus &= ~0x40U;
        stable_before_reset = matches(readiness, last.seq - 1, false) && session.follower.locked() ?
                              stable_before_reset + 1U : 0U;
        if (encos_axes && startup_fault_clears < (once ? 3U : 1U) &&
            stable_before_reset >= (once ? 8U : 1U) && (last.board.BoardStatus & board_faults) == 0x40U) {
            // Clear only the invalid-image latch, with all axes disabled.
            last = session.cycle(0, false, false, true);
            ++startup_fault_clears;
            stable_before_reset = 0;
            good = 0;
            printf("Zero-output board reset %u; %u fault-free frames still required\n", startup_fault_clears, warmup_goal);
        }
        good = matches(last, last.seq - 1, false) && session.follower.locked()
               ? good + 1 : 0;
    }
    show("warm", last);
    printf("warm_consecutive=%u\n", good);
    need(good == warmup_goal, "warmup did not pass; no MIT triggered");
    if (!fire) {
        ecrt_master_deactivate(m.get());
        return 0;
    }
    if (encos_axes) {
        const auto ready_to_arm = session.cycle(0, true, true);
        need(matches(ready_to_arm, last.seq, false), "production axis preparation failed; no MIT sent");
        last = ready_to_arm;
    }
    auto arm = session.cycle(MC_AXIS_ARM, true);
    need(matches(arm, last.seq, false), "arm cycle did not pass; no MIT triggered");
    if (encos_axes) {
        const auto all_ready = [&] {
            return std::all_of(arm.axis.begin(), arm.axis.end(), [](const auto &a) { return a.StatusWord & 4U; });
        };
        for (unsigned i = 0; i < 500 && !all_ready(); ++i) {
            const auto previous = arm.seq;
            arm = session.cycle(MC_AXIS_ARM, true);
            need(matches(arm, previous, false), "axis readiness wait failed; no MIT sent");
        }
        need(all_ready(), "not all ENCOS axes ready; no MIT sent");
    }
    show("arm", arm);
    uint32_t predecessor = arm.seq;
    unsigned sent_count = 0, verified = 0, missed = 0;
    const char *stop_reason = "complete";
    for (unsigned i = 0; i <= goal; ++i) {
        const bool command = i < goal;
        const auto frame = session.cycle(command ? start : 0, true);
        const bool matched = matches(frame, predecessor, i != 0);
        const uint32_t commanded = command && frame.sent ? frame.seq : 0U;
        rows.push_back({frame, commanded, predecessor, i != 0 && matched});
        if (commanded) ++sent_count;
        if (i != 0 && frame.received) {
            if (matched) ++verified;
            else ++missed;
        }
        if (!frame.sent) { stop_reason = "send_gate"; break; }
        if (!frame.ok) { stop_reason = "lrw_gate"; break; }
        if (!matched) { stop_reason = "next_lrw_mismatch"; break; }
        predecessor = frame.seq;
    }
    // The readback LRW may already have carried the next command; queue no more enabled PDOs.
    if (rows.back().frame.sent)
        for (unsigned i = 0; i < 4; ++i) session.cycle();
    else
        ecrt_master_deactivate_slaves(m.get());
    ecrt_master_deactivate(m.get());
    write_run(evidence.get(), rows);
    printf("planned=%u sent_pdo=%u next_lrw_ok=%u next_lrw_missed=%u unconfirmed=%u unexecuted_pdo=%u stop=%s\n",
           goal, sent_count, verified, missed, sent_count - verified - missed,
           goal - sent_count, stop_reason);
    show("first_scored", rows.front().frame);
    show("last_scored", rows.back().frame);
    return verified == goal ? 0 : 2;
} catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
    return 1;
}
