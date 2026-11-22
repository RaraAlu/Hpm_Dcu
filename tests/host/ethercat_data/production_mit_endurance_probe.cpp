#include <ecrt.h>
#include "encos_interface.h"
#include "igh_probe_frame_stats.hpp"
#include "probe_dc_clock.hpp"
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <sys/mman.h>
#include <time.h>
#include <vector>

namespace {
constexpr unsigned bus = 3, slot = 21, rx_size = 784, tx_size = 1072;
constexpr unsigned axis_rx_offset = 64 + slot * sizeof(mc_axis_rx_t);
constexpr unsigned axis_tx_offset = 64 + slot * sizeof(mc_axis_tx_t);
constexpr unsigned period_us = 2000, period_ns = period_us * 1000;
constexpr unsigned planned_frames = 30000, profile_revision = 5;
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
    mc_axis_tx_t axis{};
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

    Sample cycle(uint16_t op = 0, bool exact = false)
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
        if (op) {
            board.BoardControl = 1;
            board.ChannelEnableMask = 1U << bus;
            board.AxisEnableMask = 1U << slot;
            const mc_channel_rx_t channel{1, 1, 1, period_us};
            mc_axis_rx_t axis{};
            axis.Mode = 4;
            axis.ControlWord = op;
            std::memcpy(pdo + 32 + bus * sizeof(channel), &channel, sizeof(channel));
            std::memcpy(pdo + axis_rx_offset, &axis, sizeof(axis));
        }
        std::memcpy(pdo, &board, sizeof(board));
        ecrt_master_application_time(master, dc + now() - due);
        ecrt_master_sync_monitor_queue(master);
        ecrt_master_sync_slave_clocks(master);
        while (now() < due) {}
        if (op && now() - due > 45000) return result;
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
        result.ok = result.frames == 1 && result.bytes >= 1100 && result.bytes <= 1514 && ref && in_phase &&
            ds.wc_state == EC_WC_COMPLETE && result.wkc == 3 && sc.operational &&
            result.sent < due + 45000 && result.received < due + period_ns;
        std::memcpy(&result.board, pdo, sizeof(result.board));
        std::memcpy(&result.axis, pdo + axis_tx_offset, sizeof(result.axis));
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
           (unsigned long long)x.bytes, x.wkc, x.al, x.ok, x.board.BoardStatus, x.axis.StatusWord,
           x.axis.ValidFlags, x.axis.ActualPosition, x.axis.FeedbackAgeMs, x.sof, x.grid);
}

constexpr bool matches(const Sample &sample, uint32_t expected, bool expect_feedback)
{
    if (!sample.ok || sample.board.AcceptedCycleSeq != expected || (sample.board.BoardStatus & board_faults))
        return false;
    if (!expect_feedback) return !sample.axis.ValidFlags && !(sample.axis.StatusWord & fresh);
    return (sample.axis.StatusWord & fresh) && (sample.axis.ValidFlags & 7U) == 7U &&
        !sample.axis.MotorFaultRaw && !sample.axis.BridgeFaultCode && sample.axis.FeedbackAgeMs <= period_us / 1000;
}

struct RunRow {
    Sample frame;
    uint32_t commanded{}, expected{};
    bool verified{};
};

void write_run(FILE *file, const std::vector<RunRow> &rows)
{
    fputs("seq,commanded,expected,accepted,publish,due_ns,send_ns,receive_ns,frames,bytes,wkc,al,"
          "sof_ns,grid_ns,lrw_ok,next_lrw_ok,board,axis,valid,age_ms,position,velocity,current,motor_fault,bridge_fault\n",
          file);
    for (const auto &row : rows) {
        const auto &r = row.frame;
        fprintf(file, "%u,%u,%u,%u,%u,%llu,%llu,%llu,%llu,%llu,%u,%u,%u,%u,%d,%d,%u,%u,%u,%u,%d,%d,%d,%u,%u\n",
                r.seq, row.commanded, row.expected, r.board.AcceptedCycleSeq, r.board.PublishSeq,
                (unsigned long long)r.due, (unsigned long long)r.sent, (unsigned long long)r.received,
                (unsigned long long)r.frames, (unsigned long long)r.bytes, r.wkc, r.al, r.sof, r.grid,
                r.ok, row.verified, r.board.BoardStatus, r.axis.StatusWord, r.axis.ValidFlags,
                r.axis.FeedbackAgeMs, r.axis.ActualPosition, r.axis.ActualVelocity, r.axis.ActualCurrent,
                r.axis.MotorFaultRaw, r.axis.BridgeFaultCode);
    }
    need(fflush(file) == 0 && !ferror(file), "could not write endurance evidence");
}
}

int main(int argc, char **argv)
try {
    setvbuf(stdout, nullptr, _IOLBF, 0);
    const bool fire = argc == 3 && !std::strcmp(argv[1], "--mit-zero-60s");
    need(fire || (argc == 2 && !std::strcmp(argv[1], "--dry-run")),
         "explicit --dry-run or --mit-zero-60s CSV required");
    std::unique_ptr<FILE, decltype(&fclose)> evidence(fire ? fopen(argv[2], "wx") : nullptr, fclose);
    need(!fire || bool(evidence), "cannot create exclusive endurance CSV");
    std::vector<RunRow> rows;
    if (fire) rows.reserve(planned_frames + 1U);
    need(mlockall(MCL_CURRENT | MCL_FUTURE) == 0, "mlockall failed");
    one_cycle::FrameCounter counter;
    std::unique_ptr<ec_master_t, decltype(&ecrt_release_master)> m(ecrt_request_master(0), ecrt_release_master);
    need(bool(m), "cannot request IgH master");
    ec_master_info_t mi{};
    ec_slave_info_t si{};
    need(!ecrt_master(m.get(), &mi) && mi.link_up && !mi.scan_busy && mi.slave_count == 1, "not one isolated slave");
    need(!ecrt_master_get_slave(m.get(), 0, &si) && si.vendor_id == 0x10000 && si.product_code == 2 &&
         si.revision_number == 2 && si.al_state == 2, "production HPM in PREOP was not identified");
    need(upload(m.get(), 0x2100, 4, 4) == 1 && upload(m.get(), 0x2100, 5, 4) == profile_revision &&
         upload(m.get(), 0x2001, 1, 4) == (1U << slot) &&
         upload(m.get(), 0x3000 + 16 * slot, 1, 2) == 2, "unexpected production Profile or CAN route");
    auto *s = ecrt_master_slave_config(m.get(), 0, 0, 0x10000, 2);
    auto *d = ecrt_master_create_domain(m.get());
    need(s && d, "cannot create slave configuration and domain");
    ecrt_slave_config_overlapping_pdos(s, 1);
    unsigned out_bit = 255, in_bit = 255;
    auto out = ecrt_slave_config_reg_pdo_entry_pos(s, 2, 0, 0, d, &out_bit);
    auto in = ecrt_slave_config_reg_pdo_entry_pos(s, 3, 0, 0, d, &in_bit);
    need(out == 0 && in == 0 && !out_bit && !in_bit && ecrt_domain_size(d) == tx_size, "unexpected fixed PDO layout");
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
    Sample last{};
    for (unsigned i = 0; i < 5000 && good < 64; ++i) {
        last = session.cycle();
        good = matches(last, last.seq - 1, false) && session.follower.locked()
               ? good + 1 : 0;
    }
    show("warm", last);
    printf("warm_consecutive=%u\n", good);
    need(good == 64, "warmup did not pass; no MIT triggered");
    if (!fire) {
        ecrt_master_deactivate(m.get());
        return 0;
    }
    const auto arm = session.cycle(MC_AXIS_ARM, true);
    show("arm", arm);
    need(matches(arm, last.seq, false), "arm cycle did not pass; no MIT triggered");
    uint32_t predecessor = arm.seq;
    unsigned sent_count = 0, verified = 0, missed = 0;
    const char *stop_reason = "complete";
    for (unsigned i = 0; i <= planned_frames; ++i) {
        const bool command = i < planned_frames;
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
           planned_frames, sent_count, verified, missed, sent_count - verified - missed,
           planned_frames - sent_count, stop_reason);
    show("first_scored", rows.front().frame);
    show("last_scored", rows.back().frame);
    return verified == planned_frames ? 0 : 2;
} catch (const std::exception &e) {
    fprintf(stderr, "%s\n", e.what());
    return 1;
}
