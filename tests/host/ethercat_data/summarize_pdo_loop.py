#!/usr/bin/env python3
"""Match probe RxPDO sequences to the firmware's TxPDO accepted sequences."""

import argparse
import csv
from collections import Counter
from heapq import nlargest


def report_us(label, samples):
    values = sorted(samples)
    if not values:
        print(f"{label}_us count=0")
        return
    n = len(values)
    ranks = (("p50", 1, 2), ("p99", 99, 100), ("p99.9", 999, 1000))
    tails = " ".join(f"{name}={values[(n * num - 1) // den] / 1000:.3f}"
                     for name, num, den in ranks)
    print(f"{label}_us count={n} min={values[0] / 1000:.3f} {tails} max={values[-1] / 1000:.3f}")


def receive_stages(row):
    stamps = (row["wake_ns"], row["receive_start_ns"], row["receive_end_ns"],
              row["domain_end_ns"], row["feedback_ns"])
    if any(b < a for a, b in zip(stamps, stamps[1:])):
        raise ValueError(f"unordered receive timestamps for ack {row['ack']}")
    return tuple(b - a for a, b in zip(stamps, stamps[1:]))


def report_switch_checks(rows, threshold_ns):
    slow = []
    for row in rows:
        elapsed = row["receive_end_ns"] - row["receive_start_ns"]
        voluntary, involuntary = row["voluntary_switches"], row["involuntary_switches"]
        if row["receive_start_ns"] <= 0 or elapsed < 0:
            raise ValueError("thread switch attribution requires valid receive timestamps")
        if elapsed >= threshold_ns:
            if min(voluntary, involuntary) < 0:
                raise ValueError("slow receive is missing thread switch snapshots")
            slow.append((elapsed, row["sent"], row["ack"], voluntary, involuntary))
        elif (voluntary, involuntary) != (-1, -1):
            raise ValueError("thread switch samples do not match the requested threshold")
    print(f"rx_switch_threshold_us={threshold_ns / 1000:.3f} checked={len(slow)} "
          f"no_thread_switch={sum(not (r[3] or r[4]) for r in slow)} "
          f"any_voluntary_switch={sum(r[3] > 0 for r in slow)} "
          f"any_involuntary_switch={sum(r[4] > 0 for r in slow)}")
    for elapsed, sent, ack, voluntary, involuntary in sorted(slow, reverse=True)[:3]:
        print(f"slow_receive_us={elapsed / 1000:.3f} sent={sent} ack={ack} "
              f"voluntary_switches={voluntary} involuntary_switches={involuntary}")


def report_interval_dominance(label, segments):
    counts = Counter()
    for _, _, wake, receive, *_ in segments:
        source = "wake" if abs(wake) > abs(receive) else "receive" if abs(receive) > abs(wake) else "tie"
        counts[source] += 1
    print(f"{label} count={counts.total()} wake={counts['wake']} receive={counts['receive']} tie={counts['tie']}")


def summarize(rows, cycles, period_ns, switch_threshold_ns=None):
    measured = rows[:cycles]
    scheduled = all(b["due_ns"] - a["due_ns"] == period_ns for a, b in zip(rows, rows[1:]))
    first = measured[0]["sent"]
    if not scheduled or any(row["sent"] != first + i for i, row in enumerate(rows)):
        raise ValueError("probe did not capture a continuous scheduled sequence")

    sends = {row["sent"]: row for row in measured}
    seen = set()
    completed, latencies = [], []
    lag = Counter()
    repeats = out_of_order = impossible = 0
    previous = 0
    for row in rows:  # Tail rows can complete measured commands, but aren't measured sends.
        seq, t = row["ack"], row["feedback_ns"]
        if not row["valid"] or seq not in sends:
            continue
        if seq >= row["sent"] or t <= sends[seq]["send_ns"]:
            impossible += 1
        elif seq in seen:
            repeats += 1  # Repeated observation of the same PDO image, not a second completion.
        else:
            out_of_order += seq < previous
            previous = seq
            seen.add(seq)
            completed.append(row)
            latencies.append(t - sends[seq]["send_ns"])
            lag[row["sent"] - seq] += 1

    pairs = list(zip(completed, completed[1:]))
    intervals = [b["feedback_ns"] - a["feedback_ns"] for a, b in pairs]
    errors = [dt - period_ns for dt in intervals]
    over = sum(abs(e) > 1000 for e in errors)
    missing = len(sends) - len(seen)
    bad = sum(row["al"] != 8 or row["wkc"] != 2 or not row["valid"] for row in measured)
    print(f"target={cycles} sent={len(sends)} completed={len(seen)} missing={missing} "
          f"unrecorded_sends={cycles - len(sends)} drain_cycles={len(rows) - len(measured)} "
          f"repeat_observations={repeats} out_of_order={out_of_order} impossible={impossible} bad_al_or_wkc={bad}")
    print(f"scheduled_s={cycles * period_ns / 1e9:.6f} "
          f"observed_send_window_s={(measured[-1]['send_ns'] - measured[0]['send_ns'] + period_ns) / 1e9:.6f}")
    report_us("completion_interval", intervals)
    report_us("completion_interval_error", errors)
    report_us("completion_interval_abs_error", [abs(e) for e in errors])
    report_us("send_to_completion", latencies)
    report_us("receive_path_elapsed", [row["feedback_ns"] - row["wake_ns"] for row in completed])
    # On consecutive receive ticks, the interval error is the sum of these first two deltas.
    segments = []
    for (a, b), err in zip(pairs, errors):
        if b["due_ns"] - a["due_ns"] != period_ns or b["ack"] != a["ack"] + 1:
            continue
        wake_delta = (b["wake_ns"] - a["wake_ns"]) - period_ns
        recv_delta = (b["feedback_ns"] - b["wake_ns"]) - (a["feedback_ns"] - a["wake_ns"])
        origin_a, origin_b = sends[a["ack"]], sends[b["ack"]]
        send_error = origin_b["send_ns"] - origin_a["send_ns"] - period_ns
        send_time_delta = (origin_b["send_return_ns"] - origin_b["send_ns"]) - (
            origin_a["send_return_ns"] - origin_a["send_ns"])
        segments.append((b["ack"], err, wake_delta, recv_delta, send_error, send_time_delta))
    report_us("continuous_completion_interval_abs_error", [abs(row[1]) for row in segments])
    for col, label in enumerate(("wake_phase_delta", "receive_path_delta", "origin_send_interval_error",
                                 "origin_send_duration_delta"), 2):
        report_us(label + "_abs", [abs(row[col]) for row in segments])
    report_interval_dominance("over_1us_continuous_interval_dominance", (s for s in segments if abs(s[1]) > 1000))
    top_count = (len(segments) + 999) // 1000
    top = nlargest(max(3, top_count), segments, key=lambda x: abs(x[1]))
    report_interval_dominance("top_0.1pct_continuous_interval_dominance", top[:top_count])
    worst = top[:3]
    split_stages = {row["sent"]: receive_stages(row) for row in rows} if any(
        row.get("receive_start_ns") for row in rows) else {}
    if split_stages:
        stages_by_ack = {row["ack"]: split_stages[row["sent"]] for row in completed}
        labels = ("pre_receive", "master_receive", "domain_process", "ack_read")
        for i, label in enumerate(labels):
            report_us(f"{label}_elapsed", [split_stages[row["sent"]][i] for row in measured])
            report_us(f"{label}_delta_abs", [abs(stages_by_ack[seq][i] - stages_by_ack[seq - 1][i])
                                             for seq, *_ in segments])
    for seq, err, wake_delta, recv_delta, send_error, _ in worst:
        line = (f"worst seq={seq} interval_error_us={err / 1000:+.3f} wake_delta_us={wake_delta / 1000:+.3f} "
                f"receive_delta_us={recv_delta / 1000:+.3f} origin_send_interval_error_us={send_error / 1000:+.3f}")
        if split_stages:
            line += " " + " ".join(f"{label}_delta_us={(b - a) / 1000:+.3f}"
                                    for label, a, b in zip(labels, stages_by_ack[seq - 1], stages_by_ack[seq]))
        print(line)
    if switch_threshold_ns is not None:
        report_switch_checks(measured, switch_threshold_ns)
    print("observed_ack_lag_cycles=" + ",".join(f"{key}:{lag[key]}" for key in sorted(lag)))
    passed = cycles > 1 and len(seen) == cycles and not any((over, out_of_order, impossible, bad))
    print(f"intervals_outside_1us={over}/{len(intervals)} all_within_1us={'yes' if passed else 'no'}")
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path")
    parser.add_argument("--cycles", type=int, required=True)
    parser.add_argument("--period-us", type=int, required=True)
    parser.add_argument("--rx-switch-threshold-us", type=int, metavar="US",
                        help="Check switch snapshots at the probe's slow receive threshold")
    parser.add_argument("--require-within-1us", action="store_true",
                        help="Return failure on a missed echo or an interval outside +/-1 us")
    args = parser.parse_args()
    with open(args.csv_path, newline="") as file:
        rows = [{k: int(v) for k, v in r.items()} for r in csv.DictReader(file)]
    if not rows or args.cycles < 2 or args.period_us <= 0:
        parser.error("expected a populated trace, at least two cycles and a positive period")
    if args.rx_switch_threshold_us is not None and args.rx_switch_threshold_us <= 0:
        parser.error("receive switch threshold must be positive")
    threshold_ns = None if args.rx_switch_threshold_us is None else args.rx_switch_threshold_us * 1000
    passed = summarize(rows, args.cycles, args.period_us * 1000, threshold_ns)
    return 1 if args.require_within_1us and not passed else 0


if __name__ == "__main__":
    raise SystemExit(main())
