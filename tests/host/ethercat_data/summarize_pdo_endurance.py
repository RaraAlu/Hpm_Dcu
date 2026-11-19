"""Stream the CPU6 prepare-50 HPM-follower trace; no board access or bulk row storage."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import re

from summarize_probe_clock import PERIOD, sm3_segments_us

SHIFT, READ, PREP = 255_000, 870_000, 50_000
SECOND = 1_000_000_000
FOCUS = (1780 * SECOND, 1790 * SECOND)


def distribution(counts):
    if not counts:
        return None
    n = sum(counts.values())
    out = {"count": n, "min_us": min(counts) / 1000, "max_us": max(counts) / 1000}
    seen = 0
    for ns, count in sorted(counts.items()):
        seen += count
        for label, rank in (("p50_us", (n + 1) // 2), ("p999_us", (999 * n + 999) // 1000)):
            if seen >= rank and label not in out:
                out[label] = ns / 1000
    return out


def scan(rows, focus_writer=None, misses_writer=None):
    sends, writes, intervals, wkc, al, reasons = (Counter() for _ in range(6))
    n = passed = references = valid_acks = early = over = overruns = stale = unknown_op = 0
    first = last = None
    min_margin = min_valid_margin = min_pass_margin = min_headroom = max_send = None
    window = {"rows": 0, "misses": 0, "wkc_1_1": 0, "op": 0, "reference": 0,
              "send_over_1us": 0, "min_margin_ns": None, "max_send_ns": None,
              "first_due_ns": None, "end_ns": None}
    prev_good = False
    for raw in rows:
        r = {k: int(v) for k, v in raw.items()}
        due, end, dc, seq = (r[k] for k in ("due_ns", "deadline_ns", "dc_tick_ns", "sent"))
        if not due < end <= due + PERIOD:
            raise ValueError(f"invalid cutoff at sequence {seq}")
        if first is None:
            first = r
        if last and not (seq == last["sent"] + 1 and dc == last["dc_tick_ns"] + PERIOD and
                         due > last["due_ns"] and last["deadline_ns"] == min(last["due_ns"] + PERIOD, due)):
            raise ValueError(f"sequence, DC grid or frozen next deadline discontinuity at {seq}")
        dt = dc - first["dc_tick_ns"]
        n += 1
        seg = sm3_segments_us(r, READ, PREP)
        issued = "plan_to_send_us" in seg
        sample_op = bool(issued and r["slave_op"] and r["al"] == 8)
        pair = (r["output_wkc"], r["input_wkc"])
        wkc[pair] += 1
        if issued:
            al[(r["al"], r["slave_op"])] += 1
        else:
            unknown_op += 1  # Early exit never called collect_feedback().
        echo = issued and pair[1] == 1 and seq == r["ack"]
        valid_acks += echo
        stale += bool(issued and pair[1] == 1 and not echo)
        ref = bool(r["servo_used"] and r["dc_sof_valid"] and r["sm2_frames"] == 1 and
                   828 <= r["sm2_frame_bytes"] <= 1514)
        references += ref
        faults = []
        if not issued: faults.append("SM3 not issued")
        if not sample_op and issued: faults.append("not OP")
        if pair[0] != 1: faults.append("SM2 WKC")
        if issued and pair[1] != 1: faults.append("SM3 WKC")
        if issued and pair[1] == 1 and not echo: faults.append("wrong echo")
        if not due <= r["write_ns"] < due + SHIFT: faults.append("SM2 time")
        writes[r["write_ns"] - due] += 1
        margin = end - r["feedback_ns"]
        if margin < 0: faults.append("feedback late")
        good = not faults
        if r["on_time"] != int(good):
            raise ValueError(f"offline N+1 verdict disagrees with probe at {seq}")
        passed += good
        reasons.update(faults)
        if faults and misses_writer:
            misses_writer.writerow(dict(raw, reason="; ".join(faults)))
        if min_margin is None or margin < min_margin[0]:
            min_margin = (margin, seq)
        if echo and (min_valid_margin is None or margin < min_valid_margin[0]):
            min_valid_margin = (margin, seq)
        if good and (min_pass_margin is None or margin < min_pass_margin[0]):
            min_pass_margin = (margin, seq)
        headroom = due + READ - r["sm3_ready_ns"]
        if min_headroom is None or headroom < min_headroom[0]:
            min_headroom = (headroom, seq)
        overruns += headroom < 0
        delta = r["read_ns"] - due - READ
        if issued:
            sends[delta] += 1
            early += delta < 0
            over += abs(delta) > 1000
            if max_send is None or delta > max_send[0]:
                max_send = (delta, seq)
        if good and prev_good:
            intervals[abs(r["feedback_ns"] - last["feedback_ns"] - PERIOD)] += 1
        if FOCUS[0] <= dt < FOCUS[1]:
            if window["first_due_ns"] is None:
                window["first_due_ns"] = due
            window["end_ns"] = end
            window["rows"] += 1
            window["misses"] += not good
            window["wkc_1_1"] += pair == (1, 1)
            window["op"] += sample_op
            window["reference"] += ref
            window["send_over_1us"] += issued and abs(delta) > 1000
            if window["min_margin_ns"] is None or margin < window["min_margin_ns"]:
                window["min_margin_ns"] = margin
            if issued and (window["max_send_ns"] is None or delta > window["max_send_ns"]):
                window["max_send_ns"] = delta
            if focus_writer and 1785_130_000_000 <= dt <= 1785_170_000_000:
                focus_writer.writerow(dict(raw, dc_elapsed_ns=dt, rk_elapsed_ns=due-first["due_ns"]))
        last, prev_good = r, good
    if first is None:
        raise ValueError("no scored rows")
    def extreme(entry):
        return {"us": entry[0] / 1000, "seq": entry[1]} if entry else None
    return dict(attempted=n, passed_n_plus_one=passed, missed=n-passed, valid_matching_readbacks=valid_acks,
                valid_wkc_wrong_echo=stale, failure_reasons=dict(reasons),
                wkc={f"{a}/{b}": v for (a, b), v in wkc.items()},
                al_and_op_flag={f"{a}/{b}": v for (a, b), v in al.items()}, al_unobserved=unknown_op,
                same_frame_reference=references, send_call_deviation=distribution(sends),
                sm2_call_offset=distribution(writes), least_passing_margin=extreme(min_pass_margin),
                issued_sm3=sum(sends.values()), early_sends=early, abs_send_over_1us=over, worst_send=extreme(max_send),
                prep_overruns=overruns, least_ready_headroom=extreme(min_headroom),
                least_observation_margin=extreme(min_margin), least_matching_echo_margin=extreme(min_valid_margin),
                adjacent_pass_feedback_interval_abs_deviation=distribution(intervals),
                adjacent_pass_interval_over_1us=sum(v for k, v in intervals.items() if k > 1000),
                first_sequence=first["sent"], last_sequence=last["sent"], first_due_ns=first["due_ns"],
                last_deadline_ns=last["deadline_ns"], last_feedback_ns=last["feedback_ns"],
                rk_window_s=(last["deadline_ns"]-first["due_ns"])/SECOND,
                historical_dc_window=window)


def kernel_window(directory, log, trace):
    # printk and CLOCK_MONOTONIC need not have the same origin/rate. Bracket both ends.
    lines = (directory / "kernel-live.log").read_text(errors="replace").splitlines()
    def timestamp(line):
        match = re.match(r"^\[\s*(\d+)\.(\d+)\]", line)
        if match:
            return int(match[1]) * SECOND + int(match[2].ljust(9, "0")[:9])
        return None
    anchors = []
    for label in ("PRE", "POST"):
        before, after = (int(n) for n in re.search(rf"^KMSG_ANCHOR {label} (\d+) (\d+)$", log, re.M).groups())
        matches = [timestamp(line) for line in lines if f" {label} rk_ns={before}" in line]
        if len(matches) != 1 or matches[0] is None or before > after:
            raise ValueError(f"missing or ambiguous {label} kernel-log anchor")
        anchors.append((matches[0], (before+after)//2, after-before))
    (ka, a, ba), (kb, b, bb) = anchors
    if not (ka < kb and a <= trace["first_due_ns"] <= trace["last_feedback_ns"] <= b):
        raise ValueError("kernel log does not bracket the scored rows")
    historical = trace["historical_dc_window"]
    start = trace["first_due_ns"]
    focus_lines, active, post = [], [], []
    counts = Counter()
    for line in lines:
        ns = timestamp(line)
        if ns is None or "EtherCAT" not in line:
            continue
        rk = a + (ns-ka)*(b-a)//(kb-ka)
        text = f"{(rk-start)/SECOND:+.6f} s  {line}"
        if historical["first_due_ns"] is not None and historical["first_due_ns"] <= rk < historical["end_ns"]:
            focus_lines.append(text)
        if rk < start:
            continue
        if rk > trace["last_feedback_ns"]:
            post.append(text)
            continue
        active.append(text)
        if re.search(r"AL status (?:message|code).*0x001a|0x001a.*[Ss]ynchron", line, re.I):
            counts["al_001a"] += 1
        if re.search(r"Slave states .*\b(?:SAFEOP|PREOP|INIT)\b|Changed to SAFEOP", line):
            counts["op_exit"] += 1
        if re.search(r"Link state .*DOWN|link .*down", line, re.I):
            counts["link_down"] += 1
    for name, items in (("kernel-historical-window.log", focus_lines), ("kernel-scored.log", active),
                        ("kernel-after-scoring.log", post)):
        (directory / name).write_text("".join(line + "\n" for line in items))
    return dict(anchor_bracket_ns=[ba, bb], anchored_span_s=(b-a)/SECOND, scored_ethercat_lines=len(active),
                historical_window_ethercat_lines=len(focus_lines), after_scoring_ethercat_lines=len(post),
                scored_al_001a=counts["al_001a"], scored_op_exit=counts["op_exit"], scored_link_down=counts["link_down"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    d = args.directory
    base = d / "hpm-reference-endurance"
    log = base.with_suffix(".log").read_text()
    if not all(re.search(pattern, log, re.M) for pattern in
               (r"^TIMING cpu=6 fifo=80 mlockall=1 samples=1800000\b",
                r"^ONE_CYCLE Sync0_us=255 read_us=870 .*build_budget_us=600 sm3_prepare_us=50$",
                r"^DC_MODE=hpm-reference .*valid=1 locked=1\b")):
        raise SystemExit("wrong probe or follower did not lock")
    with (base.with_suffix(".csv").open(newline="") as src,
          (d / "historical-window.csv").open("w", newline="") as focused,
          (d / "misses.csv").open("w", newline="") as missed):
        reader = csv.DictReader(src)
        fw = csv.DictWriter(focused, reader.fieldnames + ["dc_elapsed_ns", "rk_elapsed_ns"])
        mw = csv.DictWriter(missed, reader.fieldnames + ["reason"])
        fw.writeheader()
        mw.writeheader()
        out = scan(reader, fw, mw)
    target = 1_800_000
    out.update(requested=target, not_attempted=target-out["attempted"])
    out["kernel"] = kernel_window(d, log, out)
    out["passed"] = (out["passed_n_plus_one"] == out["same_frame_reference"] == out["attempted"] == target and
                     not any(out["kernel"][field] for field in ("scored_al_001a", "scored_op_exit", "scored_link_down")) and
                     f"ONE_CYCLE_PASS commands={target}" in log and "probe_exit= 0" in log)
    print(json.dumps(out, indent=2))
    raise SystemExit(0 if out["passed"] else 1)
