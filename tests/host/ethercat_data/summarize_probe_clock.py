"""Gate one CPU6 A/B session against its own host trace, ESC grid and HPM capture."""
import argparse
from collections import Counter, deque
import csv
import json
from pathlib import Path
import re
from statistics import median
import struct

from summarize_sm2_arrival import sof_lead

PERIOD = 1_000_000


def sm3_segments_us(row, read_phase_ns, prepare_ns=0):
    planned = row["due_ns"] + read_phase_ns
    send = row["read_ns"]  # Userspace timestamp at the decision to issue the SM3 read.
    if prepare_ns:
        wake, ready, returned = (row[k] for k in ("sm3_wait_return_ns", "sm3_ready_ns", "sm3_send_return_ns"))
        if not planned-prepare_ns <= wake <= ready <= send or send < planned:
            raise ValueError("SM3 prepared or issued outside its timing window")
        segments = dict(plan_to_wait_return_us=(wake-(planned-prepare_ns))/1000,
                        wait_return_to_ready_us=(ready-wake)/1000, ready_headroom_us=(planned-ready)/1000)
        if not returned:
            if send < row["deadline_ns"] or row["on_time"] or row["input_wkc"] or row["feedback_ns"] != send:
                raise ValueError("missing SM3 send before cutoff")
            return segments
        if not send < row["deadline_ns"] or not send <= returned <= row["feedback_ns"]:
            raise ValueError("unexpected SM3 send-return timestamp")
        return dict(segments, plan_to_send_us=(send-planned)/1000, ready_to_send_us=(send-ready)/1000,
                    send_call_us=(returned-send)/1000)
    segments = {"plan_to_send_us": (send - planned) / 1000}
    if "sm3_wait_return_ns" in row:
        wake = row["sm3_wait_return_ns"]
        if not planned <= wake <= send:
            raise ValueError("SM3 wait-return timestamp is outside the plan-to-send interval")
        segments.update(plan_to_wait_return_us=(wake-planned)/1000, wait_return_to_send_us=(send-wake)/1000)
    return segments


def capture_stats(data, attempted):
    hz, period, events, edges, n, missing, coalesced, outside, lo, hi, first, last, wraps, total = (
        struct.unpack("<13I4xQ", data))
    complete = (hz > 0 and period == PERIOD and events == edges == n + 1 and n >= attempted > 0
                and not missing and not coalesced and total == last + (wraps << 32) - first
                and 0 < lo <= hi and lo * n <= total <= hi * n)
    return dict(clock_hz=hz, events=events, edges=edges, intervals=n, missing=missing,
                coalesced=coalesced, outside_1us=outside, wraps=wraps,
                min_us=lo * 1e6 / hz if hz else None, max_us=hi * 1e6 / hz if hz else None,
                mean_us=total * 1e6 / (hz*n) if hz and n else None,
                passed=bool(complete and not outside and lo * 10**9 >= hz * (period-1000)
                            and hi * 10**9 <= hz * (period+1000)))


def sm3_comparison(rows, read_phase_ns, prepare_ns=0):
    split = [sm3_segments_us(row, read_phase_ns, prepare_ns) for row in rows]
    good = [s for row, s in zip(rows, split) if row["on_time"]]
    stats = {}
    if good:
        for key in good[0]:
            values = sorted(s[key] for s in good)
            stats[key] = dict(p50=median(values), p999=values[(999*len(values)-1)//1000], maximum=values[-1])
    def before(i):
        return [split[j] for j in range(max(0, i-32), i) if rows[j]["on_time"]]

    issued = [i for i, s in enumerate(split) if "plan_to_send_us" in s]
    if not issued:
        return dict(passed_samples=0, normal={}, not_issued=len(rows))
    ranked = sorted(issued, key=lambda i: split[i]["plan_to_send_us"], reverse=True)
    peak, tail = ranked[0], ranked[:(len(issued)+999)//1000]
    previous = before(peak)
    dominance = Counter()
    for i in tail:
        if prepare_ns:
            # Intentional spinning is not waking or processing latency.
            dominance["ready_by_target" if split[i]["ready_headroom_us"] >= 0 else "ready_late"] += 1
            continue
        nearby = before(i)
        if len(nearby) != 32:
            continue
        # Compare integer-nanosecond segment excess, not the raw magnitudes.
        wake, work = (round(1000*split[i][k]) - median(round(1000*s[k]) for s in nearby)
                      for k in ("plan_to_wait_return_us", "wait_return_to_send_us"))
        dominance["wait" if wake > work else "after_wait" if wake < work else "tie"] += 1
    margins = [row["deadline_ns"] - row["feedback_ns"] for row in rows]
    least = min(range(len(rows)), key=margins.__getitem__)
    tail_detail = (dict(selected=len(tail), readiness=dict(dominance)) if prepare_ns else
                   dict(selected=len(tail), with_32_neighbors=sum(dominance.values()), greater_excess=dict(dominance)))
    result = dict(passed_samples=len(good), normal=stats, peak_seq=rows[peak]["sent"],
                peak_on_time=bool(rows[peak]["on_time"]), peak=split[peak], preceding_passed=len(previous),
                peak_preceding_median={k: median(s[k] for s in previous) for k in split[peak]} if previous else {},
                slowest_per_mille=tail_detail,
                min_deadline_margin_us=margins[least]/1000, min_margin_seq=rows[least]["sent"])
    result.update(not_issued=len(rows)-len(issued),
                  send_late_over_1us=sum(split[i]["plan_to_send_us"] > 1 for i in issued))
    if prepare_ns:
        result.update(prep_overruns=sum(s["ready_headroom_us"] < 0 for s in split),
                      min_ready_headroom_us=min(s["ready_headroom_us"] for s in split))
    return result


def summarize(directory, mode, stage, *, sm3_only=False):
    base = directory / f"{mode}-{stage}"
    expected = 32 if stage == "short" else 60000
    log = base.with_suffix(".log").read_text()
    def register(addr):
        return int(re.search(rf"^DC register {addr} (0x[0-9a-fA-F]+)", log, re.M)[1], 16)
    start, period = register("0x0990"), register("0x09a0")
    phases = re.search(r"^ONE_CYCLE Sync0_us=(\d+) read_us=(\d+)", log, re.M)
    if not phases:
        raise ValueError("missing probe phases")
    shift, read_phase_ns = (int(v) * 1000 for v in phases.groups())
    if (period, shift, read_phase_ns) != (PERIOD, 255_000, 870_000):
        raise ValueError("unexpected ESC period, Sync0 phase or SM3 read phase")
    with base.with_suffix(".csv").open(newline="") as f:
        rows = [{k: int(v) for k, v in row.items()} for row in csv.DictReader(f)]
    if not 0 < len(rows) <= expected:
        raise ValueError("missing or extra scored rows")
    prepare = re.search(r"^ONE_CYCLE [^\n]*\bsm3_prepare_us=(\d+)\b", log, re.M)
    prepare_ns = int(prepare[1])*1000 if prepare else 0
    cols = {"sm3_ready_ns", "sm3_send_return_ns"}
    if prepare_ns > 50_000 or cols.intersection(rows[0]) != (cols if prepare_ns else set()):
        raise ValueError("SM3 preparation setting does not match the trace")
    if sm3_only and (mode != "hpm-reference" or "sm3_wait_return_ns" not in rows[0]
                     or not re.search(r"^DC_MODE=hpm-reference\b.*\blocked=1\b", log, re.M)):
        raise ValueError("SM3-only verification requires a locked HPM follower and a wait-return timestamp")
    leads, failed = [], []
    recent_sm3 = deque(maxlen=32)
    for i, row in enumerate(rows):
        due, dc, end = row["due_ns"], row["dc_tick_ns"], row["deadline_ns"]
        if i and (row["sent"] != rows[i-1]["sent"] + 1 or dc != rows[i-1]["dc_tick_ns"] + PERIOD):
            raise ValueError("discontinuous DC cycle or command")
        if not due < end <= due + PERIOD:
            raise ValueError("invalid one-cycle cutoff")
        if i+1 < len(rows) and end != min(due + PERIOD, rows[i+1]["due_ns"]):
            raise ValueError("deadline differs from prebooked next wake")
        if mode == "rk-reference" and due != dc:
            raise ValueError("legacy host grid was adjusted")
        reasons = []
        if row["input_wkc"] == 1 and row["sent"] != row["ack"]:
            reasons.append("ack")
        if row["output_wkc"] != 1:
            reasons.append("SM2 WKC")
        if row["input_wkc"] != 1:
            reasons.append("SM3 WKC")
        if row["al"] != 8:
            reasons.append("not OP")
        if not (due <= row["write_ns"] < due+shift and due+read_phase_ns <= row["read_ns"] <= row["feedback_ns"] <= end):
            reasons.append("time")
        if not row["on_time"] and not reasons:
            reasons.append("not operational")
        if row["on_time"] != int(not reasons):
            raise ValueError("probe and offline N+1 verdict disagree")
        sm3 = sm3_segments_us(row, read_phase_ns, prepare_ns)
        if reasons:
            failed.append(dict(seq=row["sent"], ack_image=row["ack"], ack_valid=row["input_wkc"] == 1,
                               reason=reasons, wkc=[row["output_wkc"], row["input_wkc"]], al=row["al"],
                               sm3_send_offset_us=(row["read_ns"]-due)/1000 if "plan_to_send_us" in sm3 else None,
                               sm3=sm3, preceding_passed=len(recent_sm3),
                               sm3_preceding_median={k: median(x[k] for x in recent_sm3) for k in sm3}
                               if recent_sm3 else {},
                               feedback_offset_us=(row["feedback_ns"]-due)/1000, cutoff_us=(end-due)/1000))
        else:
            recent_sm3.append(sm3)
        # Grid congruence must use the logical DC cycle, not the steered RK wake.
        try:
            leads.append(sof_lead({**row, "due_ns": dc}, start, period, shift))
        except ValueError as err:
            if "no same-frame proof" not in str(err):
                raise
            leads.append(None)
    valid = [v for v in leads if v is not None]
    width = 8 if stage == "short" else 32
    blocks = [min(leads[i:i+width])/1000 for i in range(0, len(leads)-width+1, width)
              if all(v is not None for v in leads[i:i+width])]
    group_min = lambda vals: min(vals)/1000 if vals and all(v is not None for v in vals) else None
    capture = None if sm3_only else capture_stats(Path(str(base)+"-capture.bin").read_bytes(), len(rows))
    clock_samples = sum(row["servo_used"] for row in rows)
    passed = (len(rows) == expected and not failed and len(valid) == expected and (sm3_only or capture["passed"])
              and clock_samples == (expected if mode == "hpm-reference" else 0)
              and f"ONE_CYCLE_PASS commands={expected}" in log and "probe_exit= 0" in log)
    result = dict(mode=mode, requested=expected, attempted=len(rows), missed=len(failed),
                not_attempted=expected-len(rows), failures=failed, same_frame_sofs=len(valid),
                sm2_l2_bytes=dict(Counter(row["sm2_frame_bytes"] for row in rows)), servo_used=clock_samples,
                lead_min_us=min(valid)/1000 if valid else None, lead_window=width,
                lead_first_window_min_us=group_min(leads[:width]), lead_last_window_min_us=group_min(leads[-width:]),
                full_windows=len(blocks), falling_adjacent_windows=sum(b<a for a,b in zip(blocks,blocks[1:])),
                lead_block_min_range_us=[min(blocks), max(blocks)] if blocks else [],
                last_lead_us=leads[-1]/1000 if leads[-1] is not None else None,
                sync0=capture, passed=passed)
    if sm3_only:
        result.update(scope="SM3; no Sync0 capture", sm3_prepare_us=prepare_ns//1000,
                      sm3_comparison=sm3_comparison(rows, read_phase_ns, prepare_ns))
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory", type=Path)
    p.add_argument("mode", choices=["rk-reference", "hpm-reference"])
    p.add_argument("stage", choices=["short", "long"])
    p.add_argument("--sm3-only", action="store_true", help="compare host SM3 timing without a hardware Sync0 capture")
    args = p.parse_args()
    result = summarize(args.directory, args.mode, args.stage, sm3_only=args.sm3_only)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
