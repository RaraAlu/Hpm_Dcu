"""Attribute an independent probe's SM2 call offset without treating previous SM3 time as wake latency."""
import argparse
from collections import Counter, defaultdict, deque
import csv
import json
from pathlib import Path
import re
from statistics import median

from summarize_pdo_endurance import distribution
from summarize_probe_clock import PERIOD, sm3_segments_us
from summarize_sm2_arrival import sof_lead

SHIFT, READ, SM3_PREP = 255_000, 870_000, 50_000


def sm2_segments(r, lead):
    due, entry, wake, prepared, queued, call, returned = (r[k] for k in (
        "due_ns", "sm2_entry_ns", "sm2_wake_ns", "sm2_prepared_ns", "sm2_queued_ns", "write_ns", "sm2_send_return_ns"))
    plan = due-lead
    if not (entry <= wake <= prepared <= queued <= call and wake >= plan and call >= due):
        raise ValueError("misordered SM2 stages or early send decision")
    if returned:
        if not (call < due+SHIFT and call <= returned <= r["sm3_wait_return_ns"]):
            raise ValueError("SM2 send outside its window")
    elif call < due+SHIFT or any(r[k] for k in ("read_ns", "sm3_send_return_ns", "feedback_ns", "on_time")):
        raise ValueError("unaccounted or flushed SM2")
    return dict(prior_work_past_plan=max(0, entry-plan), dispatch=wake-max(entry, plan),
                preparation=prepared-wake, queue=queued-prepared, ready_headroom=due-queued,
                issue_after_ready_or_target=call-max(queued, due), call_offset=call-due)


def summarize(base):
    log = base.with_suffix(".log").read_text()
    lead, = re.findall(r"^SM2_TIMING prepare_us=(\d+) send_phase_us=0$", log, re.M)
    lead = int(lead)*1000
    expected = int(re.search(r"^TIMING cpu=6 fifo=80 mlockall=1 samples=(\d+)", log, re.M)[1])
    start = int(re.search(r"^DC register 0x0990 (0x\w+)", log, re.M)[1], 16)
    if (lead not in (0, 50_000) or not re.search(r"^DC_MODE=hpm-reference .*valid=1 locked=1", log, re.M)
            or not re.search(r"^ONE_CYCLE Sync0_us=255 read_us=870 collect_us=975 publish_deadline_us=856 "
                             r"publish_advance_us=399 build_budget_us=600 sm3_prepare_us=50$", log, re.M)
            or int(re.search(r"^DC register 0x09a0 (0x\w+)", log, re.M)[1], 16) != PERIOD):
        raise ValueError("expected the original SM3 and HPM-follower diagnostic settings")
    hist, wkc, al = defaultdict(Counter), Counter(), Counter()
    n = good = refs = waits = over = not_sent = over_50 = at_least_old_peak = 0
    peak, peak_waited, previous, min_margin = None, None, None, None
    failures, high_examples, recent = [], [], deque(maxlen=32)
    with base.with_suffix(".csv").open(newline="") as f:
        for raw in csv.DictReader(f):
            r = {k: int(v) for k, v in raw.items()}
            due, end, seq = (r[k] for k in ("due_ns", "deadline_ns", "sent"))
            if previous and (seq != previous["sent"]+1 or r["dc_tick_ns"] != previous["dc_tick_ns"]+PERIOD or
                             due <= previous["due_ns"] or end <= due or
                             previous["deadline_ns"] != min(previous["due_ns"]+PERIOD, due) or
                             not previous["feedback_ns"] or r["sm2_entry_ns"] < previous["feedback_ns"]):
                raise ValueError(f"sequence, deadline or previous-feedback overlap at {seq}")
            if not due < end <= due + PERIOD:
                raise ValueError(f"invalid N+1 cutoff at {seq}")
            seg = sm2_segments(r, lead)
            sleeping = r["sm2_entry_ns"] < due-lead
            waits += sleeping
            sm2_sent, sm3_sent = bool(r["sm2_send_return_ns"]), bool(r["sm3_send_return_ns"])
            not_sent += not sm2_sent
            if sm2_sent:
                sm3_segments_us(r, READ, SM3_PREP)
            n += 1
            pair = (r["output_wkc"], r["input_wkc"])
            wkc[f"{pair[0] if sm2_sent else '?'}/{pair[1] if sm3_sent else '?'}"] += 1
            al[f"{r['al']}/{r['slave_op']}" if sm3_sent else "not sampled"] += 1
            passed = bool(sm2_sent and sm3_sent and pair == (1, 1) and r["slave_op"] and r["al"] == 8 and
                          seq == r["ack"] and due+READ <= r["read_ns"] <= r["feedback_ns"] <= end)
            if passed != bool(r["on_time"]):
                raise ValueError(f"N+1 verdict disagreement at {seq}")
            good += passed
            if not passed and len(failures) < 16:
                failures.append(dict(seq=seq, ack=r["ack"], wkc=list(pair), al=r["al"], sm2_sent=sm2_sent,
                                     sm3_sent=sm3_sent, cutoff_margin_us=(end-r["feedback_ns"])/1000 if sm3_sent else None))
            measures = dict(seg)
            measures["entry_headroom"] = due-r["sm2_entry_ns"]
            if sm2_sent:
                over += seg["call_offset"] > 1000
                over_50 += seg["call_offset"] >= 50_000
                at_least_old_peak += seg["call_offset"] >= 62_831
                measures["call_duration"] = r["sm2_send_return_ns"] - r["write_ns"]
                new_peak = peak is None or seg["call_offset"] > peak["offset_ns"]
                new_waited_peak = sleeping and (peak_waited is None or seg["call_offset"] > peak_waited["offset_ns"])
                exemplar = seg["call_offset"] >= 50_000 and len(high_examples) < 16
                if new_peak or new_waited_peak or exemplar:
                    detail = dict(seq=seq, offset_ns=seg["call_offset"], waited=sleeping, on_time=passed,
                                wkc=list(pair), ack=r["ack"],
                                segments_us={k: v/1000 for k, v in seg.items()},
                                prior_n=len(recent), prior_medians_us={k: median(p[k] for p in recent)/1000
                                                                      for k in seg} if recent else {},
                                previous_feedback_headroom_us=(due-previous["feedback_ns"])/1000 if previous else None,
                                previous_feedback_to_entry_us=(r["sm2_entry_ns"]-previous["feedback_ns"])/1000 if previous else None)
                    if new_peak: peak = detail
                    if new_waited_peak: peak_waited = detail
                    if exemplar: high_examples.append(detail)
            else:
                del measures["call_offset"]
            if r["servo_used"]:
                measures["sof_lead_to_sync0"] = sof_lead(dict(r, due_ns=r["dc_tick_ns"]), start, PERIOD, SHIFT)
                refs += 1
            if sm3_sent:
                margin = end-r["feedback_ns"]
                min_margin = margin if min_margin is None else min(min_margin, margin)
                measures["sm3_call_offset"] = r["read_ns"]-due-READ
            if sleeping:
                measures["wait_return_past_plan"] = seg["dispatch"]
            for k, v in measures.items():
                hist[k][v] += 1
            if passed: recent.append(seg)
            previous = r
    passed = (expected == n == good == refs and f"ONE_CYCLE_PASS commands={expected}" in log and "probe_exit= 0" in log)
    return dict(passed=passed, requested=expected, attempted=n, on_time=good, missed=n-good, unattempted=expected-n,
                sm2_not_sent=not_sent, prepare_us=lead//1000, wait_called=waits, joined_after_prewake=n-waits,
                over_1us=over, calls_at_least_50us=over_50, calls_at_least_62_831us=at_least_old_peak,
                first_high_calls=high_examples, valid_same_frame_reference=refs,
                wkc=dict(wkc), al_op=dict(al),
                min_feedback_margin_us=min_margin/1000 if min_margin is not None else None, failures=failures,
                worst=peak, worst_when_wait_called=peak_waited, metrics={k: distribution(c) for k,c in hist.items()})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base", type=Path, help="probe log/CSV base path")
    result = summarize(parser.parse_args().base)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
