"""Audit the +975 us collect trace offline. Run with a paired CSV/log path prefix."""
import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import re

from summarize_pdo_endurance import distribution, scan


def segments(r, advance_ns=0):
    target = r["due_ns"] + 975_000
    wake_target = target - advance_ns
    entry, wake = r["collect_wait_entry_ns"], r["collect_wake_ns"]
    begin, rx, domain = (r[k] for k in ("first_receive_begin_ns", "first_receive_end_ns", "first_domain_end_ns"))
    first, last, ack, op, finish = (r[k] for k in (
        "first_state_end_ns", "last_state_end_ns", "ack_checked_ns", "op_checked_ns", "feedback_ns"))
    stamps = (entry, wake, begin, rx, domain, first, last, ack, op, finish)
    totals = (r["receive_total_ns"], r["domain_total_ns"], r["state_total_ns"])
    if not (r["sm3_send_return_ns"] <= entry and wake >= wake_target and list(stamps) == sorted(stamps)):
        raise ValueError(f"unordered collect stamps at {r['sent']}")
    if begin < target:
        raise ValueError(f"receive before +975 us at {r['sent']}")
    if (r["collect_polls"] == 1 and (totals != (rx-begin, domain-rx, first-domain) or first != last)
            or any(total < first_cost for total, first_cost in zip(totals, (rx-begin, domain-rx, first-domain)))
            or sum(totals) > last-begin):
        raise ValueError(f"inconsistent poll totals at {r['sent']}")
    wkc, match = r["wkc_valid_ns"], r["match_ns"]
    if wkc and (wkc != last or r["input_wkc"] != 1):
        raise ValueError(f"invalid WKC stamp at {r['sent']}")
    expected_match = bool(wkc and r["output_wkc"] == 1 and r["sent"] == r["ack"])
    if match != (ack if expected_match else 0) or (r["on_time"] and not match):
        raise ValueError(f"invalid echo stamp at {r['sent']}")
    return dict(entry_late=max(0, entry-wake_target), wait_return_late=wake-wake_target,
                wait_overrun=wake-max(entry, wake_target), wake_to_receive=begin-wake,
                gate_to_receive=begin-target, receive_to_verdict=finish-begin,
                receive=totals[0], domain=totals[1], wkc_query=totals[2],
                retry_gaps=last-begin-sum(totals), ack_check=ack-last, op_check=op-ack,
                finalize=finish-op, wake_to_verdict=finish-wake, target_to_verdict=finish-target)


def summarize(prefix):
    log = prefix.with_suffix(".log").read_text()
    mode = re.search(r"^COLLECT_TIMING wait=(legacy|absolute|early) clock=monotonic$", log, re.M)
    samples = re.search(r"^TIMING cpu=6 fifo=80 mlockall=1 samples=(\d+)\b", log, re.M)
    if not (mode and samples and re.search(r"^DC_MODE=hpm-reference .*valid=1 locked=1\b", log, re.M)
            and "ONE_CYCLE Sync0_us=255 read_us=870 collect_us=975 publish_deadline_us=856 "
                "publish_advance_us=399 build_budget_us=600 sm3_prepare_us=50" in log
            and "SM2_TIMING prepare_us=50 send_phase_us=0" in log):
        raise ValueError("expected the locked CPU6 HPM follower and unchanged phase/budget")
    early = mode[1] == "early"
    if early != ("COLLECT_PREWAKE wake_us=925 gate_us=975" in log.splitlines()):
        raise ValueError("wake plan disagrees with collect mode")
    hist, polls, misses = defaultdict(Counter), Counter(), []
    wake_past_gate = 0
    worst = None

    def measured(reader):
        nonlocal worst, wake_past_gate
        for row in reader:
            r = {k: int(v) for k, v in row.items()}
            polls[r["collect_polls"]] += 1
            if r["collect_polls"]:
                costs = segments(r, 50_000 if early else 0)
                wake_past_gate += r["collect_wake_ns"] > r["due_ns"] + 975_000
                for k, v in costs.items():
                    hist[k][v] += 1
                event = dict(seq=r["sent"], ack=r["ack"], margin_us=(r["deadline_ns"]-r["feedback_ns"])/1000,
                             segments_us={k: v/1000 for k, v in costs.items()})
                if worst is None or event["margin_us"] < worst["margin_us"]:
                    worst = event
                if not r["on_time"] and len(misses) < 10:
                    misses.append(event)
            elif r["on_time"] or r["sm3_send_return_ns"]:
                raise ValueError(f"missing collect after SM3 at {r['sent']}")
            yield r

    with prefix.with_suffix(".csv").open(newline="") as f:
        result = scan(measured(csv.DictReader(f)))
    n = int(samples[1])
    passed = (result["attempted"] == result["passed_n_plus_one"] == result["same_frame_reference"] == n
              and f"ONE_CYCLE_PASS commands={n}\n" in log and "probe_exit= 0" in log)
    return dict(wait=mode[1], requested=n, passed=passed, n_plus_one=result, polls=dict(polls),
                wake_past_receive_gate=wake_past_gate,
                segments_us={k: distribution(v) for k, v in hist.items()},
                least_margin_collect=worst, first_misses=misses)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("prefix", type=Path)
    result = summarize(parser.parse_args().prefix)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
