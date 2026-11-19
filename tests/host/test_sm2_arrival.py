import csv
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent / "ethercat_data"))
from summarize_sm2_arrival import joined, ring, sof_lead
from summarize_probe_clock import capture_stats, sm3_comparison, sm3_segments_us, summarize
from summarize_pdo_endurance import kernel_window, scan
from summarize_sm2_send import sm2_segments
from summarize_collect_timing import segments as collect_segments


class ArrivalTest(unittest.TestCase):
    def setUp(self):
        self.due = 2**32 - 100_000
        self.start = self.due - 5_000_000 + 255_000
        self.row = dict(due_ns=self.due, sm2_frames=1, sm2_frame_bytes=844, output_wkc=1,
                        dc_sof_valid=1, dc_sof_ns=self.due + 50_000)

    def lead(self, **changes):
        return sof_lead({**self.row, **changes}, self.start, 1_000_000, 255_000)

    def test_dc_wrap_and_zero_are_valid(self):
        self.assertEqual(self.lead(), 205_000)
        self.assertEqual(self.lead(dc_sof_ns=0), 155_000)

    def test_collect_separates_late_entry_from_wait_and_checks_markers(self):
        row = dict(due_ns=0, sent=7, ack=7, input_wkc=1, output_wkc=1, on_time=0,
                   sm3_send_return_ns=872000, collect_wait_entry_ns=1001000, collect_wake_ns=1001300,
                   first_receive_begin_ns=1001400, first_receive_end_ns=1002400, first_domain_end_ns=1003000,
                   first_state_end_ns=1003700, last_state_end_ns=1003700, wkc_valid_ns=1003700,
                   ack_checked_ns=1003800, match_ns=1003800, op_checked_ns=1004600, feedback_ns=1004800,
                   receive_total_ns=1000, domain_total_ns=600, state_total_ns=700, collect_polls=1)
        costs = collect_segments(row)
        self.assertEqual((26000, 300, 26300), tuple(costs[k] for k in ("entry_late", "wait_overrun", "wait_return_late")))
        self.assertEqual(sum(costs[k] for k in ("wake_to_receive", "receive", "domain", "wkc_query",
                                               "retry_gaps", "ack_check", "op_check", "finalize")), costs["wake_to_verdict"])
        for change in (dict(ack=6), dict(wkc_valid_ns=0), dict(receive_total_ns=0), dict(op_checked_ns=1002000)):
            with self.subTest(change=change), self.assertRaises(ValueError):
                collect_segments(dict(row, **change))
        self.assertEqual(collect_segments(dict(row, ack=6, match_ns=0))["op_check"], 800)

    def test_sm3_only_attributes_measured_segments(self):
        row = {"due_ns": self.due, "read_ns": self.due + 906_000}
        self.assertEqual(sm3_segments_us(row, 870_000), {"plan_to_send_us": 36})
        for delay in (0, 9_000, 36_000):
            row["sm3_wait_return_ns"] = self.due + 870_000 + delay
            with self.subTest(delay=delay):
                self.assertEqual(sm3_segments_us(row, 870_000), dict(plan_to_send_us=36,
                                 plan_to_wait_return_us=delay/1000, wait_return_to_send_us=36-delay/1000))
        for stamp in (self.due+869_999, row["read_ns"]+1):
            with self.subTest(stamp=stamp), self.assertRaisesRegex(ValueError, "wait-return"):
                sm3_segments_us({**row, "sm3_wait_return_ns": stamp}, 870_000)

    def test_early_collect_does_not_count_spin_as_receive_or_accept_before_gate(self):
        row = dict(due_ns=0, sent=1, ack=1, input_wkc=1, output_wkc=1, on_time=1,
                   sm3_send_return_ns=872000, collect_wait_entry_ns=874000, collect_wake_ns=965000,
                   first_receive_begin_ns=975100, first_receive_end_ns=979100, first_domain_end_ns=979400,
                   first_state_end_ns=979700, last_state_end_ns=979700, wkc_valid_ns=979700,
                   ack_checked_ns=979800, match_ns=979800, op_checked_ns=980100, feedback_ns=980200,
                   receive_total_ns=4000, domain_total_ns=300, state_total_ns=300, collect_polls=1)
        costs = collect_segments(row, 50000)
        self.assertEqual((40000, 100, 5100), (costs["wait_return_late"], costs["gate_to_receive"], costs["receive_to_verdict"]))
        with self.assertRaisesRegex(ValueError, "before"):
            collect_segments(dict(row, first_receive_begin_ns=974999), 50000)
        with self.assertRaisesRegex(ValueError, "unordered"):
            collect_segments(row)

    def test_stale_or_unproven_frame_rejected(self):
        for change in ({"sm2_frames": 2}, {"sm2_frame_bytes": 80}, {"output_wkc": 0},
                       {"dc_sof_valid": 0}, {"dc_sof_ns": self.row["dc_sof_ns"]-1_000_000},
                       {"due_ns": self.due-1}):
            with self.subTest(change), self.assertRaises(ValueError):
                self.lead(**change)

    def test_early_preparation_is_not_an_early_send_or_extra_processing(self):
        due = self.due
        row = dict(due_ns=due, sent=7, deadline_ns=due+1_000_000, on_time=1, input_wkc=1,
                   sm3_wait_return_ns=due+825_000, sm3_ready_ns=due+830_000, read_ns=due+870_020,
                   sm3_send_return_ns=due+874_000, feedback_ns=due+982_000)
        seg = sm3_segments_us(row, 870_000, 50_000)
        self.assertEqual((seg["plan_to_wait_return_us"], seg["wait_return_to_ready_us"],
                          seg["ready_headroom_us"], seg["plan_to_send_us"]), (5, 5, 40, 0.020))
        late = dict(row, sm3_ready_ns=due+874_000, read_ns=due+874_080, sm3_send_return_ns=due+878_000)
        detail = sm3_comparison([row, late], 870_000, 50_000)
        self.assertEqual((detail["prep_overruns"], detail["send_late_over_1us"]), (1, 1))
        for invalid in (dict(read_ns=due+869_999), dict(sm3_wait_return_ns=due+819_999),
                        dict(sm3_ready_ns=due+871_000), dict(sm3_send_return_ns=due+869_999),
                        dict(sm3_send_return_ns=0)):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                sm3_segments_us(dict(row, **invalid), 870_000, 50_000)
        skipped = dict(row, read_ns=row["deadline_ns"], feedback_ns=row["deadline_ns"],
                       sm3_send_return_ns=0, on_time=0, input_wkc=0)
        self.assertNotIn("plan_to_send_us", sm3_segments_us(skipped, 870_000, 50_000))
        self.assertEqual(sm3_comparison([skipped], 870_000, 50_000)["not_issued"], 1)

    def test_sm2_separates_previous_work_wake_queue_and_send_gate(self):
        due = self.due
        pre = dict(due_ns=due, sm2_entry_ns=due-18000, sm2_wake_ns=due-17900,
                   sm2_prepared_ns=due-17000, sm2_queued_ns=due-15000, write_ns=due+100,
                   sm2_send_return_ns=due+2000, sm3_wait_return_ns=due+824000)
        self.assertEqual(sm2_segments(pre, 50_000), dict(prior_work_past_plan=32000, dispatch=100,
                         preparation=900, queue=2000, ready_headroom=15000, issue_after_ready_or_target=100, call_offset=100))
        old = dict(pre, sm2_wake_ns=due+57831, sm2_prepared_ns=due+60831, sm2_queued_ns=due+62831,
                   write_ns=due+62831, sm2_send_return_ns=due+64581)
        self.assertEqual((sm2_segments(old, 0)["dispatch"], sm2_segments(old, 0)["queue"]), (57831, 2000))
        for changed in (dict(write_ns=due-1000), dict(sm2_queued_ns=due+9000)):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                sm2_segments(dict(pre, **changed), 50_000)
        expired = dict(pre, write_ns=due+255_000, sm2_send_return_ns=0,
                       read_ns=0, sm3_send_return_ns=0, feedback_ns=0, on_time=0)
        sm2_segments(expired, 50_000)
        with self.assertRaisesRegex(ValueError, "flushed"):
            sm2_segments(dict(expired, sm3_send_return_ns=due+873_000), 50_000)

    def test_sm3_only_run_keeps_strict_deadline_without_a_sync0_capture(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            log = (f"DC register 0x0990 {self.start:#x}\nDC register 0x09a0 0xf4240\n"
                   "ONE_CYCLE Sync0_us=255 read_us=870\nDC_MODE=hpm-reference locked=1\n"
                   "ONE_CYCLE_PASS commands=32\nprobe_exit= 0\n")
            (d/"hpm-reference-short.log").write_text(log)
            rows = []
            for n in range(32):
                due = self.due + n * 1_000_000
                rows.append(dict(self.row, due_ns=due, dc_tick_ns=due, deadline_ns=due+1_000_000,
                                 dc_sof_ns=(due+35_000) % 2**32, sent=n+1, ack=n+1, input_wkc=1,
                                 al=8, servo_used=1, on_time=1, write_ns=due+3000, read_ns=due+879_000,
                                 sm3_wait_return_ns=due+874_000, feedback_ns=due+981_000))
            def save():
                with (d/"hpm-reference-short.csv").open("w", newline="") as f:
                    w = csv.DictWriter(f, fieldnames=rows[0])
                    w.writeheader()
                    w.writerows(rows)

            save()
            result = summarize(d, "hpm-reference", "short", sm3_only=True)
            self.assertTrue(result["passed"])
            self.assertIsNone(result["sync0"])
            with self.assertRaises(FileNotFoundError):
                summarize(d, "hpm-reference", "short")
            rows[-1].update(on_time=0, feedback_ns=rows[-1]["deadline_ns"]+1,
                            sm3_wait_return_ns=due+890_000, read_ns=due+896_000)
            save()
            result = summarize(d, "hpm-reference", "short", sm3_only=True)
            self.assertFalse(result["passed"])
            peak = result["sm3_comparison"]
            self.assertEqual((32, False, 31), (peak["peak_seq"], peak["peak_on_time"], peak["preceding_passed"]))
            self.assertEqual(peak["peak"]["plan_to_wait_return_us"], 20)
            self.assertEqual(peak["normal"]["plan_to_wait_return_us"]["maximum"], 4)
            self.assertEqual(peak["peak_preceding_median"]["wait_return_to_send_us"], 5)
            self.assertEqual(peak["slowest_per_mille"]["with_32_neighbors"], 0)
            self.assertEqual(peak["min_deadline_margin_us"], -0.001)
            self.assertEqual(sm3_comparison(rows[-1:], 870_000)["normal"], {})

    def test_sm3_tail_compares_added_delay_not_the_larger_raw_segment(self):
        row = dict(due_ns=0, deadline_ns=1_000_000, feedback_ns=980_000, on_time=1,
                   sm3_wait_return_ns=880_000, read_ns=884_000)
        rows = [dict(row, sent=n) for n in range(1, 34)]
        rows[-1]["read_ns"] += 3_000
        detail = sm3_comparison(rows, 870_000)
        self.assertEqual(detail["peak_seq"], 33)
        self.assertEqual(detail["slowest_per_mille"], dict(selected=1, with_32_neighbors=1,
                         greater_excess={"after_wait": 1}))

    def test_endurance_counts_late_and_invalid_without_bridging_missed_intervals(self):
        rows = []
        for i in range(6):
            due = self.due + i * 1_000_000
            rows.append(dict(self.row, due_ns=due, sent=i+1, ack=i+1, write_ns=due+4000,
                             sm3_wait_return_ns=due+824_000, sm3_ready_ns=due+831_000,
                             read_ns=due+870_100, sm3_send_return_ns=due+872_000, feedback_ns=due+982_000,
                             deadline_ns=due+1_000_000, dc_tick_ns=due, on_time=1, servo_used=1,
                             input_wkc=1, slave_op=1, al=8))
        rows[2].update(input_wkc=0, ack=2, on_time=0)
        rows[3].update(feedback_ns=rows[3]["deadline_ns"]+42, on_time=0)
        rows[4].update(ack=4, on_time=0)
        out = scan(rows)
        self.assertEqual((out["attempted"], out["missed"], out["valid_matching_readbacks"],
                          out["valid_wkc_wrong_echo"], out["al_and_op_flag"]), (6, 3, 4, 1, {"8/1": 6}))
        self.assertEqual(out["adjacent_pass_feedback_interval_abs_deviation"]["count"], 1)
        self.assertEqual(out["least_observation_margin"], {"seq": 4, "us": -0.042})
        rows[3]["on_time"] = 1
        with self.assertRaisesRegex(ValueError, "verdict"):
            scan(rows)

    def test_kernel_alignment_excludes_safeop_after_last_feedback(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            log = "KMSG_ANCHOR PRE 1000000000 1000000100\nKMSG_ANCHOR POST 11000000000 11000000100\n"
            (d/"kernel-live.log").write_text("[5.000000] run PRE rk_ns=1000000000\n"
                "[8.000000] EtherCAT ERROR 0-0: AL status message 0x001A: Synchronization error\n"
                "[8.001000] EtherCAT 0: Slave states on main device: SAFEOP.\n"
                "[13.000000] EtherCAT WARNING 0-0: Changed to SAFEOP + ERROR in the meantime.\n"
                "[15.000000] run POST rk_ns=11000000000\n")
            out = kernel_window(d, log, dict(first_due_ns=2_000_000_000, last_feedback_ns=8_000_000_000,
                       historical_dc_window=dict(first_due_ns=3_000_000_000, end_ns=5_000_000_000)))
            self.assertEqual((out["scored_al_001a"], out["scored_op_exit"], out["historical_window_ethercat_lines"],
                              out["after_scoring_ethercat_lines"]), (1, 1, 2, 1))

    def test_hardware_capture_needs_full_coverage_and_independent_edge_bounds(self):
        first = 2**32 - 1_000_000
        records = [100_000_000, 1_000_000, 33, 33, 32, 0, 0, 0, 99_900, 100_100,
                   first, (first+3_200_000) % 2**32, 1, 3_200_000]

        def stats(values, count=32):
            return capture_stats(struct.pack("<13I4xQ", *values), count)

        self.assertTrue(stats(records)["passed"])  # Inclusive +/- 1 us, with rollover.
        self.assertFalse(stats(records, 33)["passed"])
        for index, value in ((0, 0), (1, 999_000), (2, 34), (5, 1), (6, 1), (7, 1),
                             (8, 99_899), (9, 100_101), (12, 0), (13, 3_200_001)):
            changed = records.copy()
            changed[index] = value
            with self.subTest(index=index):
                self.assertFalse(stats(changed)["passed"])

    def test_ring_wrap_prefers_newest_occurrence(self):
        rows = [(1, 130), (1, 129)] + [(i, i) for i in range(2, 128)]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "ring"
            path.write_bytes(struct.pack("<I", 129) + b"".join(struct.pack("<2I", *r) for r in rows))
            self.assertEqual(ring(path, 2)[1], (1, 130))

    def test_miss_needs_its_own_stage_start_and_adjacent_epoch(self):
        with tempfile.TemporaryDirectory() as tmp:
            d = Path(tmp)
            (d / "preflight.txt").write_text("DC system time transmission delay: 0 ns\n")
            (d / "counter-long.log").write_text(
                f"ONE_CYCLE Sync0_us=255\nDC register 0x0990 {self.start:#x}\nDC register 0x09a0 0xf4240\n")
            rows = [dict(self.row, sent=10, ack=10, write_ns=self.due+3000, on_time=1)]
            rows.append(dict(rows[0], sent=11, ack=10, due_ns=self.due+1_000_000, on_time=0,
                             write_ns=self.due+1_003_000, dc_sof_ns=(self.row["dc_sof_ns"]+1_000_000) % 2**32))
            with (d / "counter-long.csv").open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=rows[0])
                w.writeheader()
                w.writerows(rows)
            for name, records, width in (
                ("rx", [[10, 870, 879, 992, 875, 996, 1], [11, 1874, 1883, 1996, 1879, 2005, 1]], 7),
                ("phase", [[10, 7, 992, 1000, 10, 1290, 1300, (self.due+555_000) % 2**32] + [0]*10], 18),
            ):
                data = b"".join(struct.pack(f"<{width}I", *r) for r in records)
                (d / f"long-{name}.bin").write_bytes(struct.pack("<I", len(records)) + data.ljust(128*4*width, b"\0"))
            diag = [0]*19
            diag[4], diag[12], diag[13], diag[15], diag[16] = 1, 8, 2000, 1, 1879
            path = d / "long-diag.bin"
            path.write_bytes(struct.pack("<19I", *diag))
            self.assertEqual(list(joined(d))[-1]["pdi_lead_us"], 126)
            for field in (12, 16):
                bad = diag.copy()
                bad[field] += 1
                path.write_bytes(struct.pack("<19I", *bad))
                with self.assertRaisesRegex(ValueError, "no proven MCU latch"):
                    list(joined(d))
