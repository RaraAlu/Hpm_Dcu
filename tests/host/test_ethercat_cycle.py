import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class EthercatCycleTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise RuntimeError("EtherCAT cycle tests require a host C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-ethercat-cycle-")
        cls.addClassCleanup(directory.cleanup)
        cls.executable = Path(directory.name) / "ethercat_cycle"
        cls.trace_executable = Path(directory.name) / "ethercat_cycle_trace"
        flags = ["-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        for include in (
            "tests/host/cycle/stubs", "firmware/app/cycle", "firmware/app/motor",
            "firmware/app/runtime",
            "firmware/comm/canfd", "firmware/comm/ethercat", "firmware/cfg", "firmware/ports",
            "firmware/generated/encos", "firmware/bsp/dcu_hpm5e31",
        ):
            flags.extend(("-I", str(ROOT / include)))
        sources = [
            str(ROOT / "tests/host/cycle/ethercat_cycle_test.c"),
            str(ROOT / "firmware/app/cycle/pdo_pipeline.c"),
            str(ROOT / "firmware/comm/canfd/can_tx_gate.c"),
        ]
        cls.run_command([cc, *flags, *sources, "-o", str(cls.executable)])
        cls.run_command([cc, *flags, "-DCONFIG_PDO_PHASE_TRACE=1", *sources, "-o", str(cls.trace_executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_dc_mode_period_and_safeop_to_op_validation(self):
        self.run_command([str(self.executable), "configuration"])

    def test_single_fd_preserves_queued_frames_without_a_sync0_or_per_cycle_deadline(self):
        self.run_command([str(self.executable), "raw_events"])

    def test_single_fd_inhibits_can_if_the_worker_stalls(self):
        self.run_command([str(self.executable), "raw_stalled"])

    def test_accepts_only_the_supported_sm2_sync_pair_and_no_dc_pulses(self):
        self.run_command([str(self.executable), "sm2_config"])

    def test_sm2_commits_one_complete_image_without_sync0_and_publishes_without_a_reply(self):
        self.run_command([str(self.executable), "sm2_exchange"])

    def test_sm2_uses_arrival_for_tx_deadline_and_has_a_worker_independent_rx_watchdog(self):
        for scenario in ("sm2_deadline", "sm2_missing"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.executable), scenario])

    def test_isrs_only_handoff_and_task_publishes_one_cycle(self):
        self.run_command([str(self.executable), "normal"])

    def test_publish_waits_for_current_command_without_extending_deadline(self):
        for scenario in ("process_before_publish", "process_deadline"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.executable), scenario])

    def test_packing_and_esc_write_must_finish_before_deadline(self):
        self.run_command([str(self.executable), "prepare_late"])

    def test_already_late_packing_never_begins_an_esc_write(self):
        self.run_command([str(self.executable), "pack_already_late"])

    def test_cyclic_feedback_is_complete_before_pack_and_write(self):
        for scenario in ("cyclic_complete", "cyclic_missing", "cyclic_write_late"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.executable), scenario])

    def test_phase_trace_retains_failed_publish_stages_and_neutral_wait(self):
        for scenario in ("neutral_wait", "neutral_wait_late", "prepare_late", "pack_already_late", "sm2_exchange"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.trace_executable), scenario])

    def test_sync_during_esc_write_cannot_acknowledge_new_epoch(self):
        self.run_command([str(self.executable), "crossed_epoch"])

    def test_esc_word_copy_keeps_exact_pdo_bytes_and_tail_bounds(self):
        self.run_command([str(self.executable), "esc_pdo_copy"])

    def test_sm2_word_read_keeps_all_profiles_byte_exact(self):
        self.run_command([str(self.executable), "esc_sm2_word_read"])

    def test_sm2_reads_directly_into_an_unpublished_slot(self):
        self.run_command([str(self.executable), "esc_sm2_direct_stage"])

    def test_missing_frame_stops_without_replaying_old_command(self):
        self.run_command([str(self.executable), "missing"])

    def test_completed_preop_rearms_communication_not_old_commands(self):
        self.run_command([str(self.executable), "preop_recovery"])

    def test_safeop_ack_clears_only_own_error_outside_pending_transition(self):
        for scenario in ("safeop_recovery", "unrelated_error"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.executable), scenario])

    def test_sync_loss_still_detected_by_worker(self):
        self.run_command([str(self.executable), "sync_lost"])

    def test_hot_period_change_is_a_fault_not_an_implicit_reconfigure(self):
        self.run_command([str(self.executable), "hot_change"])

    def test_timer_inhibits_can_even_when_business_task_stalls(self):
        self.run_command([str(self.executable), "worker_stalled"])

    def test_session_restart_discards_staged_old_command(self):
        self.run_command([str(self.executable), "restart"])

    def test_reconfigure_cannot_reset_a_running_pipeline(self):
        self.run_command([str(self.executable), "reconfigure_running"])

    def test_cycle_and_rtos_diagnostics_are_independent_snapshots(self):
        self.run_command([str(self.executable), "diagnostics"])


if __name__ == "__main__":
    unittest.main()
