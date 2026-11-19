import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class PdoPipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise RuntimeError("PDO pipeline tests require a host C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-pdo-pipeline-")
        cls.addClassCleanup(directory.cleanup)
        cls.executable = Path(directory.name) / "pdo_pipeline"
        cls.run_command([
            cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-I", str(ROOT / "firmware/app/cycle"),
            "-I", str(ROOT / "firmware/cfg"),
            str(ROOT / "tests/host/cycle/pdo_pipeline_test.c"),
            "-o", str(cls.executable),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_master_period_range_and_configuration_rejection(self):
        self.run_command([str(self.executable), "configuration"])

    def test_command_changes_only_at_sync0_boundary(self):
        self.run_command([str(self.executable), "boundary"])

    def test_sync0_can_only_latch_after_publish_exchange(self):
        self.run_command([str(self.executable), "publication_edge"])

    def test_latest_complete_frame_wins_before_boundary(self):
        self.run_command([str(self.executable), "latest"])

    def test_reader_buffer_is_not_reused_by_producer(self):
        self.run_command([str(self.executable), "ownership"])

    def test_unconsumed_cycle_is_reported_without_catchup(self):
        self.run_command([str(self.executable), "overrun"])

    def test_missing_frame_does_not_replay_previous_command(self):
        self.run_command([str(self.executable), "missing"])

    def test_feedback_deadline_and_wrong_epoch_rejection(self):
        self.run_command([str(self.executable), "deadline"])

    def test_sync_interrupt_between_time_sample_and_check(self):
        self.run_command([str(self.executable), "preempted_time"])

    def test_sync_interrupt_cannot_fault_a_completed_epoch(self):
        self.run_command([str(self.executable), "preempted_watchdog"])

    def test_publish_timeout_and_sync_loss_are_latched(self):
        self.run_command([str(self.executable), "faults"])

    def test_timer_and_cycle_sequence_wrap(self):
        self.run_command([str(self.executable), "wrap"])

    def test_stopped_session_cannot_replay_old_frame(self):
        self.run_command([str(self.executable), "reset"])


if __name__ == "__main__":
    unittest.main()
