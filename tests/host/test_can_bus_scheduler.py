import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class CanBusSchedulerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("c++")
        if not compiler:
            raise RuntimeError("CAN bus scheduler tests require a host C++17 compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-can-bus-scheduler-")
        cls.addClassCleanup(directory.cleanup)
        cls.executable = Path(directory.name) / "can_bus_scheduler"
        cls.run_command([
            compiler, "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-I", str(ROOT / "firmware/app/gateway"), "-I", str(ROOT / "firmware/cfg"),
            "-I", str(ROOT / "firmware/comm/canfd"), "-I", str(ROOT / "firmware/ports"),
            str(ROOT / "tests/host/motor/can_bus_scheduler_test.cpp"),
            str(ROOT / "firmware/app/gateway/can/can_bus_scheduler.cpp"), "-o", str(cls.executable),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_round_robin_and_no_advance_before_submission(self):
        self.run_command([str(self.executable), "round_robin"])

    def test_rate_phase_and_no_burst_after_a_stall(self):
        self.run_command([str(self.executable), "phase"])

    def test_budget_refill_saturation_and_clock_wrap(self):
        self.run_command([str(self.executable), "budget"])

    def test_queries_cannot_steal_a_control_deadline(self):
        self.run_command([str(self.executable), "query"])

    def test_progress_and_empty_queue_reset_the_stall_deadline(self):
        self.run_command([str(self.executable), "progress"])

    def test_bus_state_is_isolated(self):
        self.run_command([str(self.executable), "isolation"])

    def test_existing_bus_cost_estimates_are_preserved(self):
        self.run_command([str(self.executable), "frame_cost"])


if __name__ == "__main__":
    unittest.main()
