import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class CycleTimersTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise RuntimeError("Timebase tests require a host C compiler")
        work = tempfile.TemporaryDirectory(prefix="dcu-timebase-")
        cls.addClassCleanup(work.cleanup)
        cls.executable = Path(work.name) / "timebase"
        command = [cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        for include in ("tests/host/cycle/stubs", "firmware/bsp/dcu_hpm5e31", "firmware/cfg", "firmware/ports"):
            command.extend(("-I", str(ROOT / include)))
        cls.run_command([*command, str(ROOT / "tests/host/cycle/cycle_timers_test.c"), "-o", str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_wrong_clock_configuration_fails_before_start(self):
        self.run_command([str(self.executable), "frequency"])

    def test_origin_submillisecond_precision_and_independent_microsecond_wrap(self):
        self.run_command([str(self.executable), "elapsed"])

    def test_millisecond_counter_wrap_keeps_unsigned_duration(self):
        self.run_command([str(self.executable), "wrap"])

    def test_time_advances_without_led_rtos_or_interrupt_callbacks(self):
        self.run_command([str(self.executable), "delay"])

    def test_worker_timer_configuration_and_acknowledgement(self):
        self.run_command([str(self.executable), "worker"])


if __name__ == "__main__":
    unittest.main()
