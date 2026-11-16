import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class CanFdTxGateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise RuntimeError("CAN TX gate tests require a host C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-canfd-gate-")
        cls.addClassCleanup(directory.cleanup)
        cls.executable = Path(directory.name) / "tx_gate"
        command = [cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
                   "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections"]
        for include in ("tests/host/canfd/stubs", "firmware/comm/canfd", "firmware/app/cycle",
                        "firmware/cfg", "firmware/ports"):
            command.extend(("-I", str(ROOT / include)))
        cls.run_command([*command, str(ROOT / "tests/host/canfd/tx_gate_test.c"), "-o", str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_fault_while_staging_cannot_submit_after_cancellation(self):
        self.run_command([str(self.executable), "staging_fault"])

    def test_commit_and_fault_timer_have_a_single_ordering(self):
        self.run_command([str(self.executable), "commit_fault"])

    def test_inhibited_send_does_not_stage_any_frame(self):
        self.run_command([str(self.executable), "already_inhibited"])

    def test_failed_staging_does_not_change_tag_or_submit(self):
        self.run_command([str(self.executable), "stage_failure"])

    def test_invalid_bus_and_null_frame_are_rejected(self):
        self.run_command([str(self.executable), "invalid"])

    def test_tag_is_visible_before_the_hardware_request(self):
        self.run_command([str(self.executable), "normal"])


if __name__ == "__main__":
    unittest.main()
