import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class RawCanGatewayTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cxx = shutil.which("c++")
        if not cxx:
            raise RuntimeError("Raw CAN tests require host c++")
        work = tempfile.TemporaryDirectory(prefix="dcu-raw-can-")
        cls.addClassCleanup(work.cleanup)
        cls.executable = Path(work.name) / "raw_can_gateway"
        flags = ["-std=c++17", "-fno-exceptions", "-fno-rtti", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            flags += ["-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"]
        # Build the gateway without motor, transport, BSP or EtherCAT headers.
        for include in ("firmware/ports", "firmware/app/gateway"):
            flags.extend(("-I", str(ROOT / include)))
        cls.run_command([
            cxx, *flags, str(ROOT / "tests/host/raw_can/raw_can_gateway_test.cpp"),
            str(ROOT / "firmware/app/gateway/can/raw_can_gateway.cpp"), "-o", str(cls.executable),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_sideband_sequence_wrap_rejection_and_consumption(self):
        self.run_command([str(self.executable), "sideband_sequence"])

    def test_full_bus_sequences_are_independent_and_gate_aware(self):
        self.run_command([str(self.executable), "full_bus_sequence"])

    def test_admission_bounds_and_drop_accounting(self):
        self.run_command([str(self.executable), "admission_bounds"])

    def test_failed_submission_does_not_charge_rate_and_reset_preserves_fifo(self):
        self.run_command([str(self.executable), "rate_and_reset"])

    def test_single_bus_is_selected_by_caller_policy(self):
        self.run_command([str(self.executable), "single_bus"])


if __name__ == "__main__":
    unittest.main()
