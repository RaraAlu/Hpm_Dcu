import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class ParameterTransactionsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cxx = shutil.which("c++")
        if not cxx:
            raise RuntimeError("Parameter transaction tests require host c++")
        work = tempfile.TemporaryDirectory(prefix="dcu-parameters-")
        cls.addClassCleanup(work.cleanup)
        cls.executable = Path(work.name) / "transactions"
        command = [cxx, "-std=c++17", "-O2", "-fno-exceptions", "-fno-rtti",
                   "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            command += ["-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"]
        # No dictionary, generated types, devices, axis, SSC, HPM or RTOS stubs.
        for include in ("firmware/app/motor", "firmware/comm/canfd", "firmware/protocols/encos",
                        "firmware/cfg", "firmware/ports"):
            command.extend(("-I", str(ROOT / include)))
        cls.run_command([*command, str(ROOT / "tests/host/motor/parameter_transactions_test.cpp"),
                         str(ROOT / "firmware/app/motor/encos_parameter_transactions.cpp"),
                         str(ROOT / "firmware/protocols/encos/encos_service.cpp"), "-o", str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_request_is_copied_and_write_readback_completes_the_original_sequence(self):
        self.run_command([str(self.executable), "request"])

    def test_rejected_and_duplicate_requests_do_not_change_inflight_state(self):
        self.run_command([str(self.executable), "admission"])

    def test_hold_is_explicit_wrap_safe_and_does_not_block_long_idle_uptime(self):
        self.run_command([str(self.executable), "hold"])

    def test_all_supported_scalar_and_pair_query_results_are_typed(self):
        self.run_command([str(self.executable), "values"])

    def test_uuid_requires_all_parts_and_version_bytes_remain_ordered(self):
        self.run_command([str(self.executable), "identity"])

    def test_invalid_partial_and_late_replies_never_emit_cache_updates(self):
        self.run_command([str(self.executable), "invalid"])

    def test_all_kt_groups_use_special_reply_and_ignore_ambiguous_mode_bits(self):
        self.run_command([str(self.executable), "kt_groups"])

    def test_kt_write_requires_independent_readback_and_reports_mismatch(self):
        self.run_command([str(self.executable), "kt_write"])

    def test_encoder_reply_checks_length_state_and_signed_angle_bounds(self):
        self.run_command([str(self.executable), "encoder"])


if __name__ == "__main__":
    unittest.main()
