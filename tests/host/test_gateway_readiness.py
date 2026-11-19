import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "firmware/app/gateway"
SUITE = ROOT / "tests/host/gateway/gateway_readiness_test.c"


class GatewayReadinessTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise RuntimeError("Gateway readiness tests require a host C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-gateway-readiness-")
        cls.addClassCleanup(directory.cleanup)
        cls.runner = Path(directory.name) / "gateway_readiness"
        flags = ["-O2"]
        if os.getenv("DCU_GATEWAY_SANITIZE") == "1":
            flags = ["-O1", "-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-no-pie"]
        cls.run_checked([
            cc, "-std=c11", *flags, "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-I", str(CORE), str(CORE / "gateway_readiness.c"), str(SUITE), "-o", str(cls.runner),
        ])

    @staticmethod
    def run_checked(cmd):
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {cmd}\n{result.stdout}{result.stderr}")

    def test_ready_before_master_and_first_confirmation(self):
        self.run_checked([str(self.runner), "startup"])

    def test_profile_and_context_validation(self):
        self.run_checked([str(self.runner), "mismatch"])

    def test_one_allocation_and_idempotent_retry(self):
        self.run_checked([str(self.runner), "duplicate"])

    def test_stale_evidence_and_exact_boundary(self):
        self.run_checked([str(self.runner), "freshness"])

    def test_recovery_requires_fresh_evidence_and_confirmation(self):
        self.run_checked([str(self.runner), "recovery"])

    def test_retry_does_not_extend_activation_deadline(self):
        self.run_checked([str(self.runner), "deadline"])

    def test_invalid_clock_and_invalid_profile_fail_closed(self):
        for case in ("clock_regression", "invalid_profile", "near_clock_limit"):
            with self.subTest(case=case):
                self.run_checked([str(self.runner), case])

    def test_allocator_exhaustion_or_reuse_never_reopens_session(self):
        for case in ("exhausted", "reused_session"):
            with self.subTest(case=case):
                self.run_checked([str(self.runner), case])


if __name__ == "__main__":
    unittest.main()
