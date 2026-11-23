import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class ApplicationCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cxx = shutil.which("c++")
        if not cxx:
            raise RuntimeError("Application core tests require host c++")
        work = tempfile.TemporaryDirectory(prefix="dcu-application-core-")
        cls.addClassCleanup(work.cleanup)
        cls.executable = Path(work.name) / "application"
        flags = ["-DDCU_HOST_ENGINE_TEST", "-std=c++17", "-O2", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            flags += ["-g", "-fsanitize=address,undefined,float-cast-overflow", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"]
        # Compile the gateway application and motor devices without generated/EtherCAT/SSC/HPM/RTOS headers.
        for include in ("firmware/app/gateway", "firmware/app/profile", "firmware/comm/canfd", "firmware/devices/motor",
                        "firmware/devices/motor/drivers/encos", "firmware/devices/motor/drivers/bjcx", "firmware/protocols/encos", "firmware/protocols/bjcx",
                        "firmware/cfg", "firmware/ports"):
            flags.extend(("-I", str(ROOT / include)))
        sources = [ROOT / "tests/host/motor/application_core_test.cpp"]
        for directory in ("firmware/app/gateway", "firmware/devices/motor", "firmware/protocols"):
            sources.extend(sorted((ROOT / directory).rglob("*.cpp")))
        readiness = Path(work.name) / "readiness.o"
        c_flags = [flag for flag in flags if flag not in ("-std=c++17", "-fno-exceptions", "-fno-rtti")]
        cls.run_command(["cc", "-std=c11", *c_flags, "-c",
                         str(ROOT / "firmware/app/gateway/startup/gateway_readiness.c"), "-o", str(readiness)])
        cls.run_command([cxx, *flags, str(readiness), *(str(source) for source in sources), "-o", str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_configuration_validation_and_mode_admission_need_no_dictionary(self):
        self.run_command([str(self.executable), "configuration"])

    def test_production_profile_binds_drivers_on_applied_buses_without_ethercat(self):
        self.run_command([str(self.executable), "production_profile"])

    def test_seven_bus_local_slots_pipeline_with_legacy_owner_at_profile_index_seven(self):
        self.run_command([str(self.executable), "interleaved_slots"])

    def test_faulted_bus_does_not_skip_later_queries_and_raw_stays_last(self):
        self.run_command([str(self.executable), "query_before_sideband"])

    def test_native_motion_and_publication_keep_stop_priority_and_private_state(self):
        self.run_command([str(self.executable), "stop"])

    def test_four_mit_submissions_do_not_wait_for_replies_and_require_owned_eof(self):
        for layout in ("cyclic_mit_with_raw", "cyclic_mit"):
            with self.subTest(layout=layout):
                self.run_command([str(self.executable), layout])

    def test_sdo_sequence_authority_and_duplicate_lease_are_private(self):
        self.run_command([str(self.executable), "sdo"])

    def test_owner_recovery_and_explicit_replacement_survive_session_transitions(self):
        self.run_command([str(self.executable), "owner"])

    def test_full_raw_mode_runs_from_typed_batches_and_applied_configuration(self):
        self.run_command([str(self.executable), "raw"])


if __name__ == "__main__":
    unittest.main()
