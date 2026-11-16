import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class MotorControlStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc, cxx = shutil.which("cc"), shutil.which("c++")
        if not cc or not cxx:
            raise RuntimeError("Motor-state tests require host cc/c++")
        directory = tempfile.TemporaryDirectory(prefix="dcu-motor-state-")
        cls.addClassCleanup(directory.cleanup)
        work = Path(directory.name)
        flags = ["-O0", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        for include in (
            "tests/host/motor/stubs", "firmware/cfg", "firmware/ports",
            "firmware/generated/encos", "firmware/generated/ethercat",
            "firmware/app/motor", "firmware/protocols/encos", "firmware/comm/canfd",
            "firmware/comm/ethercat", "third_party/beckhoff_ssc/Src",
            "firmware/devices/motor", "firmware/devices/motor/drivers/encos",
        ):
            flags.extend(("-I", str(ROOT / include)))
        storage = work / "storage.o"
        cls.run_command([
            cc, "-std=c11", *flags, "-c", str(ROOT / "firmware/comm/ethercat/encos_storage.c"),
            "-o", str(storage),
        ])
        cls.executable = work / "motor_state"
        cls.run_command([
            cxx, "-std=c++17", "-fno-exceptions", "-fno-rtti", *flags,
            str(ROOT / "tests/host/motor/motor_control_state.cpp"),
            str(ROOT / "firmware/app/motor/motor_driver_binding.cpp"),
            str(ROOT / "firmware/protocols/encos/encos_motor.cpp"),
            str(ROOT / "firmware/devices/motor/motor_driver.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_driver.cpp"),
            str(storage), "-lm", "-o", str(cls.executable),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_preparing_each_cycle_does_not_reuse_the_old_two_ms_cache(self):
        self.run_command([str(self.executable), "same_ms"])

    def test_mapping_is_read_only_and_preserves_snapshot_freshness(self):
        self.run_command([str(self.executable), "immutable"])

    def test_all_three_pdo_profiles_keep_their_input_boundaries(self):
        self.run_command([str(self.executable), "profiles"])

    def test_cycle_fault_clears_session_and_uses_existing_master_fault_bit(self):
        self.run_command([str(self.executable), "cycle_fault"])

    def test_invalid_pdo_does_not_override_stop_priority(self):
        self.run_command([str(self.executable), "invalid_stop"])


if __name__ == "__main__":
    unittest.main()
