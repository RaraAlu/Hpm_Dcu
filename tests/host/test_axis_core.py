import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class AxisCoreTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cxx = shutil.which("c++")
        if not cxx:
            raise RuntimeError("Axis core tests require host c++")
        work = tempfile.TemporaryDirectory(prefix="dcu-axis-core-")
        cls.addClassCleanup(work.cleanup)
        cls.executable = Path(work.name) / "axis"
        flags = ["-DDCU_HOST_ENGINE_TEST", "-std=c++17", "-O2", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            flags += ["-g", "-fsanitize=address,undefined,float-cast-overflow", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"]
        # No generated, EtherCAT, HPM, RTOS or application-implementation headers.
        for include in ("firmware/comm/canfd", "firmware/devices/motor",
                        "firmware/devices/motor/drivers/encos", "firmware/devices/motor/drivers/bjcx", "firmware/protocols/encos", "firmware/protocols/bjcx",
                        "firmware/cfg", "firmware/ports"):
            flags.extend(("-I", str(ROOT / include)))
        sources = (
            "tests/host/motor/axis_core_test.cpp", "firmware/devices/motor/motor_instance.cpp",
            "firmware/devices/motor/motor_driver_binding.cpp", "firmware/devices/motor/drivers/encos/encos_parameter_transactions.cpp",
            "firmware/devices/motor/motor_driver.cpp", "firmware/devices/motor/drivers/encos/encos_driver.cpp",
            "firmware/devices/motor/drivers/encos/encos_session.cpp",
            "firmware/devices/motor/drivers/bjcx/bjcx_driver.cpp",
            "firmware/protocols/encos/encos_motor.cpp", "firmware/protocols/encos/encos_service.cpp",
            "firmware/protocols/bjcx/bjcx_status.cpp",
            "firmware/protocols/bjcx/bjcx_control.cpp",
        )
        cls.run_command([cxx, *flags, *(str(ROOT / source) for source in sources), "-o", str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_native_configuration_is_validated_without_wire_storage(self):
        self.run_command([str(self.executable), "config"])

    def test_native_axis_runs_startup_feedback_and_stop_without_ethercat(self):
        self.run_command([str(self.executable), "lifecycle"])

    def test_impedance_without_effort_starts_in_impedance_and_cannot_gain_effort_while_armed(self):
        self.run_command([str(self.executable), "impedance_start"])

    def test_parameter_write_codec_uses_explicit_values_and_transactional_output(self):
        self.run_command([str(self.executable), "parameters"])


if __name__ == "__main__":
    unittest.main()
