import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class MotorProtocolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc, cxx = shutil.which("cc"), shutil.which("c++")
        if not cc or not cxx:
            raise RuntimeError("Motor tests need host cc/c++; install build-essential on Ubuntu")
        directory = tempfile.TemporaryDirectory(prefix="dcu-motor-tests-")
        cls.addClassCleanup(directory.cleanup)
        work = Path(directory.name)
        includes = (
            "firmware/cfg", "firmware/ports", "firmware/generated/encos",
            "firmware/protocols/encos", "firmware/app/motor",
        )
        flags = ["-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        for include in includes:
            flags.extend(("-I", str(ROOT / include)))
        objects = []
        for source in (ROOT / "firmware/comm/ethercat/encos_storage.c", ROOT / "tests/host/motor/motor_c_abi.c"):
            obj = work / (source.stem + ".o")
            cls.run_command([cc, "-std=c11", *flags, "-c", str(source), "-o", str(obj)])
            objects.append(str(obj))
        cls.executable = work / "motor_protocol"
        cls.run_command([
            cxx, "-std=c++17", "-fno-exceptions", "-fno-rtti", *flags,
            str(ROOT / "firmware/protocols/encos/encos_motor.cpp"),
            str(ROOT / "tests/host/motor/motor_protocol.cpp"), *objects, "-lm",
            "-o", str(cls.executable),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_c_cpp_linkage(self):
        self.run_command([str(self.executable), "abi"])

    def test_layout_and_object_storage(self):
        self.run_command([str(self.executable), "layout"])

    def test_profile_limits(self):
        self.run_command([str(self.executable), "profile"])

    def test_five_control_modes(self):
        self.run_command([str(self.executable), "commands"])

    def test_invalid_commands(self):
        self.run_command([str(self.executable), "reject"])

    def test_feedback_conversion(self):
        self.run_command([str(self.executable), "feedback"])

    def test_queries_and_parameters(self):
        self.run_command([str(self.executable), "parameters"])

    def test_float_input_validation(self):
        self.run_command([str(self.executable), "float"])


if __name__ == "__main__":
    unittest.main()
