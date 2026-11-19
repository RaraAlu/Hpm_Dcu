import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class DeviceAbstractionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cxx = shutil.which("c++")
        if not cxx:
            raise RuntimeError("Device tests need host c++; install build-essential on Ubuntu")
        directory = tempfile.TemporaryDirectory(prefix="dcu-device-tests-")
        cls.addClassCleanup(directory.cleanup)
        cls.core = Path(directory.name) / "device_contracts"
        cls.adapter = Path(directory.name) / "encos_adapter"
        flags = ["-std=c++17", "-fno-exceptions", "-fno-rtti", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            flags += ["-g", "-fsanitize=address,undefined,float-cast-overflow", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"]
        for include in ("firmware/ports", "firmware/devices/motor", "firmware/devices/imu"):
            flags.extend(("-I", str(ROOT / include)))
        # No SDK, SSC, generated headers or EtherCAT includes for the common Interface.
        cls.run_command([
            cxx, *flags, str(ROOT / "tests/host/devices/device_contracts.cpp"),
            str(ROOT / "firmware/devices/motor/motor_driver.cpp"),
            str(ROOT / "firmware/devices/imu/imu_device.cpp"), "-o", str(cls.core),
        ])
        for include in (
            "firmware/cfg", "firmware/generated/encos", "firmware/protocols/encos",
            "firmware/devices/motor/drivers/encos", "firmware/devices/motor/drivers/cih408", "firmware/comm/canfd", "firmware/comm/ethercat",
        ):
            flags.extend(("-I", str(ROOT / include)))
        cls.run_command([
            cxx, *flags, str(ROOT / "tests/host/devices/encos_adapter.cpp"),
            str(ROOT / "tests/host/motor/encos_motion_reference.cpp"),
            str(ROOT / "firmware/devices/motor/motor_driver.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_driver.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_session.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/cih408/cih408_driver.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_parameter_transactions.cpp"),
            str(ROOT / "firmware/devices/motor/motor_driver_binding.cpp"),
            str(ROOT / "firmware/protocols/encos/encos_motor.cpp"),
            str(ROOT / "firmware/protocols/encos/encos_service.cpp"), "-lm", "-o", str(cls.adapter),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_motor_unbound_and_unsupported_modes_are_rejected(self):
        self.run_command([str(self.core), "motor_unbound"])

    def test_motor_implementation_is_replaceable_without_vendor_headers(self):
        self.run_command([str(self.core), "motor_substitution"])

    def test_motor_failed_outputs_are_not_committed(self):
        self.run_command([str(self.core), "motor_transactional"])

    def test_missing_imu_does_not_publish_valid_samples(self):
        self.run_command([str(self.core), "imu_unavailable"])

    def test_imu_initialization_is_nonblocking_and_retryable(self):
        self.run_command([str(self.core), "imu_startup"])

    def test_imu_freshness_uses_local_receipt_time(self):
        self.run_command([str(self.core), "imu_freshness"])

    def test_imu_invalid_and_unsupported_fields_are_rejected(self):
        self.run_command([str(self.core), "imu_invalid"])

    def test_imu_errors_invalidate_samples_and_can_recover(self):
        self.run_command([str(self.core), "imu_errors"])

    def test_imu_clock_rollback_and_zero_timeout_are_rejected(self):
        self.run_command([str(self.core), "imu_clock"])

    def test_production_registry_rejects_unknown_drivers(self):
        self.run_command([str(self.adapter), "registry"])

    def test_encos_configuration_keeps_existing_constraints(self):
        self.run_command([str(self.adapter), "validation"])

    def test_encos_command_bytes_match_frozen_reference(self):
        self.run_command([str(self.adapter), "commands"])

    def test_encos_invalid_commands_do_not_replace_output(self):
        self.run_command([str(self.adapter), "reject_commands"])

    def test_encos_feedback_preserves_existing_wire_fields(self):
        self.run_command([str(self.adapter), "feedback"])

    def test_encos_invalid_feedback_does_not_damage_previous_state(self):
        self.run_command([str(self.adapter), "reject_feedback"])

    def test_encos_command_boundaries_match_frozen_reference(self):
        self.run_command([str(self.adapter), "command_boundaries"])

    def test_encos_feedback_quantization_matches_frozen_reference(self):
        self.run_command([str(self.adapter), "feedback_boundaries"])

    def test_motor_driver_configuration_is_isolated_per_axis(self):
        self.run_command([str(self.adapter), "axis_isolation"])

    def test_axis_adapter_maps_command_feedback_and_diagnostic_fields(self):
        self.run_command([str(self.adapter), "field_mapping"])


if __name__ == "__main__":
    unittest.main()
