import contextlib
import io
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "tests/target/can_internal_loopback"
sys.path.insert(0, str(ROOT / "scripts"))
import dcu


class CanInternalLoopbackTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc = shutil.which("cc")
        if not cc:
            raise RuntimeError("Internal loopback host tests require a C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-loopback-")
        cls.addClassCleanup(directory.cleanup)
        cls.executable = Path(directory.name) / "loopback"
        command = [cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        for include in ("tests/host/can_loopback/stubs", "tests/target/can_internal_loopback",
                        "firmware/ports", "firmware/bsp/dcu_hpm5e31"):
            command.extend(("-I", str(ROOT / include)))
        cls.run_command([*command, str(ROOT / "tests/host/can_loopback/loopback_test.c"),
                         "-o", str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        return result.stdout

    def run_scenarios(self, *names):
        for name in names:
            with self.subTest(scenario=name):
                self.run_command([str(self.executable), name])

    def test_irq_queue_wrap_and_each_transmit_completion(self):
        self.run_scenarios("normal")

    def test_microsecond_counter_wrap(self):
        self.run_scenarios("wrap")

    def test_separate_tx_and_rx_interrupts_in_either_order(self):
        self.run_scenarios("rx_then_tx", "tx_then_rx")

    def test_setup_failures_never_transmit(self):
        self.run_scenarios("timebase_failure", "clock_failure", "ram_failure", "init_failure")

    def test_mode_readback_is_required_before_first_transmit(self):
        self.run_scenarios("wrong_mode", "wrong_test_bit")

    def test_mode_is_rechecked_before_every_transmit(self):
        self.run_scenarios("mode_lost")

    def test_failed_staging_or_invalid_slot_does_not_submit(self):
        self.run_scenarios("tx_failure", "bad_slot")

    def test_exact_id_type_dlc_and_payload_are_required(self):
        self.run_scenarios("bad_data", "bad_id", "bad_dlc", "short_dlc", "fd", "brs", "rtr", "extended")

    def test_both_tx_and_rx_irq_paths_are_required(self):
        self.run_scenarios("no_tx", "no_rx", "no_irq")

    def test_late_frames_and_stopped_clock_cannot_pass(self):
        self.run_scenarios("late", "frozen_clock", "quiet_clock_failure")

    def test_controller_errors_and_failed_fifo_reads_fail(self):
        self.run_scenarios("irq_error", "fifo_lost", "read_failure", "ram_access_failure")

    def test_queue_overflow_and_duplicate_frames_fail(self):
        self.run_scenarios("overflow", "duplicate")

    def test_extra_frame_after_final_exchange_fails(self):
        self.run_scenarios("late_rx")

    def test_pending_frame_or_error_at_stop_cannot_leave_a_false_pass(self):
        self.run_scenarios("pending_rx_at_stop", "pending_error_at_stop")

    def test_stop_must_be_acknowledged_before_pass(self):
        self.run_scenarios("stop_failure")

    def test_compiled_payloads_match_independent_yaml_golden_bytes(self):
        name = "\u6700\u5c0f\u56de\u73af\u6d4b\u8bd5\u5411\u91cf.yaml"
        vectors = yaml.safe_load((ROOT / "tools/protocol/cfg" / name).read_text())
        expected = [bytes.fromhex(case["hex"]) for case in vectors["cases"] if case["record"] == "can_echo"]
        actual = self.run_command([str(self.executable), "payloads"])
        self.assertEqual([bytes.fromhex(line) for line in actual.splitlines()], expected)

    def test_diagnostic_is_opt_in_and_uses_an_isolated_build_directory(self):
        presets = json.loads((ROOT / "cmake/cfg/CMakePresets.json").read_text())
        configure = {preset["name"]: preset for preset in presets["configurePresets"]}
        self.assertFalse(configure["hpm-base"]["cacheVariables"]["CONFIG_CAN_INTERNAL_LOOPBACK_TEST"])
        diagnostic = configure["hpm-can-loopback"]
        self.assertEqual(diagnostic["inherits"], "hpm-base")
        self.assertTrue(diagnostic["cacheVariables"]["CONFIG_CAN_INTERNAL_LOOPBACK_TEST"])
        self.assertEqual(configure["hpm-base"]["binaryDir"], "${sourceDir}/build/${presetName}")
        self.assertNotIn("hpm-can-loopback", dcu.PRESETS)
        self.assertIn("hpm-can-loopback", dcu.TEST_PRESETS)
        self.assertEqual(dcu.elf_path("hpm-can-loopback"), ROOT / "build/hpm-can-loopback/output/dcu.elf")
        with mock.patch.object(dcu, "build") as build:
            self.assertEqual(dcu.main(["build", "--preset", "hpm-can-loopback"]), 0)
            build.assert_called_once_with("hpm-can-loopback")

    def test_diagnostic_target_actions_retain_explicit_confirmation(self):
        for action in ("flash", "verify", "debug-server"):
            with self.subTest(action=action), mock.patch.dict(dcu.os.environ, {}, clear=True), \
                 mock.patch.object(dcu, "run") as run, mock.patch.object(dcu, "require_board") as board, \
                 mock.patch.object(dcu.os, "execve") as execute, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(dcu.main([action, "--preset", "hpm-can-loopback"]), 1)
                run.assert_not_called()
                board.assert_not_called()
                execute.assert_not_called()

    def test_test_entry_does_not_call_production_or_pinmux_paths(self):
        source = (TARGET / "main.c").read_text() + (TARGET / "can_internal_loopback.c").read_text()
        for name in ("motor_control_init", "MainInit", "ecat_hardware_init", "dcu_rtos_start",
                     "board_init_ethercat", "board_init_can", "board_can_transceiver_phy_set",
                     "dcu_mcan_init", "dcu_can_submit"):
            self.assertNotRegex(source, rf"\b{name}\s*\(")
        for name in ("HPM_MCAN1", "HPM_MCAN2", "HPM_MCAN3"):
            self.assertNotIn(name, source)
        self.assertIn("dcu_can_loopback_finished();", source)
        self.assertNotIn("mcan_read_rxfifo", source[source.index("static bool receive"):])

    def test_new_implementations_keep_types_and_macros_in_headers(self):
        declaration = re.compile(r"^\s*(?:#\s*define\b|typedef\b|struct\b|enum\b)", re.MULTILINE)
        for path in TARGET.glob("*.c"):
            with self.subTest(path=path.name):
                self.assertIsNone(declaration.search(path.read_text()))


if __name__ == "__main__":
    unittest.main()
