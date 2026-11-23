import os
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
        cls.trace_executable = Path(directory.name) / "tx_gate_trace"
        cls.tdc_executable = Path(directory.name) / "tx_gate_tdc"
        command = [cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
                   "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            command += ["-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"]
        for include in ("tests/host/canfd/stubs", "firmware/comm/canfd", "firmware/app/cycle",
                        "firmware/cfg", "firmware/ports"):
            command.extend(("-I", str(ROOT / include)))
        source = str(ROOT / "tests/host/canfd/tx_gate_test.c")
        cls.run_command([*command, "-DCONFIG_PDO_PHASE_TRACE=1", source, "-o", str(cls.trace_executable)])
        cls.run_command([*command, source, "-o", str(cls.executable)])
        cls.run_command([*command, "-DCONFIG_CAN3_TDCO_AB_TEST=1", source, "-o", str(cls.tdc_executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)

    def test_device_ownership_survives_out_of_order_fifo_retirement(self):
        self.run_command([str(self.executable), "device_pending"])

    def test_trace_converts_real_eof_and_counts_observed_tx_errors(self):
        for scenario in ("frame_eof", "retry_completed"):
            self.run_command([str(self.trace_executable), scenario])

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

    def test_transaction_is_visible_before_the_hardware_request(self):
        self.run_command([str(self.executable), "normal"])

    def test_channel_scoped_transactions_ignore_late_events_after_cancellation_and_reuse(self):
        self.run_command([str(self.executable), "transaction_ids"])

    def test_mcan_fifos_and_byte_exact_transport_through_64_bytes(self):
        self.run_command([str(self.executable), "fd_roundtrip"])

    def test_oversized_and_classic_high_dlc_frames_are_not_truncated(self):
        self.run_command([str(self.executable), "invalid_lengths"])

    def test_profile_initializes_only_the_selected_controller(self):
        self.run_command([str(self.executable), "single_profile"])

    def test_tdco_comparison_only_overrides_can3_when_enabled(self):
        self.run_command([str(self.executable), "tdc_baseline"])
        self.run_command([str(self.tdc_executable), "tdc_comparison"])

    def test_hardware_retry_keeps_one_token_and_preserves_first_error(self):
        self.run_command([str(self.executable), "retry_completed"])

    def test_cancelled_hardware_retry_cannot_publish_success(self):
        self.run_command([str(self.executable), "retry_cancelled"])

    def test_tx_rx_eof_metadata_uses_real_capture_slots(self):
        self.run_command([str(self.executable), "frame_eof"])

    def test_serial_tx_waits_for_eof_and_gap_not_rx_or_other_buses(self):
        self.run_command([str(self.executable), "serial_eof_gap"])

    def test_dedicated_chain_needs_no_worker_or_reply_and_cannot_outlive_window(self):
        for scenario in ("paced_chain", "paced_cancel", "paced_stale"):
            self.run_command([str(self.executable), scenario])

    def test_a_controller_not_in_eof_mode_cannot_activate(self):
        self.run_command([str(self.executable), "bad_eof_mode"])

    def test_a_controller_with_retries_disabled_cannot_activate(self):
        self.run_command([str(self.executable), "bad_dar_mode"])

    def test_invalid_or_partial_profile_does_not_activate_controllers(self):
        for scenario in ("invalid_profile", "partial_profile"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.executable), scenario])

    def test_first_protocol_fault_is_captured_before_ack_or_tx_bookkeeping(self):
        for source in ("irq", "poll", "poll_edge", "poll_read_clear", "psr_only", "tx_event", "cancel"):
            with self.subTest(source=source):
                self.run_command([str(self.executable), f"first_fault_{source}"])


if __name__ == "__main__":
    unittest.main()
