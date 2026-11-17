import shutil
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


class MotorApplicationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cc, cxx = shutil.which("cc"), shutil.which("c++")
        if not cc or not cxx:
            raise RuntimeError("Motor-application tests require host cc/c++")
        directory = tempfile.TemporaryDirectory(prefix="dcu-motor-application-")
        cls.addClassCleanup(directory.cleanup)
        work = Path(directory.name)
        flags = ["-O0", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            flags.extend(("-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"))
        for include in (
            "tests/host/motor/stubs", "firmware/cfg", "firmware/ports",
            "firmware/generated/encos", "firmware/generated/ethercat",
            "firmware/app/motor", "firmware/app/cycle", "firmware/protocols/encos", "firmware/comm/canfd",
            "firmware/comm/ethercat", "third_party/beckhoff_ssc/Src",
            "firmware/devices/motor", "firmware/devices/motor/drivers/encos",
        ):
            flags.extend(("-I", str(ROOT / include)))
        storage = work / "storage.o"
        cls.run_command([
            cc, "-std=c11", *flags, "-c", str(ROOT / "firmware/comm/ethercat/encos_storage.c"),
            "-o", str(storage),
        ])
        cls.executable = work / "motor_application"
        cls.run_command([
            cxx, "-std=c++17", "-fno-exceptions", "-fno-rtti", *flags,
            str(ROOT / "tests/host/motor/motor_application_test.cpp"),
            str(ROOT / "firmware/app/motor/motor_application.cpp"),
            str(ROOT / "firmware/app/motor/axis_controller.cpp"),
            str(ROOT / "firmware/app/motor/raw_can_gateway.cpp"),
            str(ROOT / "firmware/app/motor/motor_driver_binding.cpp"),
            str(ROOT / "firmware/app/motor/encos_parameter_transactions.cpp"),
            str(ROOT / "firmware/app/motor/can_bus_scheduler.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_dictionary.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_coe.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_pdo.cpp"),
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

    def test_sdo_reads_refresh_feedback_without_consuming_pdo_events_or_mutating_the_image(self):
        self.run_command([str(self.executable), "sdo_publication"])

    def test_pdo_accepted_command_mirror_does_not_change_on_stop(self):
        self.run_command([str(self.executable), "accepted_pdo"])

    def test_sdo_accepted_command_mirror_updates_only_for_valid_motion(self):
        self.run_command([str(self.executable), "accepted_sdo"])

    def test_all_three_pdo_profiles_keep_their_input_boundaries(self):
        self.run_command([str(self.executable), "profiles"])

    def test_cycle_fault_clears_session_and_uses_existing_master_fault_bit(self):
        self.run_command([str(self.executable), "cycle_fault"])

    def test_invalid_pdo_does_not_override_stop_priority(self):
        self.run_command([str(self.executable), "invalid_stop"])

    def test_initial_board_defaults_come_from_the_generated_dictionary(self):
        self.run_command([str(self.executable), "config_source"])

    def test_raw_ring_has_bounded_capacity_and_preserves_fifo_order(self):
        self.run_command([str(self.executable), "raw_queue"])

    def test_rejected_parameter_encoding_cannot_corrupt_the_next_transaction(self):
        self.run_command([str(self.executable), "service_rejected"])

    def test_parameter_write_ack_and_readback_complete_the_same_transaction(self):
        self.run_command([str(self.executable), "service_verify"])

    def test_parameter_readback_mismatch_is_reported(self):
        self.run_command([str(self.executable), "service_mismatch"])

    def test_parameter_timeout_completes_and_releases_the_transaction(self):
        self.run_command([str(self.executable), "service_timeout"])

    def test_transaction_owns_queued_and_sent_deadlines_including_clock_wrap(self):
        self.run_command([str(self.executable), "transaction_deadlines"])

    def test_transaction_cancellation_is_idempotent_isolated_and_preserves_user_requests(self):
        self.run_command([str(self.executable), "transaction_cancellation"])

    def test_parameter_parser_returns_observations_without_writing_axis_query_state(self):
        self.run_command([str(self.executable), "transaction_observation"])

    def test_parameter_observations_update_axis_feedback(self):
        self.run_command([str(self.executable), "service_observations"])

    def test_maintenance_before_apply_matches_the_sent_can_id_and_holds_off(self):
        self.run_command([str(self.executable), "service_maintenance"])

    def test_busy_transaction_blocks_owner_change_and_session_exit_completes_it(self):
        self.run_command([str(self.executable), "service_cancel"])

    def test_pdo_codec_matches_fixed_wire_offsets_and_bytes(self):
        self.run_command([str(self.executable), "codec"])

    def test_axis_objects_own_independent_configuration_and_startup_state(self):
        self.run_command([str(self.executable), "axis_objects"])

    def test_axis_stop_requires_a_new_permitted_clear_and_arm_edge(self):
        self.run_command([str(self.executable), "axis_rearm"])

    def test_pending_configuration_changes_do_not_replace_applied_driver_settings(self):
        self.run_command([str(self.executable), "axis_config_snapshot"])

    def test_axis_safety_decisions_do_not_read_dictionary_or_snapshot_storage(self):
        self.run_command([str(self.executable), "axis_runtime_authority"])

    def test_axis_snapshot_reads_are_const_and_only_publication_acknowledges_events(self):
        self.run_command([str(self.executable), "axis_publication"])

    def test_apply_resets_runtime_freshness_but_preserves_diagnostic_history(self):
        self.run_command([str(self.executable), "axis_history"])

    def test_preapply_maintenance_routes_to_a_nonzero_axis_on_another_bus(self):
        self.run_command([str(self.executable), "service_other_axis"])

    def test_raw_objects_own_independent_queues_and_drop_counters(self):
        self.run_command([str(self.executable), "raw_objects"])

    def test_raw_batch_deduplication_and_two_ms_rate_window(self):
        self.run_command([str(self.executable), "raw_limits"])

    def test_raw_filtering_gating_and_failed_send_do_not_consume_credit(self):
        self.run_command([str(self.executable), "raw_filter"])

    def test_raw_classic_and_fd_frames_charge_the_exact_shared_bus_cost(self):
        self.run_command([str(self.executable), "raw_costs"])

    def test_raw_traffic_respects_cancelled_service_hold_until_deadline_or_session_reset(self):
        self.run_command([str(self.executable), "raw_service_hold"])

    def test_raw_full_session_restarts_its_cycle_deduplication(self):
        self.run_command([str(self.executable), "raw_full_session"])

    def test_raw_admission_uses_applied_routes_and_channel_configuration(self):
        self.run_command([str(self.executable), "raw_applied_policy"])

    def test_raw_pdo_decoder_preserves_header_slot_and_flag_rules(self):
        self.run_command([str(self.executable), "raw_codec"])

    def test_raw_full_input_preserves_sequence_gates_and_dictionary_mirrors(self):
        self.run_command([str(self.executable), "raw_full_admission"])

    def test_start_requires_a_separate_query_only_handshake(self):
        self.run_command([str(self.executable), "startup_gate"])

    def test_command_lease_expiry_stops_the_running_axis(self):
        self.run_command([str(self.executable), "lease_timeout"])

    def test_feedback_timeout_stops_even_with_fresh_commands(self):
        self.run_command([str(self.executable), "feedback_timeout"])

    def test_bus_off_stops_the_affected_axis(self):
        self.run_command([str(self.executable), "bus_off"])


if __name__ == "__main__":
    unittest.main()
