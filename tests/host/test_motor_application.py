import shutil
import os
import subprocess
import sys
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
        cls.run_command([
            sys.executable, str(ROOT / "tools/protocol/generate_can_profile.py"),
            "--profile", str(ROOT / "tests/host/motor/cfg/单路ENCOS验证配置.yaml"),
            "--output", str(work / "dcu_can_profile_generated.h"),
        ])
        flags = ["-I", str(work), "-DDCU_HOST_ENGINE_TEST", "-O0", "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
        if os.environ.get("DCU_HOST_SANITIZE") == "1":
            flags.extend(("-g", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie", "-no-pie"))
        for include in (
            "tests/host/motor/stubs", "firmware/cfg", "firmware/ports",
            "firmware/generated/encos", "firmware/generated/ethercat",
            "firmware/app", "firmware/app/gateway", "firmware/app/cycle",
            "firmware/protocols/encos", "firmware/comm/canfd",
            "firmware/comm/ethercat", "third_party/beckhoff_ssc/Src",
            "firmware/devices/motor", "firmware/devices/motor/drivers/encos", "firmware/devices/motor/drivers/bjcx",
        ):
            flags.extend(("-I", str(ROOT / include)))
        storage = work / "storage.o"
        cls.run_command([
            cc, "-std=c11", *flags, "-c", str(ROOT / "firmware/comm/ethercat/encos_storage.c"),
            "-o", str(storage),
        ])
        fixture = work / "fixture.o"
        cls.run_command([
            cc, "-std=c11", *flags, "-Ddcu_production_profile=host_fixture_profile",
            "-c", str(ROOT / "firmware/app/gateway/production_profile.c"), "-o", str(fixture),
        ])
        readiness = work / "readiness.o"
        cls.run_command([cc, "-std=c11", *flags, "-c",
                         str(ROOT / "firmware/app/gateway/gateway_readiness.c"), "-o", str(readiness)])
        mixed = work / "mixed.o"
        cls.run_command([
            sys.executable, str(ROOT / "tools/protocol/generate_can_profile.py"),
            "--profile", str(ROOT / "tests/host/motor/cfg/混挂电机验证配置.yaml"),
            "--output", str(work / "mixed/dcu_can_profile_generated.h"),
        ])
        cls.run_command([
            cc, "-std=c11", "-I", str(work / "mixed"), *flags,
            "-Ddcu_production_profile=host_mixed_profile",
            "-c", str(ROOT / "firmware/app/gateway/production_profile.c"), "-o", str(mixed),
        ])
        cls.executable = work / "motor_application"
        cls.run_command([
            cxx, "-std=c++17", "-fno-exceptions", "-fno-rtti", *flags,
            str(ROOT / "tests/host/motor/motor_application_test.cpp"),
            str(ROOT / "firmware/app/production_init.cpp"),
            str(ROOT / "firmware/app/gateway/gateway_application.cpp"),
            str(ROOT / "firmware/app/gateway/gateway_publication.cpp"),
            str(ROOT / "firmware/app/gateway/production_startup.cpp"),
            str(ROOT / "firmware/app/gateway/profile_update.cpp"),
            str(ROOT / "firmware/app/gateway/can_transactions.cpp"),
            str(ROOT / "firmware/devices/motor/motor_instance.cpp"),
            str(ROOT / "firmware/app/gateway/raw_can_gateway.cpp"),
            str(ROOT / "firmware/devices/motor/motor_driver_binding.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_parameter_transactions.cpp"),
            str(ROOT / "firmware/app/gateway/can_bus_scheduler.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_dictionary.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_coe.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_adapter.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_pdo.cpp"),
            str(ROOT / "firmware/comm/ethercat/motor_services.cpp"),
            str(ROOT / "firmware/protocols/encos/encos_motor.cpp"),
            str(ROOT / "firmware/protocols/encos/encos_service.cpp"),
            str(ROOT / "firmware/devices/motor/motor_driver.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_driver.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/encos/encos_session.cpp"),
            str(ROOT / "firmware/devices/motor/drivers/bjcx/bjcx_driver.cpp"),
            str(storage), str(fixture), str(mixed), str(readiness), "-lm", "-o", str(cls.executable),
        ])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_preparing_each_cycle_does_not_reuse_the_old_two_ms_cache(self):
        self.run_command([str(self.executable), "same_ms"])

    def test_input_adapter_owns_the_candidate_until_the_worker_consumes_it(self):
        self.run_command([str(self.executable), "input_staging"])

    def test_application_uses_private_owner_mode_configuration_and_counters(self):
        self.run_command([str(self.executable), "application_authority"])

    def test_sdo_acceptance_does_not_trust_the_dictionary_sequence_mirror(self):
        self.run_command([str(self.executable), "sdo_sequence_mirror"])

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

    def test_static_motor_slots_are_restored_after_raw_mapping(self):
        self.run_command([str(self.executable), "latched_layouts"])

    def test_cycle_fault_clears_session_and_uses_existing_master_fault_bit(self):
        self.run_command([str(self.executable), "cycle_fault"])

    def test_invalid_pdo_does_not_override_stop_priority(self):
        self.run_command([str(self.executable), "invalid_stop"])

    def test_profile_supersedes_the_dictionary_and_legacy_objects_are_reserved(self):
        self.run_command([str(self.executable), "config_source"])

    def test_raw_ring_has_bounded_capacity_and_preserves_fifo_order(self):
        self.run_command([str(self.executable), "raw_queue"])

    def test_rejected_parameter_encoding_cannot_corrupt_the_next_transaction(self):
        self.run_command([str(self.executable), "service_rejected"])

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

    def test_queries_preserve_telemetry_without_repopulating_old_config(self):
        self.run_command([str(self.executable), "service_cache"])

    def test_encoder_query_updates_only_defined_cache_fields(self):
        self.run_command([str(self.executable), "service_extended"])

    def test_inflight_service_uses_captured_route_dialect_and_frame_format(self):
        self.run_command([str(self.executable), "service_snapshot"])

    def test_busy_transaction_blocks_owner_change_and_session_exit_completes_it(self):
        self.run_command([str(self.executable), "service_cancel"])

    def test_pdo_codec_matches_fixed_wire_offsets_and_bytes(self):
        self.run_command([str(self.executable), "codec"])

    def test_axis_objects_own_independent_configuration_and_startup_state(self):
        self.run_command([str(self.executable), "axis_objects"])

    def test_axis_stop_requires_a_new_permitted_clear_and_arm_edge(self):
        self.run_command([str(self.executable), "axis_rearm"])

    def test_axis_startup_has_distinct_query_submission_feedback_and_running_phases(self):
        self.run_command([str(self.executable), "axis_phases"])

    def test_axis_startup_expiration_and_first_feedback_deadline_across_wrap(self):
        self.run_command([str(self.executable), "axis_startup_deadlines"])

    def test_axis_controls_use_a_copy_of_the_bound_settings(self):
        self.run_command([str(self.executable), "axis_config_snapshot"])

    def test_axis_safety_decisions_do_not_read_dictionary_or_snapshot_storage(self):
        self.run_command([str(self.executable), "axis_runtime_authority"])

    def test_axis_snapshot_reads_are_const_and_only_publication_acknowledges_events(self):
        self.run_command([str(self.executable), "axis_publication"])

    def test_device_rebind_resets_runtime_freshness_but_preserves_diagnostic_history(self):
        self.run_command([str(self.executable), "axis_history"])

    def test_raw_objects_own_independent_queues_and_drop_counters(self):
        self.run_command([str(self.executable), "raw_objects"])

    def test_raw_batch_deduplication_and_two_ms_rate_window(self):
        self.run_command([str(self.executable), "raw_limits"])

    def test_raw_filtering_gating_and_failed_send_do_not_consume_credit(self):
        self.run_command([str(self.executable), "raw_filter"])

    def test_raw_classic_and_fd_frames_charge_the_exact_shared_bus_cost(self):
        self.run_command([str(self.executable), "raw_costs"])

    def test_raw_full_session_restarts_its_cycle_deduplication(self):
        self.run_command([str(self.executable), "raw_full_session"])

    def test_raw_admission_ignores_writable_dictionary_storage(self):
        self.run_command([str(self.executable), "raw_applied_policy"])

    def test_raw_pdo_decoder_preserves_header_slot_and_flag_rules(self):
        self.run_command([str(self.executable), "raw_codec"])

    def test_legacy_raw_pdo_never_sends_or_publishes_overlong_frames(self):
        for scenario in ("raw_large_sideband", "raw_large_full"):
            with self.subTest(scenario=scenario):
                self.run_command([str(self.executable), scenario])

    def test_generated_profile_binds_without_legacy_sdo_and_unconfigured_slot_cannot_transmit(self):
        self.run_command([str(self.executable), "can_profile_config"])

    def test_dense_device_order_and_reply_identity_are_independent_of_wire_slot_and_can_id(self):
        self.run_command([str(self.executable), "cross_bus_device_mapping"])

    def test_mixed_bjcx_and_encos_preserve_routing_transport_and_diagnostic_only_policy(self):
        self.run_command([str(self.executable), "mixed_motor_profile"])

    def test_production_reconnect_reuses_pdo_and_requires_neutral_after_link_loss(self):
        self.run_command([str(self.executable), "production_startup"])

    def test_production_startup_fails_closed_without_a_boot_journal(self):
        self.run_command([str(self.executable), "production_no_journal"])

    def test_dynamic_policy_is_validated_committed_and_enforced_without_changing_pdo(self):
        candidate = self.executable.parent / "policy.bin"
        self.run_command([
            sys.executable, str(ROOT / "tools/protocol/encode_profile_update.py"),
            str(ROOT / "tests/host/motor/cfg/动态策略验证配置.yaml"),
            "--boot-epoch", "7", "--output", str(candidate),
        ])
        self.run_command([str(self.executable), "dynamic_policy", str(candidate)])

    def test_empty_device_list_does_not_fall_back_to_legacy_can_ids(self):
        self.run_command([str(self.executable), "empty_profile"])

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
