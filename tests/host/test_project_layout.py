import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import dcu


class ProjectLayoutTest(unittest.TestCase):
    def test_old_source_roots_are_absent(self):
        for name in (
            "application", "port", "SSC", "hpm_sdk_localized_for_hpm5e00evk",
            "ecat.c", "ecat_config.h", "user_config.h",
            "legacy", "docs/历史资料", "CMakeLists.txt.localized.bak",
            "README_en.rst", "README_zh.rst",
        ):
            with self.subTest(name=name):
                self.assertFalse((ROOT / name).exists())

    def test_product_headers_have_single_owners(self):
        owners = {
            "board.h": "firmware/bsp/dcu_hpm5e31/board.h",
            "digital_io.h": "firmware/comm/ethercat/digital_io.h",
            "ecat_def.h": "firmware/cfg/ecat_def.h",
            "dcu_compiler.h": "firmware/cfg/dcu_compiler.h",
            "encos_interface.h": "firmware/generated/encos/encos_interface.h",
            "digital_ioObjects.h": "firmware/generated/ethercat/digital_ioObjects.h",
            "gatewayObjects.h": "firmware/generated/ethercat/gatewayObjects.h",
            "eeprom.h": "firmware/generated/ethercat/eeprom.h",
            "can_frame.h": "firmware/ports/can_frame.h",
            "motor_driver.hpp": "firmware/devices/motor/motor_driver.hpp",
            "imu_device.hpp": "firmware/devices/imu/imu_device.hpp",
        }
        roots = (ROOT / "firmware", ROOT / "third_party/beckhoff_ssc/Src")
        for name, owner in owners.items():
            with self.subTest(name=name):
                candidates = [file for root in roots for file in root.rglob(name)]
                self.assertEqual(candidates, [ROOT / owner])

    def test_sdk_and_external_board_resources_exist(self):
        self.assertEqual(dcu.SDK, ROOT / "third_party/hpm_sdk")
        for file in (
            dcu.SDK / "CMakeLists.txt",
            dcu.SDK / "boards/openocd/soc/hpm5e00.cfg",
            dcu.SDK / "soc/HPM5E00/HPM5E31/toolchains/gcc/flash_xip.ld",
            ROOT / "firmware/bsp/dcu_hpm5e31/dcu_hpm5e31.yaml",
            ROOT / "firmware/app/main.c",
        ):
            with self.subTest(file=file):
                self.assertTrue(file.is_file())
        self.assertFalse((dcu.SDK / "boards/hpm5e00evk").exists())

    def test_documentation_paths_use_chinese_names(self):
        for file in (ROOT / "docs").rglob("*"):
            relative = file.relative_to(ROOT / "docs")
            with self.subTest(path=relative):
                self.assertTrue(any("\u4e00" <= character <= "\u9fff" for character in file.name))

    def test_rtos_and_timer_setup_have_explicit_owners(self):
        cycle = (ROOT / "firmware/app/cycle/ethercat_cycle.c").read_text()
        runtime = (ROOT / "firmware/app/runtime/ethercat_can_task.c").read_text()
        hardware = (ROOT / "firmware/bsp/dcu_hpm5e31/cycle_timers.c").read_text()
        for token in ("FreeRTOS.h", "xTaskCreateStatic", "gptmr_channel_config"):
            self.assertNotIn(token, cycle)
        self.assertIn("xTaskCreateStatic", runtime)
        self.assertNotIn("gptmr_channel_config", runtime)
        self.assertIn("gptmr_channel_config", hardware)

    def test_renamed_modules_have_no_retired_source_copies(self):
        modules = (
            ("firmware/app/motor", "motor_control", "motor_application", ("cpp", "hpp")),
            ("firmware/app/motor", "bus_scheduler", "can_bus_scheduler", ("cpp", "hpp")),
            ("firmware/app/motor", "parameter_transactions", "encos_parameter_transactions", ("cpp", "hpp")),
            ("firmware/app/motor/cfg", "motor_control", "motor_policy", ("hpp",)),
            ("firmware/app/cycle", "cycle_runtime", "ethercat_cycle", ("c", "h")),
            ("firmware/app/cycle/cfg", "runtime_config", "ethercat_cycle_policy", ("h",)),
            ("firmware/app/runtime", "app_runtime", "ethercat_can_task", ("c", "h")),
            ("firmware/app/runtime/cfg", "rtos_config", "ethercat_can_task_config", ("h",)),
            ("firmware/bsp/dcu_hpm5e31", "cycle_hardware", "cycle_timers", ("c", "h")),
            ("firmware/bsp/dcu_hpm5e31/cfg", "cycle_board", "cycle_timer_map", ("h",)),
            ("firmware/comm/canfd", "tx_guard", "can_tx_gate", ("c",)),
            ("firmware/comm/canfd", "ethercat_canfd_gateway", "hpm_mcan_transport", ("c", "h")),
            ("firmware/comm/ethercat", "digital_io", "ethercat_callbacks", ("c",)),
        )
        for directory, old, new, extensions in modules:
            for extension in extensions:
                with self.subTest(module=new, extension=extension):
                    self.assertFalse((ROOT / directory / f"{old}.{extension}").exists())
                    self.assertTrue((ROOT / directory / f"{new}.{extension}").is_file())
        for source in ("coeappl.c", "ecatappl.c", "ecatslv.c"):
            text = (ROOT / "third_party/beckhoff_ssc/Src" / source).read_text()
            self.assertIn('#include "digital_io.h"', text)

    def test_motor_policy_and_can_transport_do_not_import_ssc_or_ethercat_cycle(self):
        motor = (ROOT / "firmware/app/motor/motor_application.cpp").read_text()
        can = (ROOT / "firmware/comm/canfd/hpm_mcan_transport.c").read_text()
        for header in ("digital_io.h", "ecatslv.h", "sdoserv.h", "FreeRTOS.h"):
            self.assertNotIn(f'#include "{header}"', motor)
        self.assertNotIn('#include "ethercat_cycle.h"', can)
        self.assertIn('#include "realtime_port.h"', can)

    def test_motor_stateful_operations_have_object_owners_and_interface_tests(self):
        motor = ROOT / "firmware/app/motor"
        implementation = (motor / "detail/motor_application_impl.hpp").read_text()
        self.assertIn("AxisController axes_", implementation)
        self.assertIn("RawCanGateway raw_", implementation)
        self.assertFalse((motor / "detail/motor_application_state.hpp").exists())
        for name in ("motor_application", "axis_controller", "raw_can_gateway"):
            self.assertNotRegex((motor / f"{name}.cpp").read_text(), r"(?m)^static\s+")
        fixture = (ROOT / "tests/host/motor/motor_application_test.cpp").read_text()
        self.assertNotIn('#include "motor_application.cpp"', fixture)
        self.assertNotIn("motor_application_impl.hpp", fixture)

    def test_axis_controller_does_not_access_dictionary_storage(self):
        controller = (ROOT / "firmware/app/motor/axis_controller.cpp").read_text()
        self.assertNotIn("motor_dictionary.hpp", controller)
        self.assertNotRegex(controller, r"\b(?:mc_object|axis_feedback|axis_diagnostics)\s*\(")

    def test_raw_gateway_does_not_depend_on_axis_dictionary_or_pdo_parsing(self):
        for name in ("raw_can_gateway.hpp", "raw_can_gateway.cpp"):
            text = (ROOT / "firmware/app/motor" / name).read_text()
            with self.subTest(name=name):
                for dependency in ("axis_controller.hpp", "motor_dictionary.hpp", "motor_pdo_codec.hpp",
                                   "encos_parameter_transactions.hpp", "AxisController", "mc_channel_config_t",
                                   "mc_raw_t", "canfd_bus_pdo_t", "RawCanCycle", "stage_batch", "stage_full"):
                    self.assertNotIn(dependency, text)

    def test_motor_motion_codec_has_no_retired_wire_api_or_driver_roundtrip(self):
        for path in (ROOT / "firmware").rglob("*"):
            if path.suffix in (".c", ".cpp", ".h", ".hpp"):
                with self.subTest(path=path.relative_to(ROOT)):
                    self.assertNotRegex(path.read_text(), r"\b(?:encos_encode|encos_feedback|motor_command_from_wire)\s*\(")
        for name in (
            "firmware/devices/motor/drivers/encos/encos_driver.cpp",
            "firmware/protocols/encos/encos_motor.cpp",
            "firmware/protocols/encos/encos_motor.hpp",
        ):
            with self.subTest(path=name):
                self.assertNotRegex((ROOT / name).read_text(), r"\bmc_axis_(?:rx|tx)_t\b")

    def test_transport_and_timebase_do_not_own_ethercat_wire_layout(self):
        for name in ("can_transport.h", "hpm_mcan_transport.c", "hpm_mcan_transport.h"):
            text = (ROOT / "firmware/comm/canfd" / name).read_text()
            for token in ("raw_can_pdo.h", "canfd_bus_pdo_t", "ethercat_canfd_process_image_t", "s_now_ms"):
                self.assertNotIn(token, text)
        for path in (ROOT / "firmware").rglob("*"):
            if path.suffix in (".c", ".cpp", ".h", ".hpp"):
                self.assertNotRegex(path.read_text(), r"\b(?:ethercat_canfd_\w+|g_dcu_cycle_diagnostics)\s*\(")
        main = (ROOT / "firmware/app/main.c").read_text()
        self.assertLess(main.index("dcu_timebase_init()"), main.index("dcu_mcan_init()"))

    def test_parameter_core_and_query_codec_do_not_depend_on_dictionary_or_axis(self):
        for relative in ("app/motor/encos_parameter_transactions.cpp", "app/motor/encos_parameter_transactions.hpp",
                         "app/motor/detail/encos_parameter_state.hpp", "protocols/encos/encos_service.h",
                         "protocols/encos/encos_service.cpp"):
            with self.subTest(path=relative):
                text = (ROOT / "firmware" / relative).read_text()
                for dependency in ("motor_dictionary", "encos_interface.h", "motor_application.hpp",
                                   "motor_driver_binding.hpp", "axis_controller.hpp", "axis_object_base", "mc_object"):
                    self.assertNotIn(dependency, text)

    def test_axis_driver_and_protocol_types_are_not_ethercat_dtos(self):
        paths = [ROOT / "firmware/app/motor" / name for name in (
            "axis_contract.hpp", "axis_controller.hpp", "axis_controller.cpp", "detail/axis_state.hpp",
            "motor_driver_binding.hpp", "motor_driver_binding.cpp")]
        paths += list((ROOT / "firmware/devices/motor").rglob("*.hpp"))
        paths += list((ROOT / "firmware/devices/motor").rglob("*.cpp"))
        paths += list((ROOT / "firmware/protocols/encos").rglob("*"))
        for path in paths:
            if path.suffix not in (".h", ".hpp", ".c", ".cpp"):
                continue
            with self.subTest(path=path.relative_to(ROOT)):
                text = path.read_text()
                self.assertNotRegex(text, r"\bmc_\w+\b|\bMC_\w+\b")
                for dependency in ("encos_interface.h", "motor_dictionary", "motor_pdo", "motor_application.hpp"):
                    self.assertNotIn(dependency, text)
        for path in (ROOT / "firmware").rglob("*"):
            if path.suffix in (".h", ".hpp", ".c", ".cpp"):
                self.assertNotRegex(path.read_text(), r"\b(?:encos_profile|encode_motor_command|decode_motor_feedback)\s*\(")


if __name__ == "__main__":
    unittest.main()
