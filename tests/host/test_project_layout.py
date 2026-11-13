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


if __name__ == "__main__":
    unittest.main()
