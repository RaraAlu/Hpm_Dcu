import copy
import ctypes
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from test_ethercat_data import ROOT, SDK, SSC, TARGET, dcu

ALTERNATE = Path(__file__).parent / "gateway/alternate_static_profile.yaml"
sys.path.insert(0, str(ROOT / "tools/protocol"))
from generate_static_profile import header, load_yaml, validate


class StaticGatewayProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        directory = tempfile.TemporaryDirectory(prefix="dcu-static-profile-")
        cls.addClassCleanup(directory.cleanup)
        cls.directory = Path(directory.name)
        cls.profile = load_yaml(ALTERNATE)
        subprocess.run([
            sys.executable, str(ROOT / "tools/protocol/generate_static_profile.py"),
            "--profile", str(ALTERNATE), "--output", str(cls.directory / "dcu_static_profile_generated.h"),
        ], check=True, timeout=30)

    def test_rejects_invalid_resources_and_mapping(self):
        for path, invalid in (
            (("channels",), []),
            (("channels",), self.profile["channels"] * 2),
            (("channels", 0, "bus"), 4),
            (("channels", 0, "bus"), True),
            (("channels", 0, "mode"), "normal"),
            (("channels", 0, "devices"), []),
            (("channels", 0, "devices", 0, "protocol"), "motor"),
            (("channels", 0, "devices", 0, "can_id_base"), 0x7ff),
            (("pdo", "rx", "words"), 3),
            (("pdo", "rx", "words"), 10),
            (("pdo", "rx", "mapping"), 0x1a00),
            (("pdo", "tx", "object"), 0x7000),
            (("frame_timeout_ms",), 0),
        ):
            with self.subTest(path=path, invalid=invalid):
                profile = copy.deepcopy(self.profile)
                node = profile
                for key in path[:-1]:
                    node = node[key]
                node[path[-1]] = invalid
                with self.assertRaises(ValueError):
                    validate(profile)

    def test_header_is_deterministic_and_derived(self):
        result = (self.directory / "dcu_static_profile_generated.h").read_text()
        self.assertEqual(result, header(self.profile))
        self.assertIn("DCU_PROFILE_CAN_BUS 2U", result)
        self.assertIn("DCU_PROFILE_CAN_ID_BASE 0x650U", result)
        self.assertIn("DCU_PROFILE_RX_ENTRIES(X) X(1) X(2) X(3) X(4)\n", result)
        self.assertIn("DCU_PROFILE_TX_WORDS 8U", result)

    def test_firmware_adapter_only_enables_selected_channel_and_uses_profile_route(self):
        runner = self.directory / "profile-mcan"
        subprocess.run([
            "cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-DDCU_DATA_STATIC_PROFILE=1", "-DDCU_DATA_BACKEND_MCAN=1", "-DTEST_ALTERNATE_PROFILE=1",
            *(f"-I{p}" for p in (self.directory, TARGET, ROOT / "firmware/ports",
                                  ROOT / "tests/host/can_loopback/stubs")),
            str(TARGET / "data_contract.c"), str(TARGET / "mcan_echo.c"), str(TARGET / "static_profile.c"),
            str(ROOT / "tests/host/ethercat_data/test_mcan_echo.c"), "-o", str(runner),
        ], check=True, timeout=30)
        for scenario in ("exchange", "bad_id", "leave_op", "no_tx"):
            with self.subTest(scenario=scenario):
                subprocess.run([str(runner), scenario], check=True, timeout=5, cwd=self.directory)

    def test_generated_sizes_and_mapping_reach_sii_and_stock_object_dictionary(self):
        library = self.directory / "profile-sii.so"
        subprocess.run([
            "cc", "-std=c11", "-DDCU_DATA_STATIC_PROFILE=1", f"-I{self.directory}",
            "-fPIC", "-shared", str(TARGET / "data_contract.c"), "-o", str(library),
        ], check=True, timeout=30)
        read = ctypes.CDLL(str(library)).dcu_data_sii_read
        read.argtypes = [ctypes.c_uint32, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint64)]
        read.restype = ctypes.c_bool
        image = bytearray()
        data = ctypes.c_uint64()
        for word in range(0, 256, 4):
            self.assertTrue(read(word, 8, ctypes.byref(data)))
            image.extend(data.value.to_bytes(8, "little"))
        self.assertEqual(struct.unpack_from("<I", image, 20)[0], 0x7fff0004)
        categories = {}
        pos = 128
        while pos + 4 <= len(image):
            kind, words = struct.unpack_from("<HH", image, pos)
            if kind == 0xffff:
                break
            pos += 4
            categories[kind] = image[pos:pos + 2 * words]
            pos += 2 * words
        self.assertEqual(kind, 0xffff)
        self.assertEqual(struct.unpack_from("<HH", categories[41], 16), (0x1100, 16))
        self.assertEqual(struct.unpack_from("<HH", categories[41], 24), (0x1200, 32))
        for kind, map_id, count, sm, obj in ((50, 0x1a07, 8, 3, 0x6009), (51, 0x1603, 4, 2, 0x7005)):
            pdo = categories[kind]
            self.assertEqual(struct.unpack_from("<HBB", pdo), (map_id, count, sm))
            self.assertEqual(list(struct.iter_unpack("<HBBBBH", pdo[8:])), [
                (obj, sub, 0, 7, 32, 0) for sub in range(1, count + 1)
            ])
        runner = self.directory / "profile-ssc"
        sdk_dirs = ("arch", "arch/riscv/intc", "drivers/inc", "soc/HPM5E00/HPM5E31",
                    "soc/HPM5E00/ip", "components/eeprom_emulation", "components/debug_console")
        includes = (self.directory, TARGET, SSC, ROOT / "firmware/cfg", ROOT / "firmware/comm/ethercat/port",
                    ROOT / "firmware/bsp/dcu_hpm5e31", *(SDK / part for part in sdk_dirs))
        subprocess.run([
            "cc", "-std=gnu11", "-ffunction-sections", "-fdata-sections", "-Wno-pointer-to-int-cast",
            "-DDCU_DATA_STATIC_PROFILE=1", "-DTEST_ALTERNATE_PROFILE=1",
            *(f"-I{p}" for p in includes),
            str(TARGET / "ssc_app.c"), str(TARGET / "data_contract.c"), str(SSC / "objdef.c"),
            str(ROOT / "tests/host/ethercat_data/test_ssc_layout.c"),
            "-Wl,--gc-sections,--wrap=OBJ_Write", "-o", str(runner),
        ], check=True, timeout=30)
        subprocess.run([str(runner)], check=True, timeout=5, cwd=self.directory)

    def test_offline_preset_cannot_enter_target_operations(self):
        with mock.patch.object(dcu, "target_operation") as target:
            for action in ("flash", "verify", "debug-server"):
                with self.subTest(action=action), mock.patch("sys.stderr"):
                    with self.assertRaises(SystemExit):
                        dcu.main([action, "--preset", "hpm-gateway-profile", "--confirm-target", "RESET_TARGET"])
            target.assert_not_called()
