import ctypes
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "tests/target/ethercat_data"
SDK = ROOT / "third_party/hpm_sdk"
SSC = ROOT / "third_party/beckhoff_ssc/Src"
sys.path.insert(0, str(ROOT / "scripts"))
import dcu


class EthercatDataTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compiler = shutil.which("cc")
        if not cls.compiler:
            raise RuntimeError("EtherCAT data tests require a host C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-ethercat-data-")
        cls.addClassCleanup(directory.cleanup)
        cls.directory = Path(directory.name)
        library = cls.directory / "data.so"
        subprocess.run([
            cls.compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-fPIC", "-shared", str(TARGET / "data_contract.c"), "-o", str(library),
        ], check=True, timeout=30)
        cls.api = ctypes.CDLL(str(library))
        cls.api.dcu_data_sii_read.argtypes = [ctypes.c_uint32, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint64)]
        cls.api.dcu_data_sii_read.restype = ctypes.c_bool
        cls.api.dcu_data_reset.restype = None
        for name in ("dcu_data_capture", "dcu_data_copy"):
            func = getattr(cls.api, name)
            func.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
            func.restype = ctypes.c_bool

    def sii(self, width):
        chunks = []
        for offset in range(0, 512, width):
            data = ctypes.c_uint64()
            self.assertTrue(self.api.dcu_data_sii_read(offset // 2, width, ctypes.byref(data)))
            chunks.append(data.value.to_bytes(width, "little"))
        return b"".join(chunks)

    def test_one_cycle_timing_rejects_stale_late_and_invalid_feedback(self):
        cxx = shutil.which("c++")
        self.assertIsNotNone(cxx)
        executable = self.directory / "one_cycle_pdo"
        bench = ("-DDCU_TX_PUBLISH_ADVANCE_US=399", "-DDCU_ONE_CYCLE_SYNC0_PHASE_US=255",
                 "-DDCU_ONE_CYCLE_SM3_READ_PHASE_US=870")
        prep = (*bench, "-DDCU_ONE_CYCLE_SM3_PREPARE_US=50")
        traced = (*prep, "-DDCU_ONE_CYCLE_SOF_TRACE=1", "-DDCU_ONE_CYCLE_ENDURANCE=1")
        for defines in ((), bench, prep, traced, (*traced, "-DDCU_ONE_CYCLE_SM2_PREPARE_US=0"),
                        (*traced, "-DDCU_ONE_CYCLE_SM2_PREPARE_US=50"),
                        (*traced, "-DDCU_ONE_CYCLE_SM2_PREPARE_US=50", "-DDCU_ONE_CYCLE_COLLECT_TRACE=1")):
            with self.subTest(defines=defines):
                subprocess.run([
                    cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic-errors", *defines,
                    "-I", str(ROOT / "firmware/cfg"),
                    str(ROOT / "tests/host/ethercat_data/test_one_cycle_pdo.cpp"), "-o", str(executable),
                ], check=True, timeout=30)
                subprocess.run([str(executable)], check=True, timeout=30)

    def test_sii_has_exact_coe_sync_manager_and_pdo_layout(self):
        image = self.sii(4)
        self.assertEqual(image, self.sii(8))
        self.assertEqual(image[:16], bytes.fromhex("80 0c 81 66") + bytes(10) + b"\x62\x00")
        self.assertEqual(struct.unpack_from("<IIII", image, 16), (0x10000, 0x7fff0002, 1, 0))
        self.assertEqual(struct.unpack_from("<5H", image, 48), (0x1000, 128, 0x1080, 128, 4))
        self.assertEqual(struct.unpack_from("<2H", image, 124), (3, 1))
        categories = {}
        pos = 128
        while pos < len(image):
            kind, words = struct.unpack_from("<HH", image, pos)
            if kind == 0xffff:
                break
            self.assertNotIn(kind, categories)
            pos += 4
            categories[kind] = image[pos:pos + 2 * words]
            pos += 2 * words
        self.assertEqual(kind, 0xffff)
        self.assertEqual(list(categories), [30, 40, 41, 50, 51])
        self.assertEqual(len(categories[30]), 32)
        self.assertEqual(categories[30][5], 0x13)
        self.assertEqual(categories[40], bytes((1, 2, 3, 255)))
        self.assertEqual(list(struct.iter_unpack("<HHBBBB", categories[41])), [
            (0x1000, 128, 0x26, 0, 1, 1), (0x1080, 128, 0x22, 0, 1, 2),
            (0x1100, 32, 0x64, 0, 1, 3), (0x1200, 48, 0x20, 0, 1, 4),
        ])
        for kind, index, count, sm, obj in ((50, 0x1a00, 12, 3, 0x6000), (51, 0x1600, 8, 2, 0x7000)):
            pdo = categories[kind]
            self.assertEqual(struct.unpack_from("<HBB4x", pdo), (index, count, sm))
            self.assertEqual(list(struct.iter_unpack("<HBBBBH", pdo[8:])), [
                (obj, sub, 0, 7, 32, 0) for sub in range(1, count + 1)
            ])

    def test_sii_read_rejects_out_of_range_without_writing_destination(self):
        data = ctypes.c_uint64(123)
        for addr, width in ((255, 4), (253, 8), (0xffffffff, 8), (0, 2), (0, 0)):
            self.assertFalse(self.api.dcu_data_sii_read(addr, width, ctypes.byref(data)))
            self.assertEqual(data.value, 123)
        self.assertFalse(self.api.dcu_data_sii_read(0, 4, None))

    def test_pdo_is_byte_exact_with_length_guards_and_session_reset(self):
        self.api.dcu_data_reset()
        tx = ctypes.create_string_buffer(b"\x55" * 50, 50)
        snapshot = ctypes.byref(tx, 1)
        self.assertFalse(self.api.dcu_data_copy(snapshot, 47))
        self.assertEqual(tx.raw, b"\x55" * 50)
        self.assertTrue(self.api.dcu_data_copy(snapshot, 48))
        self.assertEqual(tx.raw[1:49], bytes(32) + struct.pack("<4I", 0, 0, 1, 0))
        for count, pattern in enumerate((bytes(range(32)), bytes(32), b"\xff" * 32), 1):
            self.assertTrue(self.api.dcu_data_capture(pattern, 32))
            self.assertFalse(self.api.dcu_data_capture(pattern, 31))
            self.assertFalse(self.api.dcu_data_capture(None, 32))
            self.assertTrue(self.api.dcu_data_copy(snapshot, 48))
            self.assertEqual(tx.raw, b"\x55" + pattern + struct.pack("<4I", count, 32, 1, 1) + b"\x55")
        self.api.dcu_data_reset()
        self.assertTrue(self.api.dcu_data_copy(snapshot, 48))
        self.assertEqual(tx.raw[1:49], bytes(32) + struct.pack("<4I", 0, 0, 1, 0))

    def test_stock_ssc_reads_the_diagnostic_dictionary_and_mapping(self):
        test = self.directory / "ssc-layout"
        includes = [
            TARGET, SSC, ROOT / "firmware/cfg", ROOT / "firmware/comm/ethercat/port",
            ROOT / "firmware/bsp/dcu_hpm5e31",
            *(SDK / part for part in ("arch", "arch/riscv/intc", "drivers/inc", "soc/HPM5E00/HPM5E31",
                                      "soc/HPM5E00/ip", "components/eeprom_emulation", "components/debug_console")),
        ]
        subprocess.run([
            self.compiler, "-std=gnu11", "-ffunction-sections", "-fdata-sections", "-Wno-pointer-to-int-cast",
            *(f"-I{path}" for path in includes),
            str(TARGET / "ssc_app.c"), str(TARGET / "data_contract.c"), str(SSC / "objdef.c"),
            str(Path(__file__).parent / "ethercat_data/test_ssc_layout.c"),
            "-Wl,--gc-sections,--wrap=OBJ_Write", "-o", str(test),
        ], check=True, timeout=30)
        subprocess.run([str(test)], check=True, timeout=5, cwd=self.directory)

    def test_test_image_is_opt_in_and_debug_access_is_guarded(self):
        presets = json.loads((ROOT / "cmake/cfg/CMakePresets.json").read_text())
        configs = {entry["name"]: entry["cacheVariables"] for entry in presets["configurePresets"]}
        self.assertFalse(configs["hpm-base"]["CONFIG_ETHERCAT_DATA_TEST"])
        self.assertTrue(configs["hpm-ethercat-data"]["CONFIG_ETHERCAT_DATA_TEST"])
        self.assertNotIn("hpm-ethercat-data", dcu.PRESETS)
        with mock.patch.dict(dcu.os.environ, {}, clear=True), mock.patch.object(dcu, "require_board") as board:
            self.assertEqual(dcu.main(["debug-server", "--preset", "hpm-ethercat-data", "--flash-first"]), 1)
            board.assert_not_called()
