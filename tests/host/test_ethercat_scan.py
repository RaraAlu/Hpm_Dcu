import ctypes
import json
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "tests/target/ethercat_scan"
sys.path.insert(0, str(ROOT / "scripts"))
import dcu


class EthercatScanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("cc")
        if not compiler:
            raise RuntimeError("EtherCAT scan tests require a host C compiler")
        directory = tempfile.TemporaryDirectory(prefix="dcu-ethercat-scan-")
        cls.addClassCleanup(directory.cleanup)
        library = Path(directory.name) / "scan.so"
        subprocess.run([
            compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-fPIC", "-shared", str(TARGET / "scan_contract.c"), "-o", str(library),
        ], check=True, timeout=30)
        cls.api = ctypes.CDLL(str(library))
        cls.api.dcu_scan_sii_read.argtypes = [ctypes.c_uint32, ctypes.c_uint, ctypes.POINTER(ctypes.c_uint64)]
        cls.api.dcu_scan_sii_read.restype = ctypes.c_bool
        cls.api.dcu_scan_al_status.argtypes = [ctypes.c_uint16, ctypes.c_uint16, ctypes.POINTER(ctypes.c_uint16)]
        cls.api.dcu_scan_al_status.restype = ctypes.c_uint16

    def sii(self, size):
        chunks = []
        for offset in range(0, 256, size):
            data = ctypes.c_uint64()
            self.assertTrue(self.api.dcu_scan_sii_read(offset // 2, size, ctypes.byref(data)))
            chunks.append(data.value.to_bytes(size, "little"))
        return b"".join(chunks)

    def test_four_and_eight_byte_reads_match_and_bounds_are_checked(self):
        self.assertEqual(self.sii(4), self.sii(8))
        data = ctypes.c_uint64(123)
        for addr, size in ((127, 4), (125, 8), (0xffffffff, 8), (0, 2), (0, 0)):
            self.assertFalse(self.api.dcu_scan_sii_read(addr, size, ctypes.byref(data)))
            self.assertEqual(data.value, 123)
        self.assertFalse(self.api.dcu_scan_sii_read(0, 4, None))

    def test_sii_board_configuration_crc_and_nonproduction_identity(self):
        image = self.sii(4)
        generated = (ROOT / "firmware/generated/ethercat/eeprom.h").read_text()
        config = bytes(int(x, 16) for x in re.findall(r"0x[0-9a-fA-F]{2}", generated)[:16])
        self.assertEqual(image[:16], config)
        crc = 0xff
        for byte in image[:14]:
            crc ^= byte
            for _ in range(8):
                crc = ((crc << 1) ^ (7 if crc & 0x80 else 0)) & 0xff
        self.assertEqual(image[14], crc)
        self.assertEqual(struct.unpack_from("<IIII", image, 16), (0x10000, 0x7fff0001, 1, 0))
        capacity, version = struct.unpack_from("<HH", image, 124)
        self.assertEqual(((capacity + 1) * 128, version), (len(image), 1))

    def test_sii_does_not_advertise_mailboxes_or_mapped_pdos(self):
        image = self.sii(4)
        self.assertEqual(image[40:58], bytes(18))
        self.assertEqual(struct.unpack_from("<HH", image, 128), (41, 16))
        for index in range(4):
            address, size, ctl, status, enabled, kind = struct.unpack_from("<HHBBBB", image, 132 + index * 8)
            self.assertEqual((address, size, status, enabled), (0, 0, 0, 0))
            self.assertEqual(ctl & 4, 4 if index == 2 else 0)
            self.assertEqual(kind, (0, 0, 3, 4)[index])
        self.assertEqual(image[164:166], b"\xff\xff")

    def test_al_never_enters_safeoperational_or_operational(self):
        code = ctypes.c_uint16()
        for requested in (0, 3, 4, 8, 15, 0x18):
            for state in (1, 2):
                self.assertEqual(self.api.dcu_scan_al_status(state, requested, ctypes.byref(code)), 0x10 | state)
                self.assertEqual(code.value, 0x11)
        for state, request, expected in ((1, 2, 2), (2, 1, 1), (0x11, 2, 0x11), (0x11, 0x12, 2)):
            self.assertEqual(self.api.dcu_scan_al_status(state, request, ctypes.byref(code)), expected)

    def test_scan_is_opt_in_and_target_operations_still_require_confirmation(self):
        presets = json.loads((ROOT / "cmake/cfg/CMakePresets.json").read_text())
        configs = {entry["name"]: entry["cacheVariables"] for entry in presets["configurePresets"]}
        self.assertFalse(configs["hpm-base"]["CONFIG_ETHERCAT_SCAN_TEST"])
        self.assertTrue(configs["hpm-ethercat-scan"]["CONFIG_ETHERCAT_SCAN_TEST"])
        self.assertNotIn("hpm-ethercat-scan", dcu.PRESETS)
        with mock.patch.dict(dcu.os.environ, {}, clear=True), mock.patch.object(dcu, "require_board") as board:
            self.assertEqual(dcu.main(["debug-server", "--preset", "hpm-ethercat-scan", "--flash-first"]), 1)
            board.assert_not_called()


if __name__ == "__main__":
    unittest.main()
