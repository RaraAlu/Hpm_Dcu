import json
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "tests/target/ethercat_data"


class EthercatMcanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        directory = tempfile.TemporaryDirectory(prefix="dcu-ethercat-mcan-")
        cls.addClassCleanup(directory.cleanup)
        cls.binary = Path(directory.name) / "mcan-echo"
        subprocess.run([
            "cc", "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
            "-DDCU_DATA_BACKEND_MCAN=1", f"-I{TARGET}", f"-I{ROOT / 'firmware/ports'}",
            f"-I{ROOT / 'tests/host/can_loopback/stubs'}",
            str(TARGET / "data_contract.c"), str(TARGET / "mcan_echo.c"),
            str(ROOT / "tests/host/ethercat_data/test_mcan_echo.c"), "-o", str(cls.binary),
        ], check=True, timeout=30)

    def run_scenarios(self, *names):
        for name in names:
            with self.subTest(scenario=name):
                subprocess.run([str(self.binary), name], check=True, timeout=5, cwd=self.binary.parent)

    def test_four_frames_require_both_tx_and_rx_without_replacing_inflight_payload(self):
        self.run_scenarios("exchange")

    def test_leaving_op_discards_old_inflight_transfer(self):
        self.run_scenarios("leave_op")

    def test_does_not_publish_invalid_missing_or_duplicate_frames(self):
        self.run_scenarios("bad_id", "bad_data", "fd", "duplicate", "no_tx", "error", "mode_lost")

    def test_internal_echo_is_opt_in_and_has_its_own_preset(self):
        configs = json.loads((ROOT / "cmake/cfg/CMakePresets.json").read_text())["configurePresets"]
        presets = {entry["name"]: entry for entry in configs}
        self.assertFalse(presets["hpm-base"]["cacheVariables"]["CONFIG_ETHERCAT_MCAN_ECHO"])
        self.assertEqual(presets["hpm-ethercat-mcan"]["inherits"], "hpm-ethercat-data")
        self.assertTrue(presets["hpm-ethercat-mcan"]["cacheVariables"]["CONFIG_ETHERCAT_MCAN_ECHO"])
