from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class Cih408MitOnceTest(unittest.TestCase):
    def test_offline_pdo_can_reply_pdo_round_trip(self):
        with tempfile.TemporaryDirectory(prefix="dcu-cih408-mit-once-") as directory:
            binary = Path(directory) / "mit-once"
            subprocess.run([
                "c++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror", "-pedantic-errors",
                f"-I{ROOT / 'firmware/ports'}",
                f"-I{ROOT / 'firmware/comm/canfd'}",
                str(ROOT / "tests/host/ethercat_data/test_cih408_mit_once.cpp"),
                "-o", str(binary),
            ], check=True, timeout=30)
            subprocess.run([str(binary)], check=True, cwd=directory, timeout=5)
