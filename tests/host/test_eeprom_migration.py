import os
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]


class EepromMigrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        directory = tempfile.TemporaryDirectory(prefix='dcu-eeprom-')
        cls.addClassCleanup(directory.cleanup)
        cls.executable = Path(directory.name) / 'eeprom_test'
        flags = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic-errors']
        if os.environ.get('DCU_HOST_SANITIZE') == '1':
            flags += ['-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie']
        for path in ('tests/host/eeprom/stubs', 'tests/host/motor/stubs', 'firmware/cfg',
                     'firmware/comm/ethercat/port', 'firmware/generated/ethercat',
                     'third_party/beckhoff_ssc/Src', 'third_party/hpm_sdk/components/eeprom_emulation',
                     'third_party/hpm_sdk/components/eeprom_emulation/port'):
            flags += ['-I', str(ROOT / path)]
        cls.run_command(['cc', *flags, '-include', str(ROOT / 'tests/host/eeprom/stubs/fixture.h'),
                         str(ROOT / 'tests/host/eeprom/eeprom_test.c'),
                         str(ROOT / 'firmware/comm/ethercat/port/hpm_ecat_e2p_emulation.c'),
                         '-o', str(cls.executable)])

    @staticmethod
    def run_command(command):
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise AssertionError(f"Command failed: {' '.join(command)}\n{result.stdout}{result.stderr}")

    def test_fresh_image_commits_identity_page_last(self):
        self.run_command([str(self.executable), 'fresh'])

    def test_regeneration_retains_alias_serial_and_recomputes_crc(self):
        self.run_command([str(self.executable), 'update'])

    def test_legacy_word_migration_retains_identity(self):
        self.run_command([str(self.executable), 'legacy'])

    def test_failed_page_write_cannot_commit_a_partial_image(self):
        self.run_command([str(self.executable), 'failure'])

    def test_runtime_bounds_op_gate_and_deferred_flush_retry(self):
        self.run_command([str(self.executable), 'io'])


if __name__ == '__main__':
    unittest.main()
