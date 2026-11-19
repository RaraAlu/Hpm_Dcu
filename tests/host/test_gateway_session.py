import copy
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from test_static_gateway_profile import GATEWAY, ROOT, SDK, SSC, TARGET, dcu, header, load_yaml
from test_image_guard import make_elf
from image_guard import FLASH_BASE, FLASH_SIZE, JOURNAL_SIZE, JOURNAL_START, RETAINED_START, RETAINED_SIZE
from prepare_gateway_journal import prepare


JOURNAL_BYTES = 8192
SECTOR_BYTES = 4096
PAGE_BYTES = 256
JOURNAL_MAGIC = 0x31444947
TCLSH = shutil.which("tclsh")


def seed_journal(path, first=(0,), second=()):
    image = bytearray(b"\xff" * JOURNAL_BYTES)
    for sector, epochs in enumerate((first, second)):
        for slot, epoch in enumerate(epochs):
            struct.pack_into("<4I", image, sector * SECTOR_BYTES + slot * PAGE_BYTES,
                             JOURNAL_MAGIC, epoch, epoch ^ 0xffffffff, JOURNAL_MAGIC ^ 0xffffffff)
    path.write_bytes(image)


def simulate_upgrade(directory, journal, *, backup_before_reset, fault=""):
    flash = directory / "full-flash.bin"
    elf = directory / "upgrade.elf"
    script = directory / "transaction.cfg"
    payload = directory / "app-span.bin"
    backup = directory / "retained-eeprom.bin"
    board = json.loads((ROOT / "scripts/cfg/board.example.json").read_text())

    elf.write_bytes(make_elf())
    report = dcu.inspect_image(elf)
    start, end = report["erase_start"], report["erase_end_exclusive"]
    app = bytearray(b"\xff" * (end - start))
    for load in report["loads"]:
        offset = load["start"] - start
        app[offset:offset + load["bytes"]] = bytes(load["bytes"])
    payload.write_bytes(app)

    before = bytearray(b"\xff" * FLASH_SIZE)
    sii = bytes(range(256)) * (RETAINED_SIZE // 256)
    sii_offset, journal_offset = RETAINED_START - FLASH_BASE, JOURNAL_START - FLASH_BASE
    before[sii_offset:sii_offset + RETAINED_SIZE] = sii
    before[journal_offset:journal_offset + JOURNAL_SIZE] = journal
    flash.write_bytes(before)
    script.write_text(dcu.transaction_script(
        elf, board, backup, flash=True, server=True, backup_before_reset=backup_before_reset))
    result = subprocess.run([
        TCLSH, str(ROOT / "tests/host/gateway/flash_upgrade_stub.tcl"), str(script), str(flash), str(elf),
        str(payload), str(FLASH_BASE), str(start), str(JOURNAL_START), fault,
    ], capture_output=True, text=True, timeout=5)
    before[start - FLASH_BASE:end - FLASH_BASE] = app
    return result, flash.read_bytes(), bytes(before), sii


class JournalPreparationTest(unittest.TestCase):
    def test_reserved_layout_matches_firmware(self):
        board = (ROOT / "firmware/bsp/dcu_hpm5e31/board.h").read_text()
        ecat = (ROOT / "firmware/cfg/ecat_config.h").read_text()
        port = (GATEWAY / "cfg/gateway_id_flash_config.h").read_text()
        geometry = (GATEWAY / "cfg/gateway_id_journal_config.h").read_text()

        def hex_value(text, macro):
            return int(re.search(rf"#define\s+{macro}\s+\(?(0x[0-9a-fA-F]+)", text)[1], 16)

        def enum_value(name):
            return int(re.search(rf"\b{name}\s*=\s*(\d+),", geometry)[1])

        self.assertEqual(FLASH_BASE, hex_value(board, "BOARD_FLASH_BASE_ADDRESS"))
        self.assertEqual(RETAINED_START - FLASH_BASE, hex_value(board, "BOARD_ECAT_FLASH_EMULATE_EEPROM_ADDR"))
        self.assertEqual(RETAINED_SIZE, hex_value(ecat, "FLASH_EEPROM_SIZE"))
        self.assertEqual(JOURNAL_START - FLASH_BASE, hex_value(port, "DCU_SESSION_JOURNAL_FLASH_OFFSET"))
        self.assertEqual(JOURNAL_SIZE, enum_value("DCU_ID_SECTORS") * enum_value("DCU_ID_SECTOR_BYTES"))
        self.assertEqual(PAGE_BYTES, enum_value("DCU_ID_PAGE_BYTES"))
        self.assertEqual(struct.calcsize("<4I"), enum_value("DCU_ID_RECORD_BYTES"))
        self.assertEqual(RETAINED_START + RETAINED_SIZE, JOURNAL_START)
        self.assertLessEqual(JOURNAL_START + JOURNAL_SIZE, FLASH_BASE + FLASH_SIZE)

    def test_seed_is_exact_and_manifest_binds_full_bank(self):
        with tempfile.TemporaryDirectory() as directory:
            snapshot = Path(directory) / "journal.bin"
            blank = b"\xff" * JOURNAL_BYTES
            snapshot.write_bytes(blank)
            output = Path(directory) / "plan"
            prepare(snapshot, output)
            seed = (output / "epoch-zero.bin").read_bytes()
            self.assertEqual(struct.unpack("<4I", seed), (JOURNAL_MAGIC, 0, 0xffffffff, JOURNAL_MAGIC ^ 0xffffffff))
            manifest = json.loads((output / "plan.json").read_text())
            self.assertEqual(manifest["required_before_sha256"], hashlib.sha256(blank).hexdigest())
            self.assertEqual(manifest["expected_after_sha256"], hashlib.sha256(seed + blank[len(seed):]).hexdigest())
            self.assertEqual(manifest["program_address"], JOURNAL_START)
            self.assertEqual(manifest["program_bytes"], 16)
            self.assertFalse(manifest["erase_allowed"])
            self.assertEqual(snapshot.read_bytes(), blank)
            with self.assertRaises(FileExistsError):
                prepare(snapshot, output)

    def test_seed_rejects_short_nonblank_or_existing_journal(self):
        with tempfile.TemporaryDirectory() as directory:
            snapshot = Path(directory) / "journal.bin"
            output = Path(directory) / "plan"
            for image in (b"\xff" * (JOURNAL_BYTES - 1), b"\xff" * (JOURNAL_BYTES - 1) + b"\xfe",
                          struct.pack("<4I", JOURNAL_MAGIC, 0, 0xffffffff, JOURNAL_MAGIC ^ 0xffffffff)
                          + b"\xff" * (JOURNAL_BYTES - 16)):
                snapshot.write_bytes(image)
                with self.assertRaises(ValueError):
                    prepare(snapshot, output)
                self.assertFalse(output.exists())
                self.assertEqual(snapshot.read_bytes(), image)

    @unittest.skipUnless(TCLSH, "offline OpenOCD transaction simulation requires tclsh")
    def test_upgrade_aborts_on_missing_backup_or_journal_mismatch(self):
        for fault, message in (("backup_failed", "backup failed"),
                               ("truncated_backup", "Incomplete retained-data backup"),
                               ("reset_changed_journal", "verification failed:"),
                               ("erase_spill", "verification failed:")):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                work = Path(directory)
                journal = work / "current-journal.bin"
                seed_journal(journal, range(4))
                original = journal.read_bytes()
                result, after, _, _ = simulate_upgrade(work, original, backup_before_reset=True, fault=fault)
                self.assertNotEqual(result.returncode, 0)
                self.assertNotIn("DCU_DEBUG_READY", result.stdout)
                self.assertIn(message, result.stderr)
                journal_was_changed = fault in ("reset_changed_journal", "erase_spill")
                if journal_was_changed:
                    self.assertEqual((work / "gateway-id-journal.bin").read_bytes(), original)
                else:
                    self.assertNotIn("DCU_RETAINED_BACKUP_COMPLETE", result.stdout)
                self.assertEqual("APP_WRITE" in result.stdout, fault == "erase_spill")
                # A mismatch aborts the transaction; the old backup is not written back.
                self.assertEqual(after[JOURNAL_START - FLASH_BASE], 0xff if journal_was_changed else original[0])


class GatewaySessionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        directory = tempfile.TemporaryDirectory(prefix="dcu-gateway-session-")
        cls.addClassCleanup(directory.cleanup)
        work = Path(directory.name)
        profile = GATEWAY / "cfg/单路内部回环配置.yaml"
        subprocess.run([
            sys.executable, str(ROOT / "tools/protocol/generate_static_profile.py"),
            "--profile", str(profile), "--output", str(work / "dcu_static_profile_generated.h"),
            "--require-session",
        ], check=True, timeout=30)
        cls.profile = load_yaml(profile)
        flags = ["cc", "-std=gnu11", "-ffunction-sections", "-fdata-sections",
                 "-DDCU_DATA_STATIC_PROFILE=1", "-DDCU_DATA_BACKEND_MCAN=1", "-DDCU_DATA_SESSION_GATE=1",
                 *(f"-I{p}" for p in (work, GATEWAY, TARGET, ROOT / "firmware/ports"))]
        mcan = work / "mcan.o"
        fake = work / "fake.o"
        host = ROOT / "tests/host/ethercat_data"
        for source, output in ((TARGET / "mcan_echo.c", mcan), (host / "test_mcan_echo.c", fake)):
            subprocess.run([
                *flags, "-Wall", "-Wextra", "-Werror", f"-I{ROOT / 'tests/host/can_loopback/stubs'}",
                "-c", str(source), "-o", str(output),
            ], check=True, timeout=30)
        sdk_dirs = ("arch", "arch/riscv/intc", "drivers/inc", "soc/HPM5E00/HPM5E31",
                    "soc/HPM5E00/ip", "components/eeprom_emulation", "components/debug_console")
        includes = (SSC, ROOT / "firmware/cfg", ROOT / "firmware/comm/ethercat/port",
                    ROOT / "firmware/bsp/dcu_hpm5e31", *(SDK / dir for dir in sdk_dirs))
        cls.binary = work / "session"
        subprocess.run([
            *flags, "-Wno-pointer-to-int-cast", *(f"-I{p}" for p in includes),
            str(TARGET / "ssc_app.c"), str(TARGET / "data_contract.c"), str(TARGET / "session_bridge.c"),
            str(SSC / "objdef.c"), str(mcan), str(fake), str(host / "test_session_gate.c"),
            str(host / "session_id_file.c"),
            *(str(GATEWAY / source) for source in (
                "static_profile.c", "gateway_readiness.c", "gateway_task.c", "gateway_identity.c")),
            "-Wl,--gc-sections,--wrap=OBJ_Write", "-o", str(cls.binary),
        ], check=True, timeout=30)

    def run_scenarios(self, *names):
        for name in names:
            with self.subTest(scenario=name), tempfile.TemporaryDirectory() as directory:
                journal = Path(directory) / "flash.bin"
                seed_journal(journal)
                self.run_boot(name, journal)

    def run_boot(self, scenario, journal, *, fault="", expected_exit=0, replay=""):
        env = {**os.environ, "DCU_TEST_ID_JOURNAL": str(journal), "DCU_TEST_ID_FAULT": fault,
               "DCU_TEST_REPLAY": str(replay)}
        result = subprocess.run([str(self.binary), scenario], cwd=self.binary.parent, env=env,
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, expected_exit, result.stdout + result.stderr)
        return result.stdout

    def allocate(self, journal, fault=""):
        return tuple(map(int, self.run_boot("allocate", journal, fault=fault).split()))

    def test_autonomous_readiness_requires_actual_matched_can_echo(self):
        self.run_scenarios("missing_probe", "wrong_probe")

    def test_sdo_activation_and_op_are_independently_required_then_pdo_and_can_run(self):
        self.run_scenarios("confirmation_first", "op_first")

    def test_real_process_restart_rejects_previous_boot_sdo_and_pdo(self):
        with tempfile.TemporaryDirectory() as directory:
            journal, replay = (Path(directory) / name for name in ("flash.bin", "requests.bin"))
            seed_journal(journal)
            self.run_boot("record_previous", journal, replay=replay)
            self.run_boot("replay_after_reset", journal, replay=replay)
            self.assertEqual(self.allocate(journal), (3 << 32, (3 << 32) | 1, (3 << 32) | 2))

    @unittest.skipUnless(TCLSH, "offline OpenOCD transaction simulation requires tclsh")
    def test_firmware_upgrade_preserves_numbering_and_rejects_old_traffic(self):
        for pre_reset in (False, True):
            with self.subTest(backup_before_reset=pre_reset), tempfile.TemporaryDirectory() as directory:
                work = Path(directory)
                journal = work / "current-journal.bin"
                replay = work / "requests.bin"
                blank = b"\xff" * JOURNAL_SIZE
                journal.write_bytes(blank)
                prepare(journal, work / "plan")
                seed = (work / "plan/epoch-zero.bin").read_bytes()
                journal.write_bytes(seed + blank[len(seed):])
                self.run_boot("record_previous", journal, replay=replay)
                before = journal.read_bytes()
                result, after, expected, sii = simulate_upgrade(work, before, backup_before_reset=pre_reset)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertIn("APP_WRITE\nDCU_DEBUG_READY", result.stdout)
                self.assertEqual(after, expected)
                offset = JOURNAL_START - FLASH_BASE
                self.assertEqual(after[offset:offset + JOURNAL_SIZE], before)
                self.assertEqual((work / "gateway-id-journal.bin").read_bytes(), before)
                self.assertEqual((work / "retained-eeprom.bin").read_bytes(), sii)
                journal.write_bytes(after[offset:offset + JOURNAL_SIZE])
                self.run_boot("replay_after_reset", journal, replay=replay)
                self.assertEqual(self.allocate(journal), (3 << 32, (3 << 32) | 1, (3 << 32) | 2))

    def test_persistent_ids_increase_through_sector_rollover(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory) / "flash.bin"
            seed_journal(journal)
            for epoch in range(1, 37):
                with self.subTest(epoch=epoch):
                    boot = epoch << 32
                    self.assertEqual(self.allocate(journal), (boot, boot | 1, boot | 2))

    def test_power_cut_before_and_after_commit_never_reissues_published_id(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory) / "flash.bin"
            seed_journal(journal)
            self.assertEqual(self.allocate(journal)[0], 1 << 32)
            self.run_boot("allocate", journal, fault="torn_program", expected_exit=90)
            self.assertEqual(self.allocate(journal)[0], 2 << 32)
            self.run_boot("allocate", journal, fault="committed_then_reset", expected_exit=90)
            self.assertEqual(self.allocate(journal)[0], 4 << 32)
            self.assertEqual(self.allocate(journal, "readback_failed"), (0, 0, 0))
            self.assertEqual(self.allocate(journal)[0], 6 << 32)

    def test_power_cut_on_rollover_never_erases_the_active_sector(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory) / "flash.bin"
            seed_journal(journal, range(16), range(16, 32))
            active = journal.read_bytes()[SECTOR_BYTES:]
            self.run_boot("allocate", journal, fault="torn_erase", expected_exit=90)
            self.assertEqual(journal.read_bytes()[SECTOR_BYTES:], active)
            self.assertEqual(self.allocate(journal)[0], 32 << 32)
            self.assertEqual(journal.read_bytes()[SECTOR_BYTES:], active)

    def test_unprovisioned_or_exhausted_journal_fails_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            journal = Path(directory) / "flash.bin"
            for epochs in ((), (0xffffffff,)):
                with self.subTest(epochs=epochs):
                    seed_journal(journal, epochs)
                    previous = journal.read_bytes()
                    self.assertEqual(self.allocate(journal), (0, 0, 0))
                    self.run_boot("unprovisioned", journal)
                    self.assertEqual(journal.read_bytes(), previous)

    def test_readiness_unconsumed_activation_and_idle_master_expire(self):
        self.run_scenarios("ready_timeout", "activation_timeout", "command_timeout")

    def test_digest_binds_route_and_mapping_and_bad_budget_is_rejected(self):
        def digest(profile):
            return next(line for line in header(profile, require_session=True).splitlines()
                        if line.startswith("#define DCU_PROFILE_DIGEST "))
        expected = digest(self.profile)
        for field in ("route", "mapping"):
            profile = copy.deepcopy(self.profile)
            if field == "route":
                profile["channels"][0]["devices"][0]["can_id_base"] += 1
            else:
                profile["pdo"]["rx"]["mapping"] += 1
            self.assertNotEqual(digest(profile), expected)
        profile = copy.deepcopy(self.profile)
        profile["session"]["readiness_ttl_ms"] = 10
        with self.assertRaises(ValueError):
            header(profile, require_session=True)
        profile.pop("session")
        with self.assertRaises(ValueError):
            header(profile, require_session=True)

    def test_no_target_operation_for_session_preset(self):
        with mock.patch.object(dcu, "target_operation") as target, mock.patch("sys.stderr"):
            for command in ("flash", "debug-server", "verify"):
                with self.assertRaises(SystemExit):
                    dcu.main([command, "--preset", "hpm-gateway-session", "--confirm-target", "RESET_TARGET"])
            target.assert_not_called()
