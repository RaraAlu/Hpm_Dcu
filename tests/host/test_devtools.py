import contextlib
import io
import json
import os
from types import SimpleNamespace
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import dcu


class DevtoolsTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.board_path = Path(self.directory.name) / "board.json"
        self.board = json.loads((ROOT / "scripts/cfg/board.example.json").read_text())

    def write_board(self):
        self.board_path.write_text(json.dumps(self.board))

    def test_example_board_cannot_authorize_target_access(self):
        self.write_board()
        with self.assertRaisesRegex(ValueError, "explicitly confirmed"):
            dcu.require_board(self.board_path)

    def test_reviewed_board_requires_real_reset_connection(self):
        self.board["board_verified"] = True
        self.write_board()
        with self.assertRaises(ValueError):
            dcu.require_board(self.board_path)

    def test_reviewed_configuration_is_accepted_offline(self):
        self.board.update(board_verified=True, srst_connected=True)
        self.write_board()
        self.assertEqual(dcu.require_board(self.board_path)["flash_size_bytes"], 0x100000)

    def test_tcl_injection_in_options_is_rejected(self):
        self.board.update(board_verified=True, srst_connected=True, flash_opt0="0x5; reset run")
        self.write_board()
        with self.assertRaisesRegex(ValueError, "hexadecimal"):
            dcu.require_board(self.board_path)

    def test_wrong_geometry_is_rejected(self):
        self.board.update(board_verified=True, srst_connected=True, flash_size_bytes=0x2000000)
        self.write_board()
        with self.assertRaisesRegex(ValueError, "capacity"):
            dcu.require_board(self.board_path)

    def test_non_object_board_is_rejected(self):
        self.board_path.write_text("[]")
        with self.assertRaisesRegex(ValueError, "JSON object"):
            dcu.require_board(self.board_path)

    def test_optional_probe_serial_defaults_to_auto_selection(self):
        self.board.update(board_verified=True, srst_connected=True)
        del self.board["jlink_serial"]
        self.write_board()
        self.assertEqual(dcu.require_board(self.board_path)["jlink_serial"], "")

    def test_without_confirmation_no_hardware_or_build_process_is_started(self):
        for action in ("flash", "verify", "debug-server"):
            with self.subTest(action=action), mock.patch.dict(os.environ, {}, clear=True), \
                 mock.patch.object(dcu, "run") as run, mock.patch.object(dcu.os, "execve") as execute, \
                 mock.patch.object(dcu, "require_board") as board, contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(dcu.main([action]), 1)
                run.assert_not_called()
                execute.assert_not_called()
                board.assert_not_called()

    def test_confirmation_without_review_still_does_not_start_a_process(self):
        with mock.patch.object(dcu, "run") as run, contextlib.redirect_stderr(io.StringIO()):
            result = dcu.main(["flash", "--confirm-target", "RESET_TARGET", "--board", str(self.board_path)])
            self.assertEqual(result, 1)
            run.assert_not_called()

    def test_verify_cannot_be_changed_into_flashing_by_a_flag(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            dcu.main(["verify", "--flash-first"])

    def test_programming_requires_backup_and_checks_it_afterward(self):
        script = dcu.transaction_script("/tmp/image.elf", self.board, "/tmp/backup.bin", flash=True)
        for name, start, size in (("backup.bin", dcu.RETAINED_START, dcu.RETAINED_SIZE),
                                  ("gateway-id-journal.bin", dcu.JOURNAL_START, dcu.JOURNAL_SIZE)):
            dump = f'dump_image "/tmp/{name}" {start} {size}'
            verify = f'verify_image "/tmp/{name}" {start} bin'
            self.assertLess(script.index(dump), script.index(verify))
            self.assertLess(script.index(verify), script.index("flash write_image erase"))
            self.assertLess(script.index('verify_image "/tmp/image.elf"'), script.rindex(verify))
        self.assertNotIn("mass_erase", script)
        self.assertNotIn("resume", script)
        self.assertTrue(script.endswith("reset halt\nshutdown\n"))

    def test_programming_without_backup_is_rejected(self):
        with self.assertRaises(ValueError):
            dcu.transaction_script("image.elf", self.board, flash=True)

    def test_pre_reset_backup_is_verified_before_and_after_programming(self):
        script = dcu.transaction_script("image.elf", self.board, "backup.bin", flash=True,
                                        server=True, backup_before_reset=True)
        steps = ["init\n", "halt\n", 'dump_image "backup.bin"', 'dump_image "gateway-id-journal.bin"',
                 "DCU_RETAINED_BACKUP_COMPLETE", "reset init", "flash probe", 'verify_image "backup.bin"',
                 'verify_image "gateway-id-journal.bin"', "flash write_image erase",
                 'verify_image "image.elf"', "reset halt", "DCU_DEBUG_READY"]
        self.assertEqual(sorted(script.index(step) for step in steps), [script.index(step) for step in steps])
        self.assertEqual(script.count('verify_image "backup.bin"'), 2)
        self.assertEqual(script.count('dump_image "backup.bin"'), 1)
        self.assertEqual(script.count('verify_image "gateway-id-journal.bin"'), 2)

    def test_pre_reset_backup_cannot_run_without_programming(self):
        with mock.patch.object(dcu, "require_target") as require_target, contextlib.redirect_stderr(io.StringIO()):
            result = dcu.main(["debug-server", "--backup-before-reset", "--confirm-target", "RESET_TARGET"])
        self.assertEqual(result, 1)
        require_target.assert_not_called()

    def test_verify_does_not_issue_flash_writes(self):
        script = dcu.transaction_script("image.elf", self.board)
        self.assertNotIn("write_image", script)
        self.assertNotIn("dump_image", script)
        self.assertIn("verify_image", script)

    def test_server_is_ready_only_after_verification_and_final_reset(self):
        script = dcu.transaction_script("image.elf", self.board, server=True)
        self.assertLess(script.index("verify_image"), script.index("DCU_DEBUG_READY"))
        self.assertLess(script.index("reset halt"), script.index("DCU_DEBUG_READY"))
        self.assertNotIn("shutdown", script)

    def test_tcl_paths_are_quoted_and_substitution_is_escaped(self):
        quoted = dcu.tcl_quote('/tmp/a $b [reset] "x".elf')
        self.assertTrue(quoted.startswith('"') and quoted.endswith('"'))
        self.assertIn("\\$b", quoted)
        self.assertIn("\\[reset\\]", quoted)
        self.assertIn('\\"x\\"', quoted)
        with self.assertRaises(ValueError):
            dcu.tcl_quote("bad\npath")

    def test_editor_uses_guarded_server_and_does_not_auto_continue(self):
        launch = json.loads((ROOT / ".vscode/launch.json").read_text())
        for configuration in launch["configurations"]:
            self.assertEqual(configuration["serverStarted"], "DCU_DEBUG_READY")
            self.assertEqual(configuration["launchCompleteCommand"], "None")
            self.assertTrue(configuration.get("stopAtConnect"))
            self.assertTrue(configuration["hardwareBreakpoints"]["require"])
            self.assertIn("${input:targetConfirmation}", configuration["debugServerArgs"])
            self.assertNotIn("setupCommands", configuration)
            self.assertIn("hbreak main", [item["text"] for item in configuration["postRemoteConnectCommands"]])

    def test_disconnect_is_explicitly_configured_to_halt(self):
        configuration = (ROOT / "debug/cfg/openocd.cfg").read_text()
        self.assertIn("-event gdb-detach { halt }", configuration)

    def test_external_breakpoints_pause_without_becoming_exceptions(self):
        launch = json.loads((ROOT / ".vscode/launch.json").read_text())
        for configuration in launch["configurations"]:
            with self.subTest(configuration=configuration["name"]):
                self.assertEqual(configuration.get("unknownBreakpointHandling"), "stop")

    def test_hardware_reset_waits_for_release_without_early_halt(self):
        configuration = (ROOT / "debug/cfg/openocd.cfg").read_text()
        self.assertIn("adapter srst delay 500", configuration)
        self.assertGreater(configuration.index("$_TARGET0 configure -event reset-deassert-pre {}"),
                           configuration.index("source [find soc/hpm5e00.cfg]"))

    def test_target_transaction_uses_a_checked_private_snapshot(self):
        directory = Path(self.directory.name)
        original = directory / "current-build.elf"
        original.write_bytes(b"image under test")
        args = SimpleNamespace(action="flash", preset="hpm-debug", skip_build=True)
        with mock.patch.object(dcu, "ROOT", directory), \
             mock.patch.object(dcu, "require_target", return_value={**self.board, "jlink_serial": "123"}), \
             mock.patch.object(dcu, "elf_path", return_value=original), \
             mock.patch.object(dcu, "inspect_image", return_value={"sha256": "test"}) as inspect, \
             mock.patch.object(dcu, "run") as run, contextlib.redirect_stdout(io.StringIO()):
            dcu.target_operation(args)
        snapshot = next((directory / "artifacts/target").glob("*/image.elf"))
        self.assertEqual(snapshot.read_bytes(), b"image under test")
        self.assertEqual(inspect.call_args.args[0], snapshot)
        script = (snapshot.parent / "transaction.cfg").read_text()
        self.assertIn(str(snapshot), script)
        self.assertNotIn(str(original), script)
        run.assert_called_once()
        self.assertIn(str(directory / "debug/cfg/openocd.cfg"), run.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
