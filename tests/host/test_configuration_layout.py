import contextlib
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import bootstrap
import dcu


class ConfigurationLayoutTest(unittest.TestCase):
    def test_configuration_files_have_canonical_cfg_locations(self):
        files = (
            "firmware/cfg/ecat_config.h",
            "firmware/cfg/user_config.h",
            "firmware/cfg/ecat_def.h",
            "firmware/cfg/dcu_compiler.h",
            "firmware/cfg/device_config.h",
            "firmware/devices/motor/cfg/motor_drivers.hpp",
            "firmware/bsp/dcu_hpm5e31/cfg/tool_config.hpmpc",
            "scripts/cfg/board.example.json",
            "scripts/cfg/60-dcu-jlink.rules",
            "debug/cfg/openocd.cfg",
            "tools/cfg/toolchain.lock.json",
            "cmake/cfg/CMakePresets.json",
        )
        for file in files:
            with self.subTest(file=file):
                self.assertTrue((ROOT / file).is_file())

    def test_retired_configuration_locations_are_absent(self):
        for file in (
            "config", "firmware/config", "debug/openocd.cfg",
            "tools/toolchain.lock.json", "firmware/bsp/dcu_hpm5e31/tool_config.hpmpc",
        ):
            with self.subTest(file=file):
                self.assertFalse((ROOT / file).exists())

    def test_tool_required_entry_files_remain_discoverable(self):
        for file in (
            "CMakeLists.txt", "CMakePresets.json", "app.yaml",
            "firmware/bsp/dcu_hpm5e31/dcu_hpm5e31.yaml",
            ".vscode/launch.json", ".vscode/tasks.json", ".vscode/settings.json",
            ".vscode/c_cpp_properties.json", ".vscode/extensions.json",
            ".clangd",
        ):
            with self.subTest(file=file):
                self.assertTrue((ROOT / file).is_file())

    def test_root_preset_only_forwards_to_cfg_and_cmake_finds_all_presets(self):
        entry = json.loads((ROOT / "CMakePresets.json").read_text())
        self.assertEqual(entry, {"version": 6, "include": ["cmake/cfg/CMakePresets.json"]})
        result = subprocess.run(
            ["cmake", "--list-presets=all"], cwd=ROOT, capture_output=True, text=True, timeout=10,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for kind in ("configure", "build", "test"):
            self.assertIn(f"Available {kind} presets:", result.stdout)
        self.assertEqual(result.stdout.count('"hpm-debug"'), 3)
        self.assertEqual(result.stdout.count('"hpm-release"'), 2)

        presets = json.loads((ROOT / "cmake/cfg/CMakePresets.json").read_text())
        builds = {preset["name"]: preset for preset in presets["buildPresets"]}
        for name in dcu.PRESETS:
            with self.subTest(preset=name):
                self.assertEqual(builds[name]["configurePreset"], name)

    def test_owned_settings_use_cfg_except_the_sdk_board_entry(self):
        extensions = {".json", ".yaml", ".yml", ".cfg", ".hpmpc", ".rules"}
        board_entry = ROOT / "firmware/bsp/dcu_hpm5e31/dcu_hpm5e31.yaml"
        for root in ("firmware", "scripts", "cmake", "debug", "tools"):
            for file in (ROOT / root).rglob("*"):
                if not file.is_file() or file.suffix not in extensions or file == board_entry:
                    continue
                with self.subTest(file=file.relative_to(ROOT)):
                    self.assertIn("cfg", file.relative_to(ROOT).parts[:-1])

    def test_clangd_uses_debug_database(self):
        config = yaml.safe_load((ROOT / ".clangd").read_text())
        self.assertEqual(config["CompileFlags"]["CompilationDatabase"], "build/hpm-debug")

    def test_local_profiles_are_ignored_and_shared_configuration_is_visible(self):
        for file, ignored in (
            ("scripts/cfg/board.local.json", True),
            ("config/board.local.json", True),
            ("scripts/cfg/board.example.json", False),
            ("scripts/cfg/60-dcu-jlink.rules", False),
            ("tools/cfg/toolchain.lock.json", False),
            ("cmake/cfg/CMakePresets.json", False),
            (".clangd", False),
        ):
            with self.subTest(file=file):
                result = subprocess.run(
                    ["git", "check-ignore", "--no-index", "--quiet", file], cwd=ROOT, timeout=10,
                )
                self.assertEqual(result.returncode, 0 if ignored else 1)

    def test_target_cli_defaults_to_the_new_local_profile(self):
        for action in ("flash", "verify", "debug-server"):
            with self.subTest(action=action), mock.patch.object(dcu, "target_operation") as operation:
                self.assertEqual(dcu.main([action, "--confirm-target", "RESET_TARGET"]), 0)
                self.assertEqual(operation.call_args.args[0].board, ROOT / "scripts/cfg/board.local.json")

    def test_bootstrap_reads_the_relocated_lock_without_fetching_anything(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            lock = directory / "tools/cfg/toolchain.lock.json"
            lock.parent.mkdir(parents=True)
            lock.write_text(json.dumps({"assets": []}))
            with mock.patch.object(bootstrap, "ROOT", directory), \
                 mock.patch.object(bootstrap, "TOOLS", directory / ".tools"), \
                 mock.patch.object(bootstrap.sys, "version_info", (3, 12)), \
                 mock.patch.object(bootstrap.platform, "system", return_value="Linux"), \
                 mock.patch.object(bootstrap.platform, "machine", return_value="x86_64"), \
                 mock.patch.object(bootstrap.platform, "freedesktop_os_release",
                                   return_value={"ID": "ubuntu", "VERSION_ID": "24.04"}), \
                 mock.patch.object(bootstrap.subprocess, "run") as run, \
                 contextlib.redirect_stdout(io.StringIO()):
                bootstrap.main()
                run.assert_not_called()
            self.assertTrue((directory / ".tools/cache").is_dir())


if __name__ == "__main__":
    unittest.main()
