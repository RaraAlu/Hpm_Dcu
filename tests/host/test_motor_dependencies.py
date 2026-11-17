import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware"
INCLUDE_ROOTS = tuple(FIRMWARE / path for path in (
    "app/motor", "devices/motor", "devices/motor/drivers/encos", "protocols/encos", "ports", "cfg", "comm/canfd",
))
HEADERS = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.MULTILINE)


class MotorDependenciesTest(unittest.TestCase):
    def test_business_device_and_codec_dependencies_have_explicit_direction(self):
        for directory in ("app/motor", "devices/motor", "protocols/encos"):
            for path in (FIRMWARE / directory).rglob("*"):
                if path.suffix not in (".h", ".hpp", ".c", ".cpp"):
                    continue
                with self.subTest(path=path.relative_to(ROOT)):
                    text = path.read_text()
                    self.assertNotRegex(text, r"\b(?:MC_|mc_)\w+")
                    for include in HEADERS.findall(text):
                        candidates = [path.parent / include, *(root / include for root in INCLUDE_ROOTS)]
                        target = next((candidate.resolve() for candidate in candidates if candidate.is_file()), None)
                        self.assertIsNotNone(target, f"Unowned or unavailable dependency: {path}: {include}")
                        relative = target.relative_to(FIRMWARE).as_posix()
                        allowed = relative.startswith(("ports/", "devices/motor/", "protocols/encos/"))
                        if directory == "app/motor":
                            allowed |= relative.startswith("app/motor/") or relative in (
                                "cfg/device_config.h", "cfg/cycle_config.h", "comm/canfd/can_transport.h",
                            )
                        self.assertTrue(allowed, f"Dependency crosses ownership: {path}: {relative}")

    def test_retired_dictionary_driven_application_entrypoints_are_absent(self):
        for path in (FIRMWARE / "app/motor").rglob("*"):
            if path.suffix in (".h", ".hpp", ".c", ".cpp"):
                self.assertNotRegex(path.read_text(), r"\b(?:PdoPublication|select_profile|owner_changed|refresh_axis_feedback)\b")


if __name__ == "__main__":
    unittest.main()
