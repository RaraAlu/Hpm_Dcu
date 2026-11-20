import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / "firmware"
PRIVATE_HEADERS = {
    "app/gateway/detail/gateway_configuration.hpp": "app/gateway/detail/gateway_application_impl.hpp",
    "app/gateway/detail/profile_update_state.hpp": "app/gateway/profile_update.cpp",
    "devices/motor/detail/motor_instance_state.hpp": "devices/motor/motor_instance.hpp",
    "app/gateway/detail/gateway_application_impl.hpp": (
        "app/gateway/gateway_application.cpp", "app/gateway/gateway_publication.cpp",
    ),
    "app/gateway/detail/can_transactions.hpp": (
        "app/gateway/can_transactions.cpp", "app/gateway/detail/gateway_application_impl.hpp",
    ),
    "app/gateway/detail/production_startup.hpp": (
        "app/gateway/production_startup.cpp", "app/gateway/detail/gateway_application_impl.hpp",
    ),
    "app/gateway/detail/raw_can_submit.hpp": "app/gateway/gateway_application.cpp",
    "devices/motor/drivers/encos/detail/encos_parameter_state.hpp": "devices/motor/drivers/encos/encos_parameter_transactions.cpp",
    "devices/motor/drivers/encos/detail/encos_session_state.hpp": "devices/motor/drivers/encos/encos_driver.hpp",
    "app/cycle/detail/ethercat_cycle_state.h": "app/cycle/ethercat_cycle.c",
    "app/cycle/detail/pdo_pipeline_slots.h": "app/cycle/pdo_pipeline.c",
    "app/runtime/detail/ethercat_can_task_state.h": "app/runtime/ethercat_can_task.c",
    "app/gateway/detail/gateway_readiness_state.h": "app/gateway/gateway_readiness.c",
    "comm/ethercat/detail/motor_pdo_state.hpp": "comm/ethercat/motor_pdo.cpp",
    "comm/ethercat/detail/ethercat_dictionary_owner.h": "comm/ethercat/ethercat_callbacks.c",
    "comm/canfd/detail/can_gateway_state.h": "comm/canfd/hpm_mcan_transport.c",
    "protocols/encos/detail/encos_math.hpp": "protocols/encos/encos_motor.cpp",
}


def owned_files():
    return [path for path in FIRMWARE.rglob("*")
            if path.suffix in (".c", ".cpp", ".h", ".hpp")
            and "generated" not in path.relative_to(FIRMWARE).parts]


class HeaderOwnershipTest(unittest.TestCase):
    def test_public_headers_do_not_export_mutable_storage(self):
        # SDK-derived ESC address contract is the sole owned-header exception.
        # SSC dictionary storage is generated and excluded by owned_files().
        exceptions = {"comm/ethercat/port/hpm_ecat_hw.h": {"extern MEM_ADDR ESCMEM * pEsc;"}}
        exports = {}
        for path in owned_files():
            if path.suffix not in (".h", ".hpp"):
                continue
            text = re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(), flags=re.DOTALL)
            declarations = set()
            for declaration in re.findall(r'\bextern\s+(?!"C")[^;{}]+;', text):
                declaration = " ".join(declaration.split())
                if "(" not in declaration:
                    declarations.add(declaration)
            if declarations:
                exports[path.relative_to(FIRMWARE).as_posix()] = declarations
        self.assertEqual(exports, exceptions)

    def test_transport_wire_and_diagnostics_headers_need_no_hardware_or_rtos_stubs(self):
        includes = ("firmware/cfg", "firmware/ports", "firmware/comm/canfd", "firmware/comm/ethercat",
                    "firmware/generated/encos", "firmware/app/cycle", "firmware/app/runtime",
                    "firmware/app/gateway")
        headers = ("can_transport.h", "raw_can_pdo.h", "realtime_port.h",
                   "ethercat_cycle.h", "ethercat_can_task.h", "gateway_readiness.h", "gateway_session.h")
        for compiler, language, standard in (("cc", "c", "c11"), ("c++", "c++", "c++17")):
            command = [compiler, "-x", language, f"-std={standard}", "-fsyntax-only",
                       "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
            for include in includes:
                command.extend(("-I", str(ROOT / include)))
            for header in (*headers, *(("motor_pdo_layout.hpp", "raw_can_gateway.hpp") if language == "c++" else ())):
                with self.subTest(language=language, header=header):
                    source = f'#include "{header}"\n#include "{header}"\nint main(void) {{ return 0; }}\n'
                    result = subprocess.run([*command, "-"], input=source, text=True,
                                            capture_output=True, timeout=30)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_implementation_files_do_not_define_types_or_macros(self):
        declaration = re.compile(
            r"^\s*(?:#\s*define\b|typedef\b|using\s+\w+\s*=|(?:(?:static|const)\s+)*"
            r"(?:struct|union|enum|class)\b)", re.MULTILINE)
        violations = []
        for path in owned_files():
            if path.suffix not in (".c", ".cpp"):
                continue
            text = re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(), flags=re.DOTALL)
            if declaration.search(text):
                violations.append(str(path.relative_to(FIRMWARE)))
        self.assertEqual(violations, [])

    def test_private_headers_are_only_included_by_their_owner(self):
        files = owned_files()
        for header, owner in PRIVATE_HEADERS.items():
            with self.subTest(header=header):
                self.assertTrue((FIRMWARE / header).is_file())
                consumers = []
                for path in files:
                    includes = re.findall(r'^\s*#\s*include\s*"([^"]+)"', path.read_text(), re.MULTILINE)
                    if any(Path(include).name == Path(header).name for include in includes):
                        consumers.append(path.relative_to(FIRMWARE).as_posix())
                self.assertCountEqual(consumers, (owner,) if isinstance(owner, str) else owner)

    def test_private_type_headers_do_not_export_or_allocate_state(self):
        for header in PRIVATE_HEADERS:
            if header.endswith("ethercat_dictionary_owner.h"):
                continue
            with self.subTest(header=header):
                text = (FIRMWARE / header).read_text()
                text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.DOTALL)
                self.assertNotRegex(text, r"(?m)^\s*(?:extern|static)\s+")
        for compiler, language, standard, includes, headers in self.compilation_groups()[:3]:
            with self.subTest(language=language, headers=headers), tempfile.TemporaryDirectory() as work:
                source = "".join(f'#include "{FIRMWARE / header}"\n' for header in headers)
                left, right = Path(work) / "left.c", Path(work) / "right.c"
                left.write_text(source + "int header_probe(void);\nint main(void) { return header_probe(); }\n")
                right.write_text(source + "int header_probe(void) { return 0; }\n")
                command = [compiler, "-x", language, f"-std={standard}", "-fno-common",
                           "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
                for include in includes:
                    command.extend(("-I", str(ROOT / include)))
                result = subprocess.run([*command, str(left), str(right), "-o", str(Path(work) / "headers")],
                                        text=True, capture_output=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    @staticmethod
    def compilation_groups():
        cc, cxx = shutil.which("cc"), shutil.which("c++")
        if not cc or not cxx:
            raise RuntimeError("Header tests require host cc/c++")
        motor_includes = (
            "tests/host/motor/stubs", "firmware/cfg", "firmware/ports",
            "firmware/generated/encos", "firmware/generated/ethercat",
            "firmware/app/gateway", "firmware/app/cycle", "firmware/protocols/encos",
            "firmware/comm/canfd", "firmware/comm/ethercat", "third_party/beckhoff_ssc/Src",
            "firmware/devices/motor", "firmware/devices/motor/drivers/encos", "firmware/devices/motor/drivers/cih408",
        )
        groups = (
            (cxx, "c++", "c++17", motor_includes,
             [header for header in PRIVATE_HEADERS if header.endswith(".hpp")]),
            (cc, "c", "c11",
             ("tests/host/cycle/stubs", "firmware/app/runtime", "firmware/cfg", "firmware/ports"),
             [header for header in PRIVATE_HEADERS if header.startswith(("app/cycle/", "app/runtime/"))]),
            (cc, "c", "c11",
             ("tests/host/canfd/stubs", "firmware/comm/canfd", "firmware/cfg", "firmware/ports"),
             ["comm/canfd/detail/can_gateway_state.h"]),
            (cc, "c", "c11", motor_includes,
             ["comm/ethercat/detail/ethercat_dictionary_owner.h"]),
        )
        return groups

    def test_private_headers_are_self_contained_and_repeatable(self):
        for compiler, language, standard, includes, headers in self.compilation_groups():
            with self.subTest(language=language, headers=headers):
                command = [compiler, "-x", language, f"-std={standard}", "-fsyntax-only",
                           "-Wall", "-Wextra", "-Werror", "-pedantic-errors"]
                for include in includes:
                    command.extend(("-I", str(ROOT / include)))
                source = "".join(f'#include "{FIRMWARE / header}"\n' for header in headers * 2)
                source += '#ifdef _DIGITAL_IO_\n#error "Application owner selector leaked"\n#endif\n'
                source += "int main(void) { return 0; }\n"
                result = subprocess.run([*command, "-"], input=source, text=True,
                                        capture_output=True, timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
