#!/usr/bin/env python3
"""Reproducible offline builds and explicitly gated HPM5E31 target operations."""

import argparse
import datetime
import json
import os
import re
import shutil
import socket
import subprocess
import sys
from pathlib import Path

from image_guard import FLASH_SIZE, RETAINED_SIZE, RETAINED_START, SECTOR_SIZE, inspect_image

ROOT = Path(__file__).resolve().parents[1]
SDK = ROOT / "hpm_sdk_localized_for_hpm5e00evk"
GCC = ROOT / ".tools/hpm-gcc-13.2.0/bin"
OPENOCD = ROOT / "scripts/openocd.sh"
PRESETS = ("hpm-debug", "hpm-release")
CONFIRMATION = "RESET_TARGET"


def run(command, **kwargs):
    print("+ " + " ".join(str(part) for part in command), flush=True)
    return subprocess.run([str(part) for part in command], cwd=ROOT, check=True, **kwargs)


def elf_path(preset):
    return ROOT / "build" / preset / "output/dcu.elf"


def configure(preset):
    run(["cmake", "--preset", preset])


def build(preset):
    configure(preset)
    run(["cmake", "--build", "--preset", preset])
    return inspect_image(elf_path(preset))


def usb_devices():
    devices = []
    for node in Path("/sys/bus/usb/devices").glob("*"):
        try:
            if (node / "idVendor").read_text().strip() != "1366":
                continue
            bus = int((node / "busnum").read_text())
            number = int((node / "devnum").read_text())
            device = Path(f"/dev/bus/usb/{bus:03d}/{number:03d}")
            devices.append({
                "serial": (node / "serial").read_text().strip(), "node": str(device),
                "product": (node / "product").read_text().strip(),
                "writable": os.access(device, os.R_OK | os.W_OK),
            })
        except (OSError, ValueError):
            continue
    return devices


def doctor():
    checks = [
        ("GCC", [GCC / "riscv32-unknown-elf-gcc", "--version"], "13.2.0"),
        ("GDB RV32", [ROOT / "scripts/gdb.sh", "--batch", "-ex", "set architecture riscv:rv32",
                       "-ex", "show architecture"], '"riscv:rv32"'),
        ("Ninja", [ROOT / ".tools/host/usr/bin/ninja", "--version"], "1.11.1"),
        ("OpenOCD J-Link", [OPENOCD, "-c", "adapter list", "-c", "shutdown"], "jlink"),
    ]
    ok = True
    for name, command, expected in checks:
        try:
            result = subprocess.run([str(part) for part in command], cwd=ROOT, text=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
            passed = result.returncode == 0 and expected in result.stdout
            print(f"{'PASS' if passed else 'FAIL'} {name}: {result.stdout.strip()}")
            ok &= passed
        except (OSError, subprocess.TimeoutExpired) as error:
            print(f"FAIL {name}: {error}")
            ok = False
    devices = usb_devices()
    for device in devices:
        print(f"{'PASS' if device['writable'] else 'WARN'} USB: {json.dumps(device)}")
    if not devices:
        print("WARN No SEGGER USB device is currently enumerated")
    try:
        require_board(ROOT / "config/board.local.json")
        print("PASS Board profile review is declared (not a hardware verification)")
    except (OSError, ValueError) as error:
        print(f"WARN Target operations are disabled: {error}")
    print("No target initialization, reset, halt or programming was attempted.")
    return 0 if ok else 1


def require_board(path):
    if not path.is_file():
        raise ValueError("Create config/board.local.json from the example and verify the real board first")
    board = json.loads(path.read_text())
    if not isinstance(board, dict):
        raise ValueError("Board configuration must be a JSON object")
    if board.get("board_verified") is not True or board.get("srst_connected") is not True:
        raise ValueError("Board review and the nSRST connection must both be explicitly confirmed")
    if type(board.get("flash_size_bytes")) is not int or board["flash_size_bytes"] != FLASH_SIZE:
        raise ValueError("Flash capacity must match this localized 1 MiB firmware; review the SDK for other boards")
    if type(board.get("flash_sector_size_bytes")) is not int or board["flash_sector_size_bytes"] != SECTOR_SIZE:
        raise ValueError("This EEPROM implementation requires verified 4096-byte erase sectors")
    speed = board.get("adapter_speed_khz")
    if type(speed) is not int or not 1 <= speed <= 1000:
        raise ValueError("Initial JTAG speed must be between 1 and 1000 kHz")
    serial = board.get("jlink_serial", "")
    if not isinstance(serial, str) or (serial and not serial.isdigit()):
        raise ValueError("J-Link serial must be a decimal string")
    board["jlink_serial"] = serial
    for key in ("flash_opt0", "flash_opt1"):
        if not isinstance(board.get(key), str) or not re.fullmatch(r"0x[0-9a-fA-F]{1,8}", board[key]):
            raise ValueError(f"{key} must be an explicitly reviewed hexadecimal value")
    return board


def require_target(args):
    if args.confirm_target != CONFIRMATION:
        raise ValueError("Target access denied. Isolate motor power, then explicitly pass --confirm-target RESET_TARGET")
    board = require_board(args.board)
    devices = [item for item in usb_devices()
               if not board["jlink_serial"] or item["serial"] == board["jlink_serial"]]
    if len(devices) != 1:
        raise ValueError("Exactly one matching J-Link must be present; configure its serial explicitly")
    if not devices[0]["writable"]:
        raise ValueError("No read/write USB access. Run scripts/setup-usb.sh in a local terminal, then reconnect J-Link")
    board["jlink_serial"] = devices[0]["serial"]
    if not board["jlink_serial"].isdigit():
        raise ValueError("The selected probe did not report a decimal serial")
    with socket.socket() as probe:
        probe.settimeout(0.2)
        if probe.connect_ex(("127.0.0.1", 3333)) == 0:
            raise ValueError("Port 3333 is already in use; stop the other debug server first")
    return board


def tcl_quote(value):
    value = str(value)
    if "\n" in value or "\r" in value:
        raise ValueError("Newlines are not allowed in OpenOCD paths")
    for source, replacement in (("\\", "\\\\"), ('"', '\\"'), ("$", "\\$"), ("[", "\\["), ("]", "\\]")):
        value = value.replace(source, replacement)
    return '"' + value + '"'


def transaction_script(elf, board, backup=None, flash=False, server=False):
    lines = [
        "init", "reset init", "flash probe xpi0",
        "set actual_bank [lindex [flash list] 0]",
        f"if {{[dict get $actual_bank size] != {board['flash_size_bytes']}}} {{ error \"Flash capacity mismatch\" }}",
        "flash info xpi0",
    ]
    if flash:
        if backup is None:
            raise ValueError("A retained-data backup is mandatory before programming")
        lines.extend([
            f"dump_image {tcl_quote(backup)} {RETAINED_START} {RETAINED_SIZE}",
            f"flash write_image erase {tcl_quote(elf)}",
            f"verify_image {tcl_quote(elf)}",
            f"verify_image {tcl_quote(backup)} {RETAINED_START} bin",
        ])
    else:
        # Do not debug current symbols against a different image in Flash.
        lines.append(f"verify_image {tcl_quote(elf)}")
    lines.append("reset halt")
    if server:
        lines.append('echo "DCU_DEBUG_READY"')
    else:
        lines.append("shutdown")
    return "\n".join(lines) + "\n"


def target_operation(args):
    board = require_target(args)
    if not args.skip_build:
        build(args.preset)
    image = elf_path(args.preset)
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    directory = ROOT / "artifacts/target" / stamp
    directory.mkdir(parents=True)
    # A concurrent rebuild must not replace the image after its erase span was checked.
    snapshot = directory / "image.elf"
    shutil.copyfile(image, snapshot)
    report = inspect_image(snapshot, board["flash_size_bytes"], board["flash_sector_size_bytes"])
    report["source_elf"] = str(image)
    backup = directory / "retained-eeprom.bin"
    flashing = args.action == "flash" or getattr(args, "flash_first", False)
    script = directory / "transaction.cfg"
    script.write_text(transaction_script(snapshot, board, backup, flashing, args.action == "debug-server"))
    (directory / "image-check.json").write_text(json.dumps(report, indent=2) + "\n")
    environment = os.environ.copy()
    environment.update({
        "DCU_JLINK_SERIAL": board["jlink_serial"], "DCU_JTAG_KHZ": str(board["adapter_speed_khz"]),
        "DCU_FLASH_BYTES": str(board["flash_size_bytes"]), "DCU_FLASH_OPT0": board["flash_opt0"],
        "DCU_FLASH_OPT1": board["flash_opt1"],
    })
    command = [str(OPENOCD), "-s", str(SDK / "boards/openocd"),
               "-f", str(ROOT / "debug/openocd.cfg"), "-f", str(script)]
    print(f"Target operation: {args.action}; audit directory: {directory}", flush=True)
    if args.action == "debug-server":
        # Preserve the PID so a debugger stopping its server cannot orphan OpenOCD.
        os.execve(str(OPENOCD), command, environment)
    command.extend(["-l", str(directory / "openocd.log")])
    run(command, env=environment, timeout=180)
    print("Completed. No resume command was issued; keep motor power isolated.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    sub.add_parser("doctor", help="Check tools and USB permissions without initializing a target")
    sub.add_parser("bootstrap", help="Install locked local tools, without sudo")
    sub.add_parser("test", help="Run offline guard and tooling tests")
    sub.add_parser("usb-check", help="Check USB enumeration and permissions without opening the probe")
    for action in ("configure", "build", "check-image", "flash", "verify", "debug-server"):
        command = sub.add_parser(action)
        command.add_argument("--preset", choices=PRESETS, default="hpm-debug")
        if action in ("flash", "verify", "debug-server"):
            command.add_argument("--board", type=Path, default=ROOT / "config/board.local.json")
            command.add_argument("--confirm-target", default=os.environ.get("DCU_TARGET_CONFIRM", ""))
            command.add_argument("--skip-build", action="store_true")
            if action == "debug-server":
                command.add_argument("--flash-first", action="store_true", help="Program before starting the debug server")
    args = parser.parse_args(argv)
    try:
        if args.action == "bootstrap":
            import bootstrap
            bootstrap.main()
        elif args.action == "doctor":
            return doctor()
        elif args.action == "usb-check":
            devices = usb_devices()
            print(json.dumps(devices, indent=2))
            return 0 if len(devices) == 1 and devices[0]["writable"] else 1
        elif args.action == "test":
            run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"])
        elif args.action == "configure":
            configure(args.preset)
        elif args.action == "build":
            build(args.preset)
        elif args.action == "check-image":
            print(json.dumps(inspect_image(elf_path(args.preset)), indent=2))
        else:
            target_operation(args)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
        print(f"dcu: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
