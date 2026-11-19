#!/usr/bin/env python3
"""Reject Flash images whose load/erase span can damage retained data."""

import argparse
import hashlib
import json
import struct
import sys
from pathlib import Path

FLASH_BASE = 0x80000000
FLASH_SIZE = 0x100000
SECTOR_SIZE = 0x1000
RETAINED_START = 0x80080000
RETAINED_SIZE = 0x10000
JOURNAL_START = 0x80090000
JOURNAL_SIZE = 2 * SECTOR_SIZE


def inspect_image(path, flash_size=FLASH_SIZE, sector_size=SECTOR_SIZE):
    data = Path(path).read_bytes()
    if len(data) < 52 or data[:7] != b"\x7fELF\x01\x01\x01":
        raise ValueError("Expected a little-endian ELF32 image")
    header = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    if header[1] != 2 or header[2] != 243:
        raise ValueError("Expected an executable RISC-V ELF")
    entry, phoff, ehsize, phsize, phnum = header[4], header[5], header[8], header[9], header[10]
    if ehsize != 52 or phsize != 32 or not phnum or phoff < ehsize:
        raise ValueError("Unsupported ELF program-header layout")
    if phoff + phsize * phnum > len(data):
        raise ValueError("Truncated ELF program-header table")
    if sector_size <= 0 or sector_size & (sector_size - 1):
        raise ValueError("Erase sector size must be a positive power of two")
    if flash_size <= 0 or FLASH_BASE + flash_size > 0x100000000:
        raise ValueError("Invalid Flash capacity")
    loads = []
    for number in range(phnum):
        kind, offset, _, address, size, memsize, _, _ = struct.unpack_from(
            "<IIIIIIII", data, phoff + number * phsize)
        if kind != 1 or not size:
            continue
        end = address + size
        if size > memsize or offset + size > len(data):
            raise ValueError("Truncated or malformed ELF load segment")
        if address < FLASH_BASE or end > FLASH_BASE + flash_size:
            raise ValueError(f"Nonempty load outside configured Flash: 0x{address:08x}..0x{end:08x}")
        loads.append({"start": address, "end_exclusive": end, "bytes": size})
    if not loads:
        raise ValueError("ELF has no nonempty Flash load segments")
    first = min(item["start"] for item in loads)
    last = max(item["end_exclusive"] for item in loads)
    if first != FLASH_BASE + 0x400 or entry != FLASH_BASE + 0x3000:
        raise ValueError("Image does not match this project's Flash XIP boot layout")
    # OpenOCD may erase holes between segments, so check the whole rounded span.
    erase_start = first // sector_size * sector_size
    erase_end = (last + sector_size - 1) // sector_size * sector_size
    if erase_start < FLASH_BASE or erase_end > FLASH_BASE + flash_size:
        raise ValueError("Rounded erase span exceeds configured Flash")
    for name, start, size in (("gateway boot journal", JOURNAL_START, JOURNAL_SIZE),
                              ("retained EEPROM", RETAINED_START, RETAINED_SIZE)):
        if erase_start < start + size and erase_end > start:
            raise ValueError(f"Erase span overlaps the {name} region")
    return {
        "elf": str(Path(path).resolve()), "sha256": hashlib.sha256(data).hexdigest(),
        "entry": entry, "flash_bytes": flash_size, "sector_bytes": sector_size,
        "loads": loads, "erase_start": erase_start, "erase_end_exclusive": erase_end,
        "retained_start": RETAINED_START, "retained_end_exclusive": RETAINED_START + RETAINED_SIZE,
        "journal_start": JOURNAL_START, "journal_end_exclusive": JOURNAL_START + JOURNAL_SIZE,
        "hardware_geometry_verified": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    try:
        report = inspect_image(args.elf)
    except (OSError, ValueError, struct.error) as error:
        parser.exit(1, f"image guard: {error}\n")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Image guard PASS: erase 0x{report['erase_start']:08x}.."
          f"0x{report['erase_end_exclusive']:08x}, retained EEPROM and gateway boot journal excluded")


if __name__ == "__main__":
    main()
