#!/usr/bin/env python3
"""Prepare an offline first-install seed for a proven-blank gateway boot journal."""

import argparse
import hashlib
import json
import struct
from pathlib import Path

from image_guard import JOURNAL_SIZE, JOURNAL_START

MAGIC = 0x31444947


def prepare(snapshot, output_dir):
    before = Path(snapshot).read_bytes()
    if len(before) != JOURNAL_SIZE:
        raise ValueError(f"Expected a complete {JOURNAL_SIZE}-byte journal snapshot")
    if before != b"\xff" * JOURNAL_SIZE:
        raise ValueError("Journal is not entirely blank; refusing to initialize or repair it")

    # Epoch zero provisions the journal but is never issued as a boot ID.
    seed = struct.pack("<4I", MAGIC, 0, 0xffffffff, MAGIC ^ 0xffffffff)
    after = seed + before[len(seed):]
    manifest = {
        "journal_start": JOURNAL_START,
        "journal_bytes": JOURNAL_SIZE,
        "required_before_sha256": hashlib.sha256(before).hexdigest(),
        "program_address": JOURNAL_START,
        "program_file": "epoch-zero.bin",
        "program_bytes": len(seed),
        "erase_allowed": False,
        "expected_after_sha256": hashlib.sha256(after).hexdigest(),
    }
    output_dir = Path(output_dir)
    output_dir.mkdir()
    (output_dir / manifest["program_file"]).write_bytes(seed)
    (output_dir / "plan.json").write_text(json.dumps(manifest, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blank-snapshot", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True, help="New directory for the seed and manifest")
    args = parser.parse_args()
    try:
        prepare(args.blank_snapshot, args.output_dir)
    except (OSError, ValueError) as error:
        parser.exit(1, f"gateway boot journal: {error}\n")


if __name__ == "__main__":
    main()
