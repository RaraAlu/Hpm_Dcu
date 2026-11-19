#!/usr/bin/env python3
"""Encode the static-topology Profile policy snapshot for CoE 0x2100:01."""

import argparse
from pathlib import Path
import struct
import zlib

from check_gateway_contract import keys, load_yaml, require
from generate_can_profile import bounded


def encode(document, boot_epoch):
    keys(document, {"schema_version", "profile_id", "base_revision", "revision", "policy"}, "candidate")
    require(type(document["schema_version"]) is int and document["schema_version"] == 1,
            "candidate.schema_version must be 1")
    boot_epoch = bounded(boot_epoch, 1, 0xffffffff, "boot_epoch")
    profile_id = bounded(document["profile_id"], 1, 0xffffffff, "profile_id")
    base = bounded(document["base_revision"], 1, 0xffffffff, "base_revision")
    revision = bounded(document["revision"], base + 1, 0xffffffff, "revision")
    policy = document["policy"]
    keys(policy, {"max_command_lease_ms", "feedback_timeout_ms", "readiness_ttl_ms"}, "policy")
    image = struct.pack("<HHIIIQHHI", 1, 36, profile_id, base, revision, boot_epoch << 32,
                        bounded(policy["max_command_lease_ms"], 2, 10, "max_command_lease_ms"),
                        bounded(policy["feedback_timeout_ms"], 5, 20, "feedback_timeout_ms"),
                        bounded(policy["readiness_ttl_ms"], 100, 10000, "readiness_ttl_ms"))
    return image + struct.pack("<I", zlib.crc32(image))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", type=Path)
    parser.add_argument("--boot-epoch", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        args.output.write_bytes(encode(load_yaml(args.profile), args.boot_epoch))
    except (ValueError, OSError) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__":
    main()
