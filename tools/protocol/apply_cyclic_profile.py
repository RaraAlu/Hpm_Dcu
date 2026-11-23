#!/usr/bin/env python3
"""Apply or verify a production YAML's cyclic device subset on an idle IgH master."""

import argparse
from pathlib import Path
import subprocess
import sys

from generate_can_profile import header, load_yaml, profile_digest


def selection(catalogue, desired):
    for p in (catalogue, desired):
        header(p)
    other = lambda p: {k: v for k, v in p.items() if k not in ("channels", "devices")}
    if other(catalogue) != other(desired):
        raise ValueError("only channels and cyclic devices can change in a topology transaction")
    for key, identity in (("channels", "bus"), ("devices", "id")):
        wanted = {row[identity] for row in desired[key]}
        kept = [row for row in catalogue[key] if row[identity] in wanted]
        if kept != desired[key]:
            raise ValueError(f"{key}: unknown, reordered or modified entry; retain catalogue routes, types and limits")
    return [hex(sum(1 << ch["bus"] for ch in desired["channels"])), *(d["id"] for d in desired["devices"])]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalogue", required=True, type=Path, help="Production YAML used to build this firmware")
    parser.add_argument("--profile", required=True, type=Path, help="Same schema, with the desired active subset")
    parser.add_argument("--client", required=True, type=Path, help="IgH executable built against the same catalogue")
    parser.add_argument("--apply", action="store_true", help="Commit in PREOP; default only verifies")
    args = parser.parse_args()
    try:
        catalogue = load_yaml(args.catalogue)
        selectors = selection(catalogue, load_yaml(args.profile))
        client = str(args.client.resolve(strict=True))
        bound = subprocess.check_output([client, "--catalogue-digest"], text=True, timeout=5).strip()
        if bound != profile_digest(catalogue).hex():
            raise ValueError("client and catalogue Profile digests differ")
        return subprocess.run([client, "--apply" if args.apply else "--check", *selectors], check=False).returncode
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Profile selection: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
