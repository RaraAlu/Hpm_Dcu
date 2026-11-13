#!/usr/bin/env python3
"""Install pinned tools inside the workspace, without sudo or target access."""

import hashlib
import json
import platform
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / ".tools"
REQUIRED_FILES = {
    "hpm-gcc-13.2.0": ["bin/riscv32-unknown-elf-gcc", "bin/riscv32-unknown-elf-gdb"],
    "openocd-hpm-0.3.0": ["bin/openocd", "share/openocd/scripts/interface/jlink.cfg"],
    "host": ["usr/bin/ninja", "usr/lib/x86_64-linux-gnu/libhidapi-hidraw.so.0"],
}


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    if sys.version_info < (3, 12):
        raise RuntimeError("Python 3.12 or newer is required for safe archive extraction")
    if platform.system() != "Linux" or platform.machine() != "x86_64":
        raise RuntimeError("This tool lock targets Ubuntu 24.04 x86_64")
    host = platform.freedesktop_os_release()
    if host.get("ID") != "ubuntu" or host.get("VERSION_ID") != "24.04":
        raise RuntimeError("The locked host packages require Ubuntu 24.04")
    lock = json.loads((ROOT / "tools/toolchain.lock.json").read_text())
    cache = TOOLS / "cache"
    cache.mkdir(parents=True, exist_ok=True)
    groups = {}
    for asset in lock["assets"]:
        package = cache / asset["file"]
        if package.exists() and sha256(package) != asset["sha256"]:
            raise RuntimeError(f"Checksum mismatch: {package}; refusing to use or overwrite it")
        if not package.exists():
            partial = package.with_suffix(package.suffix + ".part")
            subprocess.run(["curl", "--fail", "--location", "--retry", "3",
                            "--connect-timeout", "20", "--output", str(partial),
                            asset["url"]], check=True)
            if sha256(partial) != asset["sha256"]:
                raise RuntimeError(f"Downloaded checksum mismatch: {partial}")
            partial.replace(package)
        groups.setdefault(asset["destination"], []).append((asset, package))

    for destination, packages in groups.items():
        target = TOOLS / destination
        manifest = {asset["name"]: asset["sha256"] for asset, _ in packages}
        marker = target / ".dcu-install.json"
        if (marker.is_file() and json.loads(marker.read_text()) == manifest
                and all((target / item).is_file() for item in REQUIRED_FILES[destination])):
            print(f"Already installed: {destination}")
            continue
        with tempfile.TemporaryDirectory(prefix=".install-", dir=TOOLS) as temporary:
            staging = Path(temporary) / "payload"
            staging.mkdir()
            for asset, package in packages:
                if package.suffix == ".deb":
                    subprocess.run(["dpkg-deb", "--extract", str(package), str(staging)], check=True)
                else:
                    unpacked = Path(temporary) / asset["name"]
                    unpacked.mkdir()
                    with tarfile.open(package) as archive:
                        archive.extractall(unpacked, filter="data")
                    source = unpacked / asset["archive_root"]
                    shutil.copytree(source, staging, dirs_exist_ok=True, symlinks=True)
            (staging / ".dcu-install.json").write_text(json.dumps(manifest, indent=2) + "\n")
            if target.is_symlink():
                raise RuntimeError(f"Refusing to replace symlink: {target}")
            if target.exists():
                # Only generated tool directories named by this lock are replaced.
                shutil.rmtree(target)
            staging.replace(target)
        print(f"Installed: {destination}")
    print("Tools are local to .tools; no system packages or USB permissions were changed.")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"bootstrap: {error}", file=sys.stderr)
        sys.exit(1)
