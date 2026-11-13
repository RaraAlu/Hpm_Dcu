#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
exec "$ROOT/.tools/hpm-gcc-13.2.0/bin/riscv32-unknown-elf-gdb" \
    -nx --data-directory="$ROOT/.tools/hpm-gcc-13.2.0/share/gdb" "$@"
