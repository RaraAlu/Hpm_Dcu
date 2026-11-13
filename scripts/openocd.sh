#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export LD_LIBRARY_PATH="$ROOT/.tools/host/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$ROOT/.tools/openocd-hpm-0.3.0/bin/openocd" \
    -s "$ROOT/.tools/openocd-hpm-0.3.0/share/openocd/scripts" "$@"
