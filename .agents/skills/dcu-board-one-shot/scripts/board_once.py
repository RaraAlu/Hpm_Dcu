"""Run or inspect one protected HPM/RK production motor PDO acceptance shot."""

import argparse
import contextlib
import fcntl
import hashlib
import json
from pathlib import Path
import re
import shlex
import struct
import subprocess as sp
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[4]
SCRIPTS = Path(__file__).resolve().parent
BUILD = ROOT / "build/hpm-release"
CONTRACT = BUILD / "firmware/profile-contract/protocol/ethercat"
ELF = BUILD / "output/dcu.elf"
LAYOUT = "\u5468\u671f\u5e03\u5c40.json"
CONTRACT_FILES = (
    LAYOUT, "dcu_device_pdo.hpp", "dcu_profile_pdo.hpp",
    "HPM_ENCOS_4CAN_28AXIS_IMU.xml", "HPM_ENCOS_4CAN_28AXIS_IMU.bin",
)
NM = ROOT / ".tools/hpm-gcc-13.2.0/bin/riscv32-unknown-elf-nm"
SAMPLES = struct.Struct("<17I")
PIPELINE_FIELDS = struct.Struct("<21I")
LATENCY_FIELDS = struct.Struct("<8I")
DUMPS = {
    "batch-sample": ("s_batch_sampleE", 28 * SAMPLES.size),
    "pipeline": ("pipeline", 2496),
    "cyclic_latency": ("cyclic_latency", LATENCY_FIELDS.size),
}
MASK32 = 0xFFFFFFFF
MIT_LOG = re.compile(
    r"Motor MIT: batches=(\d+) n_plus_one=(\d+) seq=(\d+) accepted=(\d+)"
    r" .*?requested=(0x[0-9a-fA-F]+) fresh=(0x[0-9a-fA-F]+)"
)


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def log_command(path, args, timeout, stdin=None):
    print(f"{path.name}: {shlex.join(map(str, args))}", flush=True)
    with path.open("wb") as out:
        return sp.run(args, input=stdin, stdout=out, stderr=sp.STDOUT,
                      cwd=ROOT, timeout=timeout, check=False).returncode


def remote(args, path, mode, manifest, timeout):
    python = shlex.join(["/usr/bin/python3", "-u", "-", mode, json.dumps(manifest)])
    command = f"cd -- {shlex.quote(args.rk_repo)} && "
    command += ("source /opt/ros/humble/setup.bash && source ./env.sh && "
                f"export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID={args.ros_domain} && exec {python}")
    cmd = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
           "--", args.rk, "bash -c " + shlex.quote(command)]
    return log_command(path, cmd, timeout, (SCRIPTS / "rk_once.py").read_bytes())


def locate_symbols(image):
    output = sp.check_output([NM, "-S", "--defined-only", image], text=True, timeout=10)
    lines = [line.split() for line in output.splitlines()]
    result = {}
    for artifact, (symbol, size) in DUMPS.items():
        matches = [fields for fields in lines if len(fields) == 4 and
                   (fields[3].endswith(symbol) if artifact == "batch-sample" else fields[3] == symbol)]
        require(len(matches) == 1 and int(matches[0][1], 16) == size,
                f"missing or changed {symbol} layout; cannot decode HPM evidence")
        result[artifact] = (int(matches[0][0], 16), size)
    return result


@contextlib.contextmanager
def protected_server(args, run, expected_sha):
    cmd = [sys.executable, ROOT / "scripts/dcu.py", "debug-server", "--preset", "hpm-release",
           "--skip-build", "--board", args.board, "--confirm-target", "RESET_TARGET"]
    if args.flash:
        cmd.append("--flash-first")
    log = run / "protected-target.log"
    with log.open("wb") as out:
        server = sp.Popen(cmd, stdout=out, stderr=sp.STDOUT, cwd=ROOT)
        try:
            until = time.monotonic() + 180
            while time.monotonic() < until:
                require(server.poll() is None, "protected target operation failed")
                content = log.read_text(errors="replace")
                match = re.search(r"(?m)^Target operation: debug-server; audit directory: (.+)$", content)
                if match and "DCU_DEBUG_READY" in content.splitlines():
                    image = Path(match[1]) / "image.elf"
                    require(sha(image) == expected_sha, "protected image differs from the checked ELF")
                    (run / "target-audit.txt").write_text(str(image.parent) + "\n")
                    yield image
                    return
                time.sleep(0.15)
            raise TimeoutError("protected server did not report ready")
        finally:
            if server.poll() is None:
                server.terminate()
                try:
                    server.wait(timeout=5)
                except sp.TimeoutExpired:
                    server.kill()
                    server.wait(timeout=5)


def gdb(image, log, commands):
    cmd = [ROOT / "scripts/gdb.sh", "-q", "-batch", image,
           "-ex", "set pagination off", "-ex", "target remote localhost:3333"]
    for action in [*commands, "disconnect"]:
        cmd.extend(("-ex", action))
    return log_command(log, cmd, 35)


def snapshot(image, symbols, run):
    # RK is already Idle/PREOP. Halt only for a coherent dump, without resetting.
    commands = ["monitor halt"]
    for name, (addr, size) in symbols.items():
        commands.append(f'dump binary memory "{run / (name + ".bin")}" {addr:#x} {addr + size:#x}')
    try:
        require(gdb(image, run / "hpm-snapshot.log", commands) == 0, "HPM evidence dump failed")
    finally:
        require(gdb(image, run / "hpm-resume.log", ["monitor resume"]) == 0,
                "could not confirm HPM resume")
    for name, (_, size) in symbols.items():
        require((run / f"{name}.bin").stat().st_size == size, f"incomplete HPM dump: {name}")


def evidence_value(content, label):
    lines = [line.partition(" ")[2] for line in content.splitlines() if line.startswith(label + " ")]
    require(len(lines) == 1, f"missing or duplicate RK evidence: {label}")
    return json.loads(lines[0])


def report(run, layout_file):
    axes = json.loads(layout_file.read_bytes())["axes"]
    require(len(axes) == 13 and len({a["slot"] for a in axes}) == 13, "invalid 13-axis contract")
    required_mask = sum(1 << a["slot"] for a in axes)
    log = (run / "one-shot.log").read_text(errors="replace")
    matches = MIT_LOG.findall(log)
    require(len(matches) == 1, "missing or ambiguous RK N+1 evidence")
    batches, n1, sequence, accepted, requested, fresh = [int(s, 0) for s in matches[0]]
    require(requested == required_mask, "RK requested axis mask differs from the contract")
    publications = evidence_value(log, "application_publications")
    initial, final = (evidence_value(log, k) for k in ("preop_epoch", "final_epoch"))
    before, after = (evidence_value(log, k) for k in ("before", "after"))
    require(len(before) == len(after) == len(axes), "RK counter inventory differs from the contract")
    deltas = [(b - a) & MASK32 for a, b in zip(before, after)]
    require(evidence_value(log, "deltas") == deltas, "inconsistent RK counter evidence")
    raw = {}
    for name, (_, size) in DUMPS.items():
        raw[name] = (run / f"{name}.bin").read_bytes()
        require(len(raw[name]) == size, f"HPM evidence size differs: {name}")
    rows = list(SAMPLES.iter_unpack(raw["batch-sample"]))[:len(axes)]
    pipeline = PIPELINE_FIELDS.unpack_from(raw["pipeline"], len(raw["pipeline"]) - PIPELINE_FIELDS.size)
    period, advance = pipeline[15:17]
    require(period == 2000 and advance < period, "invalid HPM period or advance")
    cutoff = period - max(advance, 400)
    require(cutoff > 0 and all(row[0] == sequence and row[2] == rows[0][2] for row in rows),
            "HPM samples are not bound to the same RK command")
    sync = (rows[0][2] - cutoff) & MASK32

    def relative(value):
        return (value - sync) & MASK32 if value else None

    samples = []
    for axis, row in zip(axes, rows):
        values = dict(zip(("tx_eof", "rx_eof", "enqueued", "dequeue_begin", "dequeue_end",
                           "decode_begin", "decoded", "staged"),
                          [relative(row[i]) for i in (4, 5, 9, 11, 12, 13, 14, 6)]))
        tx, rx, staged = (values[key] for key in ("tx_eof", "rx_eof", "staged"))
        valid = tx is not None and rx is not None and staged is not None and 0 <= tx < rx <= staged < cutoff
        samples.append({"bus": axis["bus"], "can_id": axis["can_id"], **values, "on_time": valid})
    latency = LATENCY_FIELDS.unpack(raw["cyclic_latency"])
    end = relative(latency[7])
    publish_ok = (latency[1] == 1 and latency[4] == latency[5] == pipeline[9]
                  and latency[6] == sync and end is not None and end <= cutoff)
    on_time = sum(row["on_time"] for row in samples)
    passed = (publications == batches == n1 == 1 and initial == final and bool(initial[0])
              and accepted == sequence and fresh == required_mask and deltas == [1] * len(axes)
              and on_time == len(axes) and publish_ok and pipeline[12] == 0)
    result = {
        "verdict": "PASS" if passed else "FAIL", "command": sequence, "accepted_n1": accepted,
        "fresh_mask": hex(fresh), "required_mask": hex(required_mask), "tx_deltas": deltas,
        "sync0_us": sync, "cutoff_us": cutoff, "axes_on_time": on_time,
        "strict_sm3_count": latency[1], "strict_sm3_end_us": end,
        "hpm_faults": hex(pipeline[12]), "axes": samples,
    }
    print(json.dumps(result, indent=2))
    return passed


def run_shot(args):
    require(args.confirm_isolated, "explicit --confirm-isolated is required for any J-Link or live run")
    require(not args.rk.startswith("-"), "invalid SSH host")
    require(Path(args.rk_repo).is_absolute(), "--rk-repo must be an absolute path")
    require(0 <= args.ros_domain <= 101, "--ros-domain must be in 0..101")
    target = ROOT / "artifacts/target"
    target.mkdir(parents=True, exist_ok=True)
    with (target / ".dcu-board-one-shot.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        run = Path(tempfile.mkdtemp(prefix="board-one-shot-", dir=target))
        print(f"Evidence: {run}", flush=True)
        if args.build:
            require(log_command(run / "build.log", [sys.executable, ROOT / "scripts/dcu.py", "build",
                                                   "--preset", "hpm-release"], 240) == 0, "firmware build failed")
        manifest = {name: sha(CONTRACT / name) for name in CONTRACT_FILES}
        (run / "contract.sha256.json").write_text(json.dumps(manifest, indent=2) + "\n")
        (run / LAYOUT).write_bytes((CONTRACT / LAYOUT).read_bytes())
        symbols = locate_symbols(ELF)
        image_sha = sha(ELF)
        require(remote(args, run / "rk-preflight.log", "check", manifest, 30) == 0, "RK preflight failed")
        with protected_server(args, run, image_sha) as image:
            require(gdb(image, run / "boot.log", [
                "monitor hpm5e00.cpu0 configure -event gdb-attach {}",
                "monitor hpm5e00.cpu0 configure -event gdb-detach {}",
                "monitor reset run",
            ]) == 0, "HPM boot failed")
            require(manifest == {name: sha(CONTRACT / name) for name in CONTRACT_FILES},
                    "local contract changed before the shot")
            (run / "shot-attempted").touch(exist_ok=False)
            shot_status = None
            try:
                shot_status = remote(args, run / "one-shot.log", "shot", manifest, 150)
            finally:
                require(remote(args, run / "final-state.log", "state", manifest, 20) == 0,
                        "RK Idle/HPM PREOP not confirmed; no HPM halt/readback")
                snapshot(image, symbols, run)
            require(shot_status == 0, "RK shot stopped; evidence preserved; do not retry automatically")
            with (run / "verdict.json").open("w") as out, contextlib.redirect_stdout(out):
                passed = report(run, run / LAYOUT)
        print(f"{'PASS' if passed else 'FAIL'}: {run / 'verdict.json'}")
        return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="mode", required=True)
    live = commands.add_parser("run", help="run a single protected board shot")
    live.add_argument("--rk", required=True)
    live.add_argument("--rk-repo", required=True)
    live.add_argument("--ros-domain", type=int, default=74)
    live.add_argument("--board", type=Path, default=ROOT / "scripts/cfg/board.local.json")
    live.add_argument("--confirm-isolated", action="store_true")
    live.add_argument("--build", action="store_true", help="build hpm-release first")
    live.add_argument("--flash", action="store_true", help="protected flash; otherwise verify the installed ELF")
    offline = commands.add_parser("report", help="inspect existing evidence without SSH or J-Link")
    offline.add_argument("directory", type=Path)
    offline.add_argument("--layout", type=Path)
    args = parser.parse_args()
    try:
        if args.mode == "report":
            layout = args.layout or args.directory / LAYOUT
            return 0 if report(args.directory, layout) else 1
        return run_shot(args)
    except (OSError, ValueError, RuntimeError, sp.SubprocessError, struct.error) as error:
        print(f"STOP: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
