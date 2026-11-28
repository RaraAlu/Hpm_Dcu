"""RK side of a bounded WG3 production DriverModule single shot."""

import contextlib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess as sp
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

CLI = "/opt/etherlab/bin/ethercat"
CFG = Path("src/module/driver_module/cfg/wg3.yaml")
CONTRACT = Path("build/src/module/driver_module/generator/generated/protocol/ethercat")
LAYOUT = "\u5468\u671f\u5e03\u5c40.json"
ESI = "HPM_ENCOS_4CAN_28AXIS_IMU.xml"
RUNTIME = {
    "aimrt_main": "build/install/bin/aimrt_main",
    "libaimrt_ros2_plugin.so": "build/install/bin/libaimrt_ros2_plugin.so",
    "libpkg.so": "build/libpkg.so",
}


def require(condition, reason):
    if not condition:
        raise RuntimeError(reason)


def emit(key, value):
    print(key, json.dumps(value, ensure_ascii=False), flush=True)


def ec(*args):
    return sp.check_output([CLI, *args], timeout=3)


def require_idle():
    master = ec("master").decode()
    print(master, flush=True)
    require(re.search(r"(?m)^\s*Phase: Idle\s*$", master)
            and re.search(r"(?m)^\s*Active: no\s*$", master)
            and re.search(r"(?m)^\s*Slaves: 1\s*$", master),
            "RK master is not Idle/inactive with exactly one slave")


def slave_al():
    state = ec("slaves", "-p0").decode().strip()
    require(re.match(r"^0\s+\d+:\d+\s+\w+\s+\+\s+HPM_28AXIS(?:\s|$)", state),
            f"unexpected EtherCAT slave: {state}")
    return state.split()[2]


def upload(index, subindex, kind):
    raw = ec("upload", "-p0", "-t", kind, hex(index), str(subindex))
    return raw.hex() if kind == "octet_string" else int(raw.split()[0], 0)


def epoch():
    return [upload(0x2103, 5, "uint64"), upload(0x2103, 2, "uint32")]


def counters(axes):
    return [upload(0x2400 + 16 * axis["slot"], 2, "uint32") for axis in axes]


def preflight(manifest):
    import yaml

    require_idle()
    for name, digest in manifest.items():
        require(Path(name).name == name, f"invalid contract filename: {name}")
        actual = hashlib.sha256((CONTRACT / name).read_bytes()).hexdigest()
        require(actual == digest, f"RK contract differs: {name}")
    for path in RUNTIME.values():
        require(Path(path).is_file(), f"missing RK production runtime: {path}")
    layout = json.loads((CONTRACT / LAYOUT).read_bytes())
    cfg = yaml.safe_load(CFG.read_bytes())
    routes = [(bus["channel"], m["id"], m["name"])
              for bus in cfg["wg3"] for m in bus.get("motor", [])]
    axes = layout["axes"]
    require([(b, i) for b, i, _ in routes] == [(a["bus"], a["can_id"]) for a in axes],
            "RK YAML routes differ from the generated layout")
    require([sum(b == bus for b, _, _ in routes) for bus in (1, 2, 3)] == [3, 5, 5]
            and len(routes) == 13 and len({n for _, _, n in routes}) == 13,
            "this skill only drives the WG3 CAN1/2/3 3+5+5 topology")
    require(cfg["can_passthrough"] is False and cfg["pdo"]["period_us"] == 2000
            and cfg["pdo"]["sync0_shift_us"] == 375,
            "expected production motor PDO, 500 Hz and +375 us Sync0")
    emit("rk_contract", [layout["profile_id"], layout["profile_revision"], layout["sha256"]])
    return cfg, layout, routes


def verify_preop(layout):
    def number(element, label):
        return int(element.findtext(label).replace("#x", "0x"), 0)

    require(slave_al() == "PREOP", "slave must be in PREOP")
    actual = [upload(0x2100, 4, "uint32"), upload(0x2100, 5, "uint32"),
              upload(0x2102, 1, "octet_string")]
    require(actual == [layout["profile_id"], layout["profile_revision"], layout["sha256"]],
            "online Profile identity does not match the build contract")
    emit("online_contract", actual)
    device = ET.parse(CONTRACT / ESI).find(".//Devices/Device")
    for sm, tag, index in ((2, "RxPdo", 0x1C12), (3, "TxPdo", 0x1C13)):
        groups = device.findall(f"{tag}[@Sm='{sm}']")
        require(groups and len(groups) == upload(index, 0, "uint8"),
                f"SM{sm} assignment count differs")
        for subindex, group in enumerate(groups, 1):
            mapping = number(group, "Index")
            require(upload(index, subindex, "uint16") == mapping,
                    f"SM{sm} assignment {subindex} differs")
            expected = [(number(e, "Index") << 16) | (number(e, "SubIndex") << 8)
                        | number(e, "BitLen") for e in group.findall("Entry")]
            require(upload(mapping, 0, "uint8") == len(expected),
                    f"PDO {mapping:#x} entry count differs")
            actual = [upload(mapping, i, "uint32") for i in range(1, len(expected) + 1)]
            require(actual == expected, f"PDO {mapping:#x} entries differ")
        emit(f"preop_sm{sm}", len(groups))
    require(slave_al() == "PREOP", "slave left PREOP during contract verification")


def stop_driver(proc):
    for sig, timeout in ((signal.SIGINT, 6), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        if proc.poll() is not None:
            break
        with contextlib.suppress(ProcessLookupError):
            os.killpg(proc.pid, sig)
        try:
            proc.wait(timeout=timeout)
        except sp.TimeoutExpired:
            continue
    require(proc.poll() is not None, "could not stop the production DriverModule")


def publish_one(node, proc, cfg, routes, run, shot):
    import rclpy
    from my_ros2_proto.msg import JointCommand, MotorStates
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    command_topic = cfg["topics"]["motor_command"]
    pub = node.create_publisher(JointCommand, command_topic, 1)
    states = []
    sub = node.create_subscription(
        MotorStates, cfg["topics"]["motor_states"], lambda msg: states.__setitem__(slice(None), [msg]),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    command = JointCommand()
    command.name = [name for _, _, name in routes]
    for field in ("position", "velocity", "stiffness", "damping", "effort"):
        setattr(command, field, [0.0] * len(routes))
    until = time.monotonic() + 25
    first_sequence = None
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.012)
        require(proc.poll() is None, "production process exited before publication")
        require(node.count_publishers(command_topic) <= 1, "another motor command publisher is present")
        if pub.get_subscription_count() != 1 or node.count_publishers(command_topic) != 1 or not states:
            continue
        state = states[0]
        values = state.motors
        require([(m.can, m.can_id, m.name) for m in values] == routes, "online motor routes differ")
        require(not any(m.motor_fault or m.bridge_fault for m in values), "motor or bridge fault before firing")
        age = time.time_ns() - (state.header.stamp.sec * 10**9 + state.header.stamp.nanosec)
        if not 0 <= age < 10_000_000:
            continue
        if first_sequence is None:
            first_sequence = values[0].publish_seq
            continue
        if values[0].publish_seq == first_sequence or slave_al() != "OP":
            continue
        (run / "submission-started").touch(exist_ok=False)
        command.header.stamp = node.get_clock().now().to_msg()
        shot[0] = 1  # From here an exception is an unknown outcome, not permission to retry.
        pub.publish(command)
        time.sleep(0.12)
        return
    node.destroy_subscription(sub)
    raise RuntimeError("no fresh production motor PDO in OP; no command was published")


def shot(cfg, layout, routes):
    import rclpy

    verify_preop(layout)
    axes = layout["axes"]
    initial = epoch()
    require(initial[0] != 0, "invalid boot identity")
    before = counters(axes)
    emit("preop_epoch", initial)
    emit("before", before)
    run = Path(tempfile.mkdtemp(prefix="dcu-board-one-shot-"))
    emit("rk_evidence", str(run))
    (run / "cfg").mkdir()
    (run / "cfg/wg3.yaml").symlink_to(CFG.resolve(strict=True))
    for name, path in RUNTIME.items():
        (run / name).symlink_to(Path(path).resolve(strict=True))
    sent = [0]
    driver_log = run / "driver.log"
    rclpy.init()
    node = rclpy.create_node("dcu_board_one_shot")
    try:
        with driver_log.open("wb") as log:
            proc = sp.Popen(["./aimrt_main", "--cfg_file_path=./cfg/wg3.yaml"], cwd=run,
                            stdout=log, stderr=sp.STDOUT, start_new_session=True)
            try:
                publish_one(node, proc, cfg, routes, run, sent)
            finally:
                stop_driver(proc)
                emit("process_exit", proc.returncode)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        emit("application_publications", sent[0])
        if driver_log.exists():
            with driver_log.open(errors="replace") as log:
                for line in log:
                    if "Motor MIT:" in line or "PDO timing:" in line:
                        print(line.rstrip(), flush=True)
    require_idle()
    require(slave_al() == "PREOP", "slave did not return to PREOP")
    after = counters(axes)
    emit("after", after)
    emit("final_epoch", epoch())
    emit("deltas", [(new - old) & 0xFFFFFFFF for old, new in zip(before, after)])


def interrupted(signum, _frame):
    raise InterruptedError(f"interrupted by signal {signum}")


def main():
    mode, manifest = sys.argv[1:]
    for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, interrupted)
    with Path("build/.dcu-board-one-shot.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        cfg, layout, routes = preflight(json.loads(manifest))
        if mode == "shot":
            shot(cfg, layout, routes)


if __name__ == "__main__":
    try:
        main()
    except (KeyError, OSError, RuntimeError, ValueError, sp.SubprocessError) as error:
        print(f"RK one-shot stopped: {error}", file=sys.stderr, flush=True)
        sys.exit(2)
