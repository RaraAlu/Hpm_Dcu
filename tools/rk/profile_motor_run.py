import argparse
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import yaml


def snapshot():
    def read(index, sub, kind="uint32"):
        output = subprocess.check_output(["/opt/etherlab/bin/ethercat", "upload", "-p0",
                                          "-t", kind, hex(index), str(sub)], text=True)
        return int(output.split()[0], 16)
    mask = read(0x2103, 4)
    return (read(0x2103, 5, "uint64"), read(0x2103, 2), mask,
            {a: read(0x2400 + 16 * a, 2) for a in range(28) if mask & (1 << a)})


def supervise(launch, names, cycles, period_us, log_path):
    import rclpy
    from my_ros2_proto.msg import JointCommand
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data

    prefix = f"bench_{os.getpid()}_{cycles}"
    node = rclpy.create_node("profile_request_" + prefix)
    ready = False

    def observe(state):
        nonlocal ready
        ready = len(state.name) == len(names) and set(state.name) == set(names)

    node.create_subscription(JointState, f"/{prefix}/joint_states", observe, qos_profile_sensor_data)
    pub = node.create_publisher(JointCommand, f"/{prefix}/joint_cmds", 1) if cycles else None
    command = JointCommand(name=names, **{key: [0.] * len(names) for key in
                           ("position", "velocity", "effort", "stiffness", "damping")})
    env = os.environ.copy()
    # The launcher sources its install overlay; bind the typesupport built with libpkg1.
    env["LD_PRELOAD"] = str(Path("libmy_ros2_proto__rosidl_typesupport_cpp.so").resolve()) + (
        ":" + env["LD_PRELOAD"] if env.get("LD_PRELOAD") else "")
    success = False
    with log_path.open("x") as output:
        proc = subprocess.Popen(["./run_driver_dcu_hpm.sh", "--config", str(launch),
                                 "--topic-prefix", prefix, "--ethercat-wait-timeout", "0"],
                                stdout=output, stderr=subprocess.STDOUT, env=env)
        try:
            deadline = time.monotonic() + 25 + cycles * period_us / 1_000_000
            next_request, sent_once = 0., False
            with log_path.open() as records:
                while proc.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    for line in records.readlines():
                        if any(tag in line for tag in ("PDO timing:", "N+1 arrival:", "Profile partial:")):
                            print(line.strip(), flush=True)
                        if "Profile window complete:" in line:
                            match = re.search(r"sent=(\d+) matched_n_plus_one=(\d+) missed=(\d+) first_to_drain_us=(\d+)", line)
                            if not match or tuple(map(int, match.groups()[:3])) != (cycles, cycles, 0):
                                raise RuntimeError(line.strip())
                            print(line.strip(), flush=True)
                            success = True
                        if "Driver fault (latched)" in line or "Driver start failed:" in line:
                            raise RuntimeError(line.strip())
                    if success or (not cycles and ready):
                        success = True
                        break
                    if (cycles and ready and pub.get_subscription_count() == 1 and
                            (not sent_once or cycles > 1) and time.monotonic() >= next_request):
                        # Ordinary command-source heartbeat only. DriverTick owns every PDO and its quota.
                        pub.publish(command)
                        sent_once = True
                        next_request = time.monotonic() + 0.04
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            proc.wait(timeout=20)
            node.destroy_node()
    if not success:
        raise RuntimeError(f"Driver did not confirm this window: {log_path}")


def run(profile, cycles, log_dir, schedule_file, period_us):
    axes = sorted((a for a in profile["devices"] if a["bus"] == 2),
                  key=lambda a: a["can_id"])
    if [a["can_id"] for a in axes] != [1, 2, 3, 4] or not all(
            a["type"] == "EC-A7216-P2-22H" and a["zero_mit_loopback"] for a in axes):
        raise ValueError("unexpected CAN2 Profile")
    commands = []
    for i, a in enumerate(axes, 1):
        lim = a["limits"]
        commands.append(dict(name=a["id"], type="7216", slot=i, can_id=a["can_id"],
                             position_limit_rad=[v / 1000 for v in lim["position_mrad"]],
                             current_limit_a=lim["current_dA"] / 10,
                             torque_limit_nm=lim["torque_mNm"] / 1000,
                             velocity_limit_rad_s=lim["velocity_mrad_s"] / 1000))
    device = dict(schema_version=5, connect_device=True, enable_actuator=bool(cycles),
                  actuator_list=[a["id"] for a in axes],
                  dcu=dict(ethercat=dict(master=0, schedule_file=str(schedule_file),
                                         profile_managed=True, command_window_cycles=cycles,
                                         command_timeout_ms=100, enable_dc=True, bind_cpu=6, rt_priority=80),
                           axis_defaults=dict(can_fd_brs=True),
                           dcu_network=[dict(name="bench", ecat_id=1, slave_position=0, channel_3=commands)]))
    runtime = yaml.safe_load(Path("cfg/driver_dcu_hpm.yaml").read_text())
    runtime.pop("driver_gateway_config")
    with tempfile.TemporaryDirectory(prefix="profile-driver-") as temp:
        work = Path(temp)
        (work / "axes.yaml").write_text(yaml.safe_dump(dict(schema_version=1, axes=[
            dict(joint=a["id"], actuator=a["id"]) for a in axes])))
        (work / "policy.yaml").write_text(yaml.safe_dump(dict(command_timeout_behavior="stop", command_timeout_ms=100)))
        gw = yaml.safe_load(Path("cfg/robot_gateway/gateway.yaml").read_text())
        gw.update(robot_description="axes.yaml", command_policy="policy.yaml")
        (work / "gateway.yaml").write_text(yaml.safe_dump(gw))
        device["gateway_config"] = str(work / "gateway.yaml")
        driver = work / "device.yaml"
        driver.write_text(yaml.safe_dump(device))
        module, = runtime["aimrt"]["module"]["modules"]
        assert module["name"] == "DriverModule"
        module["cfg_file_path"] = str(driver)
        launch = work / "runtime.yaml"
        launch.write_text(yaml.safe_dump(runtime))
        before = snapshot()
        if before[2] & (15 << 14) != 15 << 14:
            raise RuntimeError("CAN2 four-axis topology not active")
        log_path = log_dir / f"driver-{cycles}.log"
        print(f"cycles={cycles} before={before} log={log_path}", flush=True)
        try:
            supervise(launch, device["actuator_list"], cycles, period_us, log_path)
        finally:
            after = snapshot()
            print(f"cycles={cycles} after={after}", flush=True)
        if (before[:3] != after[:3] or any((after[3][a] - v) % 2**32 !=
                (cycles if 14 <= a <= 17 else 0) for a, v in before[3].items())):
            raise RuntimeError("HPM epoch or per-axis TX counts differ; not continuing")


def main():
    cli = argparse.ArgumentParser(description="Run from the RK deployment build directory")
    cli.add_argument("profile", type=Path)
    cli.add_argument("--stage", choices=("ready", "once", "endurance", "overnight"), default="ready")
    cli.add_argument("--log-dir", type=Path, required=True)
    args = cli.parse_args()
    args.log_dir.mkdir(parents=True, exist_ok=False)
    profile = yaml.safe_load(args.profile.read_text())
    schedule_file = args.profile.resolve().parent / "rk" / args.profile.name
    period_us = yaml.safe_load(schedule_file.read_text())["pdo"]["period_us"]
    if not isinstance(period_us, int) or period_us <= 0:
        raise ValueError("invalid Profile PDO period")
    import rclpy
    rclpy.init(args=[])
    try:
        if args.stage == "overnight" and period_us != 2000:
            raise ValueError("overnight requires the active 500 Hz bench Profile")
        for cycles in {"ready": (0,), "once": (1,), "endurance": (1, 60_000_000 // period_us),
                       "overnight": (28_800_000_000 // period_us,)}[args.stage]:
            run(profile, cycles, args.log_dir, schedule_file, period_us)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
