#!/usr/bin/env python3
"""Compile production CAN and device Profiles into immutable C tables."""

import argparse
import hashlib
import json
from pathlib import Path

from check_gateway_contract import identifier, keys, load_yaml, require, uint


def bounded(value, low, high, name):
    require(type(value) is int and low <= value <= high, f"{name}: expected integer in [{low}, {high}]")
    return value


def pair(value, name, *, minimum=-32768, maximum=32767, crosses_zero=True):
    require(type(value) is list and len(value) == 2, f"{name}: expected [min, max]")
    low, high = (bounded(part, minimum, maximum, name) for part in value)
    require(low < high and (not crosses_zero or low < 0 < high), f"{name}: invalid range")
    return low, high


def device_line(device, channels, names, can_ids, slots):
    common = {"id", "protocol", "bus", "can_id"}
    protocol = device.get("protocol") if isinstance(device, dict) else None
    require(protocol in ("encos", "bjcx"), "device: unsupported protocol")
    fields = ({"model_profile_id", "limits", "encoding", "power_query_period_ms"} if protocol == "encos"
              else {"model", "status_poll_period_ms", "status_reply_timeout_ms"})
    keys(device, common | fields, "device")
    name = device["id"]
    identifier(name, "device.id")
    require(name not in names, f"duplicate device: {name}")
    names.add(name)
    bus = bounded(device["bus"], 0, 3, "device.bus")
    require(bus in channels, f"{name}: CAN bus is not configured")
    can_id = bounded(device["can_id"], 1, 8 if protocol == "encos" else 0x3FF, "device.can_id")
    require((bus, can_id) not in can_ids, f"duplicate CAN ID on bus {bus}: {can_id}")
    can_ids.add((bus, can_id))
    slot = slots.get(bus, 0)
    require(slot < 7, f"bus {bus}: too many devices for the fixed PDO slots")
    slots[bus] = slot + 1
    if protocol == "bjcx":
        require(channels[bus], f"{name}: bjcx requires CAN FD+BRS at 1000/5000 kbps")
        models = {"HRA40H-101": 0x8101, "HRA55H-101": 0x8102}
        model = device["model"]
        require(type(model) is str and model in models, f"{name}: unsupported bjcx model")
        timeout = bounded(device["status_reply_timeout_ms"], 2, 20, "status_reply_timeout_ms")
        period = bounded(device["status_poll_period_ms"], 2 * timeout, 1000, "status_poll_period_ms")
        unused_encos = "{ {{0}, {0}, {0}, {0}, {0}, {0}, 0}, 0, 0 }"
        return [f'    {{ "{name}", {bus}U, {can_id}U, 0x{models[model]:04x}U, 0U,',
                f"      0, 0, 0U, 0U, 0U, {unused_encos}, {period}U, {timeout}U }},"]

    require(type(device["model_profile_id"]) is int and device["model_profile_id"] == 0x8000,
            f"{name}: unsupported ENCOS model")
    power = bounded(device["power_query_period_ms"], 0, 1000, "power_query_period_ms")
    require(power == 0 or power >= 20, "power_query_period_ms: expected 0 or 20..1000")
    limits, encoding = device["limits"], device["encoding"]
    scaled_fields = {"position_mrad", "velocity_mrad_s", "torque_mNm", "current_dA"}
    keys(limits, scaled_fields, "limits")
    keys(encoding, scaled_fields | {"kp", "kd", "torque_constant_cNm_A", "can_timeout_ms"}, "encoding")
    kp, kd = (pair(encoding[field], field, minimum=0, maximum=65535, crosses_zero=False)
              for field in ("kp", "kd"))
    pos = pair(encoding["position_mrad"], "encoding.position_mrad", minimum=-327680, maximum=327670)
    velocity = pair(encoding["velocity_mrad_s"], "encoding.velocity_mrad_s", minimum=-327680, maximum=327670)
    torque = pair(encoding["torque_mNm"], "encoding.torque_mNm", minimum=-3276800, maximum=3276700)
    current = pair(encoding["current_dA"], "encoding.current_dA")
    p_min, p_max = pair(limits["position_mrad"], "limits.position_mrad",
                        minimum=pos[0], maximum=pos[1], crosses_zero=False)
    require(pos[0] <= p_min < p_max <= pos[1], f"{name}: position limits exceed the encoding range")
    v_lim = bounded(limits["velocity_mrad_s"], 1, min(-velocity[0], velocity[1]), "limits.velocity_mrad_s")
    t_lim = bounded(limits["torque_mNm"], 1, min(-torque[0], torque[1]), "limits.torque_mNm")
    i_lim = bounded(limits["current_dA"], 1, min(-current[0], current[1]), "limits.current_dA")
    kt = bounded(encoding["torque_constant_cNm_A"], 0, 65535, "torque_constant_cNm_A")
    timeout = bounded(encoding["can_timeout_ms"], 1, 500, "encoding.can_timeout_ms")
    ranges = ", ".join(f"{{ {a}.0 / {scale}.0, {b}.0 / {scale}.0 }}"
                       for (a, b), scale in ((kp, 1), (kd, 1), (pos, 1000), (velocity, 1000),
                                             (torque, 1000), (current, 10)))
    return [
        f'    {{ "{name}", {bus}U, {can_id}U, 0x8000U, {power}U,',
        f"      {p_min * 1000}, {p_max * 1000}, {v_lim * 1000}U, {t_lim}U, {i_lim}U,",
        f"      {{ {{ {ranges}, {kt}.0 / 100.0 }}, {timeout}U, {int(channels[bus])}U }}, 0U, 0U }},",
    ]


def header(profile):
    keys(profile, {"schema_version", "channels", "safety", "devices", "startup"}, "production Profile")
    uint(profile["schema_version"], 8, "schema_version", 1)
    require(profile["schema_version"] == 2, "unsupported production Profile schema")
    startup = profile["startup"]
    keys(startup, {"profile_id", "revision", "readiness_ttl_ms", "activation_wait_s"}, "startup")
    profile_id = bounded(startup["profile_id"], 1, 0xFFFFFFFF, "profile_id")
    revision = bounded(startup["revision"], 1, 0xFFFFFFFF, "revision")
    ttl = bounded(startup["readiness_ttl_ms"], 100, 10000, "readiness_ttl_ms")
    wait = bounded(startup["activation_wait_s"], 1, 60, "activation_wait_s")
    digest = hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()).digest()
    digest_bytes = ", ".join(f"0x{byte:02x}" for byte in digest)
    safety = profile["safety"]
    keys(safety, {"max_command_lease_ms", "feedback_timeout_ms"}, "safety")
    lease = bounded(safety["max_command_lease_ms"], 2, 10, "max_command_lease_ms")
    feedback = bounded(safety["feedback_timeout_ms"], 5, 20, "feedback_timeout_ms")
    channels = profile["channels"]
    require(type(channels) is list and 1 <= len(channels) <= 4, "channels: expected one to four entries")
    seen = {}
    lines = ["/* Generated by generate_can_profile.py; do not edit. */",
             "#ifndef DCU_CAN_PROFILE_GENERATED_H", "#define DCU_CAN_PROFILE_GENERATED_H",
             "static const dcu_can_channel_config_t dcu_can_profile_channels[] = {"]
    for channel in channels:
        fd = type(channel) is dict and channel.get("mode") == "fd"
        fields = {"bus", "mode", "nominal_bitrate_kbps"}
        if fd:
            fields.add("data_bitrate_kbps")
        keys(channel, fields, "channel")
        bus = channel["bus"]
        uint(bus, 2, "channel.bus")
        require(bus not in seen, f"duplicate CAN bus: {bus}")
        seen[bus] = fd
        require(channel["mode"] in ("classic", "fd"), "channel.mode: expected classic or fd")
        nominal = channel["nominal_bitrate_kbps"]
        data = channel["data_bitrate_kbps"] if fd else 0
        uint(nominal, 16, "nominal_bitrate_kbps", 1)
        if fd:
            uint(data, 16, "data_bitrate_kbps", 1)
        # Current production bus budgets are calibrated for these rates.
        require(nominal == 1000 and (not fd or data == 5000), "bitrate unsupported by the current bus budget")
        lines.append(f"    {{ {bus}U, {nominal * 1000}U, {data * 1000}U, {'true' if fd else 'false'}, false }},")
    lines.append("};")
    devices = profile["devices"]
    require(type(devices) is list and len(devices) <= 28, "devices: expected a list of up to 28 entries")
    if devices:
        lines.append("static const dcu_motor_device_profile_t dcu_profile_devices[] = {")
        names, can_ids, slots = set(), set(), {}
        for device in devices:
            lines.extend(device_line(device, seen, names, can_ids, slots))
        lines.append("};")
    lines += ["static const dcu_production_profile_t dcu_profile = {",
              f"    dcu_can_profile_channels, {len(channels)}U, "
              f"{'dcu_profile_devices' if devices else 'NULL'}, {len(devices)}U, {lease}U, {feedback}U,",
              f"    {{ {profile_id}U, {revision}U, {{ {digest_bytes} }}, {ttl}U, {wait}U }},",
              "};", "#endif", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        content = header(load_yaml(args.profile))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="ascii")
    except (OSError, ValueError) as error:
        parser.exit(1, f"CAN Profile: {error}\n")


if __name__ == "__main__":
    main()
