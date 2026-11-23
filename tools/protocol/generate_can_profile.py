#!/usr/bin/env python3
"""Compile production CAN and device Profiles into immutable C++ tables."""

import argparse
import hashlib
import json
import re
from pathlib import Path

from check_gateway_contract import identifier, keys, load_yaml, require, uint


def profile_digest(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":")).encode()).digest()


def bounded(value, low, high, name):
    require(type(value) is int and low <= value <= high, f"{name}: expected integer in [{low}, {high}]")
    return value


def pair(value, name, *, minimum=-32768, maximum=32767, crosses_zero=True):
    require(type(value) is list and len(value) == 2, f"{name}: expected [min, max]")
    low, high = (bounded(part, minimum, maximum, name) for part in value)
    require(low < high and (not crosses_zero or low < 0 < high), f"{name}: invalid range")
    return low, high


def model_binding(bindings, name, protocol, motor_type, require_mit=False):
    # YAML reads an unquoted ENCOS short name, such as 7216, as an integer.
    if protocol == "encos" and type(motor_type) is int:
        motor_type = str(motor_type)
    require(type(motor_type) is str and re.fullmatch(r"[A-Za-z0-9]+(?:[._-][A-Za-z0-9]+)*", motor_type),
            f"{name}: invalid model name or alias")
    model = f"dcu_model_{len(bindings)}"
    bindings += [
        f'static constexpr auto *{model} = dcu::motor::{protocol}_config::find_model("{motor_type}");',
        f'static_assert(dcu_model_found({model}), "{name}: unknown or ambiguous {protocol} model {motor_type}");',
    ]
    if require_mit:
        bindings.append(f'static_assert(!dcu_model_found({model}) || {model}->mit.verified(), '
                        f'"{name}: unverified ENCOS MIT encoding");')
    return model


def device_line(device, channels, names, can_ids, slots, bindings, allow_encos_fixture):
    common = {"id", "protocol", "bus", "can_id"}
    protocol = device.get("protocol") if isinstance(device, dict) else None
    require(protocol in ("encos", "bjcx"), "device: unsupported protocol")
    neutral_mit = protocol == "bjcx" and "neutral_mit_only" in device
    fixture = protocol == "encos" and allow_encos_fixture and "type" not in device
    if protocol == "encos":
        fields = {"limits", "encoding", "power_query_period_ms", "model_profile_id" if fixture else "type"}
    elif neutral_mit:
        fields = {"type", "neutral_mit_only", "neutral_mit_frame_limit"}
    else:
        fields = {"type", "status_poll_period_ms", "status_reply_timeout_ms"}
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
        model = model_binding(bindings, name, protocol, device["type"])
        frame_limit = 0
        if neutral_mit:
            require(device["neutral_mit_only"] is True, "neutral_mit_only: expected true")
            frame_limit = bounded(device["neutral_mit_frame_limit"], 2, 900000, "neutral_mit_frame_limit")
            period = timeout = 0
        else:
            timeout = bounded(device["status_reply_timeout_ms"], 2, 20, "status_reply_timeout_ms")
            period = bounded(device["status_poll_period_ms"], 2 * timeout, 1000, "status_poll_period_ms")
        unused_encos = "{ {{0}, {0}, {0}, {0}, {0}, {0}, 0}, 0, 0 }"
        return [f'    {{ "{name}", {bus}U, {can_id}U, 0U, 0U,',
                f'      0, 0, 0U, 0U, 0U, {unused_encos}, {period}U, {timeout}U,',
                f"      DCU_MOTOR_PROTOCOL_BJCX, {model}->part_number.data(), {str(neutral_mit).lower()}, {frame_limit}U }},"]

    if fixture:
        require(type(device["model_profile_id"]) is int and device["model_profile_id"] == 0x8000,
                f"{name}: invalid ENCOS fixture encoding ID")
    else:
        require(channels[bus], f"{name}: named ENCOS encoding requires CAN FD+BRS")
    power = bounded(device["power_query_period_ms"], 0, 1000, "power_query_period_ms")
    require(power == 0 or power >= 20, "power_query_period_ms: expected 0 or 20..1000")
    limits, encoding = device["limits"], device["encoding"]
    scaled_fields = {"position_mrad", "velocity_mrad_s", "torque_mNm", "current_dA"}
    keys(limits, scaled_fields, "limits")
    encoding_keys = {"kp", "kd", "position_mrad", "can_timeout_ms"}
    if fixture:
        encoding_keys |= scaled_fields | {"torque_constant_cNm_A"}
    keys(encoding, encoding_keys, "encoding")
    kp, kd = (pair(encoding[field], field, minimum=0, maximum=65535, crosses_zero=False)
              for field in ("kp", "kd"))
    pos = pair(encoding["position_mrad"], "encoding.position_mrad", minimum=-327680, maximum=327670)
    p_min, p_max = pair(limits["position_mrad"], "limits.position_mrad",
                        minimum=pos[0], maximum=pos[1], crosses_zero=False)
    require(pos[0] <= p_min < p_max <= pos[1], f"{name}: position limits exceed the encoding range")
    if fixture:
        velocity = pair(encoding["velocity_mrad_s"], "encoding.velocity_mrad_s", minimum=-327680, maximum=327670)
        torque = pair(encoding["torque_mNm"], "encoding.torque_mNm", minimum=-3276800, maximum=3276700)
        current = pair(encoding["current_dA"], "encoding.current_dA")
        v_max, t_max, i_max = (min(-low, high) for low, high in (velocity, torque, current))
        kt = f'{bounded(encoding["torque_constant_cNm_A"], 0, 65535, "torque_constant_cNm_A")} / 100.0'
        model_type = "nullptr"
        range_scales = (1000, 1000, 10)
    else:
        model = model_binding(bindings, name, protocol, device["type"], require_mit=True)
        v_max, t_max, i_max = 327670, 3276700, 32767  # Model-specific bounds are checked at initialization.
        velocity, torque, current = ((f"-{model}->mit.{field}", f"{model}->mit.{field}")
                                     for field in ("max_speed_rad_s", "max_torque_Nm", "max_current_A"))
        kt, model_type = f"{model}->mit.torque_constant_Nm_per_A", f"{model}->part_number.data()"
        range_scales = (1, 1, 1)
    v_lim = bounded(limits["velocity_mrad_s"], 1, v_max, "limits.velocity_mrad_s")
    t_lim = bounded(limits["torque_mNm"], 1, t_max, "limits.torque_mNm")
    i_lim = bounded(limits["current_dA"], 1, i_max, "limits.current_dA")
    timeout = bounded(encoding["can_timeout_ms"], 1, 500, "encoding.can_timeout_ms")
    ranges = ", ".join(f"{{ {a} / {scale}.0, {b} / {scale}.0 }}"
                       for (a, b), scale in zip((kp, kd, pos, velocity, torque, current), (1, 1, 1000, *range_scales)))
    return [
        f'    {{ "{name}", {bus}U, {can_id}U, dcu::motor::encos_config::coe_encoding_id, {power}U,',
        f"      {p_min * 1000}, {p_max * 1000}, {v_lim * 1000}U, {t_lim}U, {i_lim}U,",
        f"      {{ {{ {ranges}, {kt} }}, {timeout}U, {int(channels[bus])}U }}, 0U, 0U,",
        f"      DCU_MOTOR_PROTOCOL_ENCOS, {model_type}, false, 0U }},",
    ]


def header(profile, *, allow_encos_fixture=False):
    fields = {"schema_version", "channels", "safety", "devices", "startup"}
    if isinstance(profile, dict) and "passthrough_devices" in profile:
        fields.add("passthrough_devices")
    keys(profile, fields, "production Profile")
    uint(profile["schema_version"], 8, "schema_version", 1)
    require(profile["schema_version"] == 3, "unsupported production Profile schema")
    startup = profile["startup"]
    keys(startup, {"profile_id", "revision", "readiness_ttl_ms", "activation_wait_s"}, "startup")
    profile_id = bounded(startup["profile_id"], 1, 0xFFFFFFFF, "profile_id")
    revision = bounded(startup["revision"], 1, 0xFFFFFFFF, "revision")
    ttl = bounded(startup["readiness_ttl_ms"], 100, 10000, "readiness_ttl_ms")
    wait = bounded(startup["activation_wait_s"], 1, 60, "activation_wait_s")
    digest = profile_digest(profile)
    digest_bytes = ", ".join(f"0x{byte:02x}" for byte in digest)
    safety = profile["safety"]
    keys(safety, {"max_command_lease_ms", "feedback_timeout_ms"}, "safety")
    lease = bounded(safety["max_command_lease_ms"], 2, 10, "max_command_lease_ms")
    feedback = bounded(safety["feedback_timeout_ms"], 5, 20, "feedback_timeout_ms")
    channels = profile["channels"]
    require(type(channels) is list and 1 <= len(channels) <= 4, "channels: expected one to four entries")
    seen = {}
    raw_limits = [0] * 4
    preamble = ["/* Generated by generate_can_profile.py; do not edit. */",
                "#ifndef DCU_CAN_PROFILE_GENERATED_H", "#define DCU_CAN_PROFILE_GENERATED_H",
                '#include "drivers/encos/cfg/encos_models.h"',
                '#include "drivers/bjcx/cfg/bjcx_models.h"',
                "template <typename T>",
                "static constexpr bool dcu_model_found(const T *model) { return model != nullptr; }"]
    bindings = []
    lines = ["static constexpr dcu_can_channel_config_t dcu_can_profile_channels[] = {"]
    for channel in channels:
        fd = type(channel) is dict and channel.get("mode") == "fd"
        fields = {"bus", "mode", "nominal_bitrate_kbps"}
        if fd:
            fields.add("data_bitrate_kbps")
        if type(channel) is dict and "timestamp_eof" in channel:
            fields.add("timestamp_eof")
        if type(channel) is dict and "raw_rate_limit" in channel:
            fields.add("raw_rate_limit")
        keys(channel, fields, "channel")
        bus = channel["bus"]
        uint(bus, 2, "channel.bus")
        require(bus not in seen, f"duplicate CAN bus: {bus}")
        seen[bus] = fd
        raw_limits[bus] = bounded(channel.get("raw_rate_limit", 0), 0, 8, "raw_rate_limit")
        require(channel["mode"] in ("classic", "fd"), "channel.mode: expected classic or fd")
        nominal = channel["nominal_bitrate_kbps"]
        data = channel["data_bitrate_kbps"] if fd else 0
        timestamp_eof = channel.get("timestamp_eof", False)
        require(type(timestamp_eof) is bool, "timestamp_eof: expected a boolean")
        uint(nominal, 16, "nominal_bitrate_kbps", 1)
        if fd:
            uint(data, 16, "data_bitrate_kbps", 1)
        # Current production bus budgets are calibrated for these rates.
        require(nominal == 1000 and (not fd or data == 5000), "bitrate unsupported by the current bus budget")
        lines.append(f"    {{ {bus}U, {nominal * 1000}U, {data * 1000}U, "
                     f"{str(fd).lower()}, {str(timestamp_eof).lower()} }},")
    lines.append("};")
    devices = profile["devices"]
    require(type(devices) is list and len(devices) <= 28, "devices: expected a list of up to 28 entries")
    names, can_ids, slots = set(), set(), {}
    if devices:
        require(not any(raw_limits), "raw-only Profile must not configure motors")
        lines.append("static constexpr dcu_motor_device_profile_t dcu_profile_devices[] = {")
        for device in devices:
            lines.extend(device_line(device, seen, names, can_ids, slots, bindings, allow_encos_fixture))
        neutral_buses = {device["bus"] for device in devices if device.get("neutral_mit_only")}
        require(not neutral_buses or (len(neutral_buses) == 1 and any(
            ch["bus"] in neutral_buses and ch.get("timestamp_eof") for ch in channels)),
            "neutral MIT requires one CAN bus with EOF timestamps")
        lines.append("};")
    passthrough = profile.get("passthrough_devices", [])
    require(type(passthrough) is list, "passthrough_devices: expected a list")
    if passthrough:
        lines.append("static constexpr dcu_passthrough_device_profile_t dcu_profile_passthrough[] = {")
    for device in passthrough:
        keys(device, {"id", "protocol", "type", "bus", "can_id"}, "passthrough device")
        name = device["id"]
        identifier(name, "passthrough device.id")
        require(name not in names, f"duplicate device: {name}")
        names.add(name)
        require(device["protocol"] == "encos", f"{name}: unsupported passthrough protocol")
        bus = bounded(device["bus"], 0, 3, "passthrough device.bus")
        require(seen.get(bus, False) and bus not in slots and not raw_limits[bus],
                f"{name}: passthrough requires a dedicated CAN FD channel")
        can_id = bounded(device["can_id"], 1, 8, "passthrough device.can_id")
        require((bus, can_id) not in can_ids, f"duplicate CAN ID on bus {bus}: {can_id}")
        can_ids.add((bus, can_id))
        model = model_binding(bindings, name, "encos", device["type"])
        lines.append(f"    {{ {bus}U, {can_id}U, DCU_MOTOR_PROTOCOL_ENCOS, {model}->part_number.data() }},")
    if passthrough:
        lines.append("};")
    lines += ["static constexpr dcu_production_profile_t dcu_profile = {",
              f"    dcu_can_profile_channels, {len(channels)}U, "
              f"{'dcu_profile_devices' if devices else 'nullptr'}, {len(devices)}U, {lease}U, {feedback}U,",
              f"    {{ {profile_id}U, {revision}U, {{ {digest_bytes} }}, {ttl}U, {wait}U }},",
              "    { " + ", ".join(f"{limit}U" for limit in raw_limits) + " },",
              f"    {'dcu_profile_passthrough' if passthrough else 'nullptr'}, {len(passthrough)}U,",
              "};", "#endif", ""]
    return "\n".join(preamble + bindings + lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--allow-encos-fixture", action="store_true", help="allow synthetic host test encodings")
    args = parser.parse_args()
    try:
        content = header(load_yaml(args.profile), allow_encos_fixture=args.allow_encos_fixture)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(content, encoding="ascii")
    except (OSError, ValueError) as error:
        parser.exit(1, f"CAN Profile: {error}\n")


if __name__ == "__main__":
    main()
