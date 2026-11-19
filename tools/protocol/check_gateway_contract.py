#!/usr/bin/env python3
"""Read-only validation of the isolated bench contract, not the firmware ABI."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import sys

import yaml

CFG = Path(__file__).resolve().parent / "cfg"
DEFAULT_PROFILE = CFG / "\u6700\u5c0f\u56de\u73af\u914d\u7f6e.yaml"
DEFAULT_CONTRACT = CFG / "\u6700\u5c0f\u7f51\u5173\u5951\u7ea6.yaml"
DEFAULT_VECTORS = CFG / "\u6700\u5c0f\u56de\u73af\u6d4b\u8bd5\u5411\u91cf.yaml"
FORMATS = {"u8": "B", "u16": "H", "u32": "I", "u64": "Q", "bytes32": "32s"}
RECORDS = {"can_echo", "rxpdo", "txpdo", "status", "activate", "activation_result"}
ENUMS = {
    "states": ("BOOT", "PREPARING", "READY", "RUNNING", "HOLD", "FAULT"),
    "activation_results": ("ACCEPTED", "NOT_READY", "CONFIG_MISMATCH", "STALE_CONTEXT", "BUSY", "INVALID"),
    "reasons": ("NONE", "CONFIG_INVALID", "CAN_OFFLINE", "FEEDBACK_TIMEOUT", "MASTER_TIMEOUT",
                "PROTOCOL_MISMATCH", "INVALID_COMMAND", "BUSY", "COUNTER_EXHAUSTED"),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def keys(value, expected, where):
    require(type(value) is dict, f"{where}: expected object")
    require(set(value) == set(expected), f"{where}: missing or unknown keys")


def uint(value, bits, where, minimum=0):
    require(type(value) is int and minimum <= value < (1 << bits),
            f"{where}: expected unsigned {bits}-bit integer >= {minimum}")


def identifier(value, where):
    require(type(value) is str and re.fullmatch(r"[a-z][a-z0-9_]{0,63}", value) is not None,
            f"{where}: invalid identifier")


def scaled_uint(value, factor, where):
    uint(value, 32, where, 1)
    result = value * factor
    uint(result, 32, where, 1)
    return result


def hex_bytes(value, size, where):
    require(type(value) is str and re.fullmatch(r"[0-9a-f \t\r\n]+", value) is not None,
            f"{where}: expected lowercase hex bytes")
    try:
        payload = bytes.fromhex(value)
    except ValueError as error:
        raise ValueError(f"{where}: invalid hex byte grouping") from error
    require(len(payload) == size, f"{where}: expected exactly {size} bytes")
    return payload


class BenchLoader(yaml.SafeLoader):
    """A single explicit data tree, without YAML overrides or implicit numeric surprises."""

    def compose_node(self, parent, index):
        event = self.peek_event()
        require(not isinstance(event, yaml.AliasEvent) and event.anchor is None,
                "YAML anchors and aliases are not supported")
        return super().compose_node(parent, index)

    def construct_object(self, node, deep=False):
        prefix = "tag:yaml.org,2002:"
        require(node.tag in {prefix + kind for kind in ("map", "seq", "str", "int", "bool")},
                f"unsupported YAML type: {node.tag}")
        if node.tag in (prefix + "bool", prefix + "int"):
            require(isinstance(node, yaml.ScalarNode), "YAML numbers and booleans must be scalars")
        if node.tag == prefix + "bool":
            require(node.value in ("true", "false"), "YAML booleans must be true or false")
        elif node.tag == prefix + "int":
            require(re.fullmatch(r"(?:0|-?[1-9][0-9]*|0x[0-9a-fA-F]+)", node.value) is not None,
                    "YAML integers must be decimal or 0x hexadecimal, without leading zeros")
        return super().construct_object(node, deep=deep)

    def construct_mapping(self, node, deep=False):
        require(isinstance(node, yaml.MappingNode), "expected YAML mapping")
        result = {}
        for key_node, value_node in node.value:
            name = self.construct_object(key_node, deep=deep)
            require(type(name) is str, "YAML keys must be strings")
            require(name not in result, f"duplicate YAML key: {name}")
            result[name] = self.construct_object(value_node, deep=deep)
        return result


def load_yaml(path):
    try:
        return yaml.load(Path(path).read_text(encoding="utf-8"), Loader=BenchLoader)
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML: {error}") from error


def validate_contract(contract):
    keys(contract, {"schema_version", "layout_id", "byte_order", "records", *ENUMS}, "contract")
    uint(contract["schema_version"], 16, "contract.schema_version", 1)
    require(contract["schema_version"] == 1 and contract["layout_id"] == "gateway_probe_v1"
            and contract["byte_order"] == "little", "unsupported bench contract")
    for name, labels in ENUMS.items():
        keys(contract[name], labels, name)
        for ordinal, label in enumerate(labels):
            uint(contract[name][label], 8, f"{name}.{label}")
            require(contract[name][label] == ordinal, f"{name}.{label}: unsupported code")
    keys(contract["records"], RECORDS, "records")
    for name, record in contract["records"].items():
        keys(record, {"size_bytes", "fields"}, name)
        uint(record["size_bytes"], 16, name, 1)
        fields = record["fields"]
        require(type(fields) is list and 1 <= len(fields) <= 32, f"{name}: invalid fields")
        offset, names = 0, set()
        for field in fields:
            keys(field, {"name", "type", "offset"}, name)
            identifier(field["name"], name)
            require(field["name"] not in names, f"{name}: duplicate field")
            names.add(field["name"])
            kind = field["type"]
            require(type(kind) is str and kind in FORMATS, f"{name}: unsupported field type")
            uint(field["offset"], 16, name)
            require(field["offset"] == offset, f"{name}: gap or overlapping offset")
            offset += struct.calcsize("<" + FORMATS[kind])
        require(offset == record["size_bytes"], f"{name}: size mismatch")
    require(contract["records"]["can_echo"]["size_bytes"] == 8, "CAN payload must be 8 bytes")


def validate_profile(profile, contract):
    """Validate unit-suffixed integer fields and return canonical bps/us data without mutation."""
    keys(profile, {"schema_version", "profile_id", "revision", "layout_id", "scope",
                   "activation_required", "persistent", "channels", "timing"}, "profile")
    for name in ("schema_version", "profile_id", "revision"):
        uint(profile[name], 32, name, 1)
    require(profile["schema_version"] == 1 and profile["layout_id"] == contract["layout_id"],
            "unsupported profile schema or layout")
    require(profile["scope"] == "isolated_bench" and profile["activation_required"] is True
            and profile["persistent"] is False, "bench scope/activation/persistence mismatch")
    channels = profile["channels"]
    require(type(channels) is list and len(channels) == 1, "bench requires one CAN channel")
    channel = channels[0]
    keys(channel, {"bus", "mode", "nominal_bitrate_kbps", "devices"}, "channel")
    uint(channel["bus"], 2, "bus")
    bitrate = scaled_uint(channel["nominal_bitrate_kbps"], 1000, "nominal_bitrate_kbps")
    require(channel["mode"] == "classic" and bitrate == 1000000,
            "bench requires classic CAN at 1 Mbps")
    devices = channel["devices"]
    require(type(devices) is list and len(devices) == 1, "bench requires one echo peer")
    device = devices[0]
    keys(device, {"id", "protocol", "tx_id", "rx_id", "required"}, "device")
    identifier(device["id"], "device.id")
    require(device["protocol"] == "echo_u64_v1" and device["required"] is True,
            "bench requires an echo_u64_v1 peer")
    for name in ("tx_id", "rx_id"):
        uint(device[name], 11, name)
    require(device["tx_id"] != device["rx_id"], "request and reply IDs must differ")
    timing = profile["timing"]
    timing_fields = {
        "pdo_period_ms": ("pdo_period_us", 1000),
        "exchange_interval_ms": ("exchange_interval_us", 1000),
        "reply_timeout_ms": ("reply_timeout_us", 1000),
        "readiness_ttl_ms": ("readiness_ttl_us", 1000),
        "command_timeout_ms": ("command_timeout_us", 1000),
        "activation_wait_s": ("activation_wait_us", 1000000),
    }
    keys(timing, timing_fields, "timing")
    timing = {target: scaled_uint(timing[source], factor, source)
              for source, (target, factor) in timing_fields.items()}
    period, interval, reply = (timing[name] for name in
                               ("pdo_period_us", "exchange_interval_us", "reply_timeout_us"))
    require(period == 1000, "bench PDO period must be 1000 us")
    require(interval >= period and interval % period == 0 and reply < interval,
            "exchange interval must be a PDO multiple and exceed reply timeout")
    require(timing["readiness_ttl_us"] >= interval + reply, "readiness TTL too short")
    require(timing["command_timeout_us"] >= 2 * period, "command timeout too short")
    require(timing["activation_wait_us"] > timing["readiness_ttl_us"], "activation wait too short")
    channel = dict(channel)
    del channel["nominal_bitrate_kbps"]
    channel["nominal_bitrate"] = bitrate
    return {**profile, "channels": [channel], "timing": timing}


def record_codec(contract, name):
    require(type(name) is str and name in contract["records"], "unknown record")
    fields = contract["records"][name]["fields"]
    return fields, struct.Struct("<" + "".join(FORMATS[field["type"]] for field in fields))


def pack_record(contract, name, values):
    """Encode widths only; live state, sequence and authorization checks are not implemented."""
    fields, codec = record_codec(contract, name)
    keys(values, (field["name"] for field in fields), name)
    packed = []
    for field in fields:
        value, kind = values[field["name"]], field["type"]
        if kind == "bytes32":
            value = hex_bytes(value, 32, f"{name}.{field['name']}")
        else:
            uint(value, struct.calcsize("<" + FORMATS[kind]) * 8, f"{name}.{field['name']}")
        packed.append(value)
    return codec.pack(*packed)


def unpack_record(contract, name, payload):
    fields, codec = record_codec(contract, name)
    require(type(payload) is bytes and len(payload) == codec.size, f"{name}: wrong payload size")
    return {field["name"]: value.hex() if field["type"] == "bytes32" else value
            for field, value in zip(fields, codec.unpack(payload))}


def validate_vectors(vectors, contract):
    keys(vectors, {"schema_version", "cases"}, "vectors")
    uint(vectors["schema_version"], 16, "vectors.schema_version", 1)
    require(vectors["schema_version"] == 1, "unsupported vector schema")
    require(type(vectors["cases"]) is list and 1 <= len(vectors["cases"]) <= 64, "invalid vector count")
    names, covered = set(), set()
    for case in vectors["cases"]:
        keys(case, {"name", "record", "values", "hex"}, "vector")
        identifier(case["name"], "vector.name")
        require(case["name"] not in names, "duplicate vector name")
        names.add(case["name"])
        packed = pack_record(contract, case["record"], case["values"])
        expected = hex_bytes(case["hex"], len(packed), case["name"])
        require(packed == expected, f"{case['name']}: golden bytes mismatch")
        values = dict(case["values"])
        for field in contract["records"][case["record"]]["fields"]:
            if field["type"] == "bytes32":
                values[field["name"]] = hex_bytes(values[field["name"]], 32, case["name"]).hex()
        require(unpack_record(contract, case["record"], packed) == values, "decode mismatch")
        covered.add(case["record"])
    require(covered == RECORDS, "vectors must cover every record")


def profile_digest(profile, contract):
    validate_contract(contract)
    profile = validate_profile(profile, contract)
    payload = json.dumps({"contract": contract, "profile": profile}, sort_keys=True,
                         separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--contract", type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument("--vectors", type=Path, default=DEFAULT_VECTORS)
    parser.add_argument("--show-digest", action="store_true", help="print the full configuration SHA-256")
    args = parser.parse_args(argv)
    try:
        profile, contract, vectors = (load_yaml(path) for path in (args.profile, args.contract, args.vectors))
        digest = profile_digest(profile, contract)
        validate_vectors(vectors, contract)
    except (OSError, UnicodeError, ValueError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("PASS: isolated bench description and byte vectors only; no firmware or hardware tested.")
    print(f"Profile {profile['profile_id']} revision {profile['revision']}")
    if args.show_digest:
        print(f"SHA-256 {digest}")
    print(", ".join(f"{name}={record['size_bytes']} B" for name, record in contract["records"].items()))
    print(f"Golden vectors: {len(vectors['cases'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
