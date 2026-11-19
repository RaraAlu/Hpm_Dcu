import copy
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/protocol/check_gateway_contract.py"
spec = importlib.util.spec_from_file_location("dcu_bench_contract", SCRIPT)
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


def changed(source, path, value):
    result = copy.deepcopy(source)
    target = result
    for part in path[:-1]:
        target = target[part]
    target[path[-1]] = value
    return result


class GatewayContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = bench.load_yaml(bench.DEFAULT_PROFILE)
        cls.contract = bench.load_yaml(bench.DEFAULT_CONTRACT)
        cls.vectors = bench.load_yaml(bench.DEFAULT_VECTORS)

    def test_checked_in_bench_description_and_vectors(self):
        bench.validate_contract(self.contract)
        bench.validate_profile(self.profile, self.contract)
        bench.validate_vectors(self.vectors, self.contract)
        self.assertEqual(len(self.vectors["cases"]), 8)

    def test_layout_sizes_offsets_and_unaligned_fields(self):
        expected = {
            "can_echo": (8, "II", [0, 4]),
            "rxpdo": (32, "HHQIIIQ", [0, 2, 4, 12, 16, 20, 24]),
            "txpdo": (48, "HBBQIIIIIIIII", [0, 2, 3, 4, 12, 16, 20, 24, 28, 32, 36, 40, 44]),
            "status": (80, "HBBII32sQIIQIII", [0, 2, 3, 4, 8, 12, 44, 52, 56, 60, 68, 72, 76]),
            "activate": (64, "II32sQIIII", [0, 4, 8, 40, 48, 52, 56, 60]),
            "activation_result": (16, "IHBBQ", [0, 4, 6, 7, 8]),
        }
        for name, (size, format_, offsets) in expected.items():
            with self.subTest(record=name):
                fields, codec = bench.record_codec(self.contract, name)
                self.assertEqual(codec.format, "<" + format_)
                self.assertEqual(codec.size, size)
                self.assertEqual(self.contract["records"][name]["size_bytes"], size)
                self.assertEqual([field["offset"] for field in fields], offsets)

    def test_pdo_can_mapping_has_independent_byte_expectations(self):
        cases = {case["name"]: case for case in self.vectors["cases"]}
        payloads = {name: bench.pack_record(self.contract, cases[name]["record"], cases[name]["values"])
                    for name in ("pdo_request", "pdo_feedback", "can_asymmetric")}
        expected = bytes.fromhex("4433221104030201")
        self.assertEqual(payloads["can_asymmetric"], expected)
        self.assertEqual(payloads["pdo_request"][16:24], expected)
        self.assertEqual(payloads["pdo_feedback"][20:28], expected)
        self.assertEqual(struct.unpack_from("<Q", payloads["pdo_request"], 4)[0], 0x0102030405060708)
        self.assertEqual(struct.unpack_from("<I", payloads["pdo_feedback"], 16)[0], 0x10203040)

    def test_invalid_profile_fields_topology_and_timing(self):
        cases = [
            (("extra",), 1), (("schema_version",), True), (("schema_version",), 2),
            (("profile_id",), 0), (("revision",), -1), (("layout_id",), "production"),
            (("scope",), "production"), (("activation_required",), 1), (("persistent",), True),
            (("channels",), []), (("channels",), [self.profile["channels"][0]] * 2),
            (("channels", 0, "bus"), 4), (("channels", 0, "bus"), False),
            (("channels", 0, "mode"), "fd"), (("channels", 0, "nominal_bitrate_kbps"), 500),
            (("channels", 0, "devices"), []),
            (("channels", 0, "devices", 0, "protocol"), "encos"),
            (("channels", 0, "devices", 0, "required"), False),
            (("channels", 0, "devices", 0, "tx_id"), 0x800),
            (("channels", 0, "devices", 0, "rx_id"), 0x600),
            (("timing", "pdo_period_ms"), 0), (("timing", "pdo_period_ms"), 2),
            (("timing", "exchange_interval_ms"), 5),
            (("timing", "reply_timeout_ms"), 10),
            (("timing", "readiness_ttl_ms"), 1),
            (("timing", "readiness_ttl_ms"), 5000),
            (("timing", "command_timeout_ms"), 1),
            (("timing", "activation_wait_s"), 0),
        ]
        for path, value in cases:
            with self.subTest(path=path, value=value), self.assertRaises(ValueError):
                bench.validate_profile(changed(self.profile, path, value), self.contract)

    def test_contract_rejects_gaps_overlap_duplicates_types_and_sizes(self):
        cases = [
            (("extra",), 0), (("byte_order",), "big"), (("schema_version",), True),
            (("states", "READY"), 3), (("states", "BOOT"), False),
            (("records", "can_echo", "size_bytes"), 7),
            (("records", "rxpdo", "fields", 1, "offset"), 1),
            (("records", "rxpdo", "fields", 1, "offset"), 3),
            (("records", "rxpdo", "fields", 1, "offset"), 2.0),
            (("records", "rxpdo", "fields", 1, "name"), "layout_version"),
            (("records", "rxpdo", "fields", 1, "type"), "native_int"),
            (("records", "rxpdo", "fields", 1, "type"), []),
            (("records", "rxpdo", "fields"), []),
        ]
        for path, value in cases:
            with self.subTest(path=path, value=value), self.assertRaises(ValueError):
                bench.validate_contract(changed(self.contract, path, value))

    def test_missing_keys_are_rejected(self):
        for name in self.profile:
            profile = copy.deepcopy(self.profile)
            del profile[name]
            with self.subTest(key=name), self.assertRaises(ValueError):
                bench.validate_profile(profile, self.contract)

    def test_integer_boundaries_and_exact_value_keys(self):
        for value in (0, 1, 0xFFFFFFFF):
            expected = struct.pack("<II", value, value)
            self.assertEqual(bench.pack_record(self.contract, "can_echo", {
                "probe_tag": value, "probe_seq": value}), expected)
        for value in (-1, 1 << 32, True, 1.0, "1", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                bench.pack_record(self.contract, "can_echo", {"probe_tag": 1, "probe_seq": value})
        for values in ({"probe_tag": 1}, {"probe_tag": 1, "probe_seq": 2, "unknown": 3}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                bench.pack_record(self.contract, "can_echo", values)

    def test_exact_lengths_on_decode_for_every_record(self):
        for case in self.vectors["cases"]:
            payload = bytes.fromhex(case["hex"])
            for bad in (payload[:-1], payload + b"\x00"):
                with self.subTest(case=case["name"], length=len(bad)), self.assertRaises(ValueError):
                    bench.unpack_record(self.contract, case["record"], bad)

    def test_digest_octets_are_not_truncated_or_silently_padded(self):
        values = next(case["values"] for case in self.vectors["cases"] if case["record"] == "activate")
        for digest in ("00" * 31, "00" * 33, "AB" * 32, "zz" * 32, 0):
            with self.subTest(digest=digest), self.assertRaises(ValueError):
                bench.pack_record(self.contract, "activate", {**values, "profile_digest": digest})

    def test_digest_is_canonical_and_binds_profile_and_layout(self):
        digest = bench.profile_digest(self.profile, self.contract)
        self.assertEqual(digest, "da33ce40d518cadb26b0e5b5cedbe9b9b83eaa5d9e908121475be699de06352f")
        reordered = json.loads(json.dumps(self.profile, sort_keys=True, indent=4))
        self.assertEqual(bench.profile_digest(reordered, self.contract), digest)
        for path, value in ((("revision",), 2), (("channels", 0, "bus"), 1),
                            (("channels", 0, "devices", 0, "tx_id"), 0x610),
                            (("timing", "reply_timeout_ms"), 4)):
            with self.subTest(path=path):
                self.assertNotEqual(bench.profile_digest(changed(self.profile, path, value), self.contract), digest)
        contract = copy.deepcopy(self.contract)
        contract["records"]["rxpdo"]["fields"].append({"name": "future", "type": "u32", "offset": 32})
        contract["records"]["rxpdo"]["size_bytes"] = 36
        self.assertNotEqual(bench.profile_digest(self.profile, contract), digest)

    def test_unit_suffixes_normalize_exactly_without_mutating_source(self):
        before = copy.deepcopy(self.profile)
        profile = bench.validate_profile(self.profile, self.contract)
        self.assertEqual(self.profile, before)
        self.assertEqual(profile["channels"][0]["nominal_bitrate"], 1000000)
        self.assertNotIn("nominal_bitrate_kbps", profile["channels"][0])
        self.assertEqual(profile["timing"], {
            "pdo_period_us": 1000, "exchange_interval_us": 10000, "reply_timeout_us": 5000,
            "readiness_ttl_us": 50000, "command_timeout_us": 5000, "activation_wait_us": 5000000,
        })

    def test_unit_fields_reject_non_integer_values_and_overflow(self):
        paths = [("channels", 0, "nominal_bitrate_kbps")]
        paths.extend(("timing", name) for name in self.profile["timing"])
        for path in paths:
            factor = 1000000 if path[-1] == "activation_wait_s" else 1000
            for value in (0, -1, True, None, 1.0, "1", "1 ms", "5 s", "1000 kbps",
                          (1 << 32) // factor + 1, 1 << 32):
                with self.subTest(path=path, value=value), self.assertRaises(ValueError):
                    bench.validate_profile(changed(self.profile, path, value), self.contract)

    def test_integer_scaling_boundaries(self):
        for factor in (1000, 1000000):
            maximum = 0xFFFFFFFF // factor
            with self.subTest(factor=factor):
                self.assertEqual(bench.scaled_uint(1, factor, "value"), factor)
                self.assertEqual(bench.scaled_uint(maximum, factor, "value"), maximum * factor)
                with self.assertRaises(ValueError):
                    bench.scaled_uint(maximum + 1, factor, "value")

    def test_unit_fields_reject_legacy_names_and_alternate_suffixes(self):
        cases = [(("channels", 0), "nominal_bitrate_kbps", alias)
                 for alias in ("nominal_bitrate", "nominal_bitrate_k", "nominal_bitrate_bps")]
        for name in self.profile["timing"]:
            base = name.rsplit("_", 1)[0]
            cases.extend((("timing",), name, alias) for alias in (base, base + "_us"))
        cases.extend([(("timing",), "pdo_period_ms", "pdo_period_s"),
                      (("timing",), "activation_wait_s", "activation_wait_ms")])
        for path, name, alias in cases:
            for keep_original in (False, True):
                profile = copy.deepcopy(self.profile)
                target = profile
                for part in path:
                    target = target[part]
                target[alias] = target[name]
                if not keep_original:
                    del target[name]
                with self.subTest(alias=alias, keep_original=keep_original), self.assertRaises(ValueError):
                    bench.validate_profile(profile, self.contract)

    def test_grouped_hex_preserves_bytes_and_rejects_split_octets(self):
        self.assertEqual(bench.hex_bytes("44 33 22 11\n04 03 02 01", 8, "payload"),
                         bytes.fromhex("4433221104030201"))
        values = next(case["values"] for case in self.vectors["cases"] if case["record"] == "activate")
        compact = {**values, "profile_digest": bytes.fromhex(values["profile_digest"]).hex()}
        self.assertEqual(bench.pack_record(self.contract, "activate", values),
                         bench.pack_record(self.contract, "activate", compact))
        for value in ("0 0", "00,00", "0x00", "AA", "gg", "", " ", 0):
            with self.subTest(value=value), self.assertRaises(ValueError):
                bench.hex_bytes(value, 1, "payload")

    def test_duplicate_yaml_keys_fail_at_every_level(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.yaml"
            for text in ("x: 1\nx: 2\n", "x:\n  a: 1\n  a: 2\n", '{"x":1,"x":2}'):
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaises(ValueError):
                    bench.load_yaml(path)

    def test_yaml_comments_hex_and_key_order_preserve_the_digest(self):
        content = bench.DEFAULT_PROFILE.read_text(encoding="utf-8")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.yaml"
            for text in (content, "# bench profile\n" + content,
                         content.replace("0x600", "1536").replace("0x601", "1537"),
                         content.replace("profile_id: 1\nrevision: 1", "revision: 1\nprofile_id: 1")):
                path.write_text(text, encoding="utf-8")
                profile = bench.load_yaml(path)
                self.assertEqual(profile, self.profile)
                self.assertEqual(bench.profile_digest(profile, self.contract),
                                 "da33ce40d518cadb26b0e5b5cedbe9b9b83eaa5d9e908121475be699de06352f")

    def test_yaml_rejects_anchors_merges_tags_and_multiple_documents(self):
        samples = (
            "x: &value 1\ny: *value\n", "x: *missing\n", "x: &loop [*loop]\n",
            "x:\n  <<: {a: 1}\n  a: 2\n", "x: !custom 1\n",
            "x: !!python/object/apply:os.system ['exit 1']\n",
            "x: 1\n---\nx: 2\n", "[x]: 1\n", "true: 1\n",
            "!!map []", "x: !!int []", "x: !!bool {}",
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.yaml"
            for text in samples:
                path.write_text(text, encoding="utf-8")
                with self.subTest(text=text), self.assertRaises(ValueError):
                    bench.load_yaml(path)

    def test_yaml_rejects_ambiguous_scalars_and_non_finite_numbers(self):
        samples = ("yes", "no", "on", "off", "True", "01", "1:00", "0b10", "+1", "1_000",
                   "1.0", ".nan", ".inf", "null", "2026-11-17")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "input.yaml"
            for scalar in samples:
                path.write_text(f"x: {scalar}\n", encoding="utf-8")
                with self.subTest(scalar=scalar), self.assertRaises(ValueError):
                    bench.load_yaml(path)
            path.write_text('enabled: true\ndisabled: false\nzero: 0\nnegative: -1\nlabel: "on"\n',
                            encoding="utf-8")
            self.assertEqual(bench.load_yaml(path), {
                "enabled": True, "disabled": False, "zero": 0, "negative": -1, "label": "on"})

    def test_trial_inputs_are_yaml_only_and_hex_vectors_remain_strings(self):
        for path in (bench.DEFAULT_PROFILE, bench.DEFAULT_CONTRACT, bench.DEFAULT_VECTORS):
            with self.subTest(path=path):
                self.assertEqual(path.suffix, ".yaml")
                self.assertFalse(path.with_suffix(".json").exists())
        for case in self.vectors["cases"]:
            self.assertIs(type(case["hex"]), str)
        zeros = next(case["hex"] for case in self.vectors["cases"] if case["name"] == "can_zero")
        self.assertEqual(bytes.fromhex(zeros), bytes(8))

    def test_vectors_require_golden_bytes_unique_names_and_full_coverage(self):
        cases = [
            changed(self.vectors, ("cases", 0, "hex"), "00" * 8),
            changed(self.vectors, ("cases", 1, "name"), "can_asymmetric"),
            changed(self.vectors, ("cases", 0, "record"), "unknown"),
            changed(self.vectors, ("cases",), self.vectors["cases"][:-1]),
        ]
        for vectors in cases:
            with self.subTest(vectors=vectors), self.assertRaises(ValueError):
                bench.validate_vectors(vectors, self.contract)

    def test_cli_from_another_directory_is_read_only(self):
        paths = [bench.DEFAULT_PROFILE, bench.DEFAULT_CONTRACT, bench.DEFAULT_VECTORS]
        for directory in ("firmware/generated", "protocol/ethercat"):
            paths.extend(path for path in (ROOT / directory).rglob("*") if path.is_file())
        before = {path: path.read_bytes() for path in paths}
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, str(SCRIPT)], cwd=directory,
                                    capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("no firmware or hardware tested", result.stdout)
            self.assertNotIn("SHA-256", result.stdout)
            self.assertEqual(list(Path(directory).iterdir()), [])
        self.assertEqual(before, {path: path.read_bytes() for path in paths})

    def test_full_digest_is_available_only_when_requested(self):
        result = subprocess.run([sys.executable, str(SCRIPT), "--show-digest"],
                                capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("SHA-256 " + bench.profile_digest(self.profile, self.contract), result.stdout)

    def test_cli_rejects_bad_input_without_traceback(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            for text in ("[]", "x: [", "schema_version: true\n", "x: 1\nx: 2\n",
                         "x: .nan\n", "x: !!int []", "!!map []", ""):
                path.write_text(text, encoding="utf-8")
                result = subprocess.run([sys.executable, str(SCRIPT), "--profile", str(path)],
                                        capture_output=True, text=True, timeout=10)
                with self.subTest(text=text):
                    self.assertEqual(result.returncode, 1)
                    self.assertIn("FAIL:", result.stderr)
                    self.assertNotIn("Traceback", result.stderr)


if __name__ == "__main__":
    unittest.main()
