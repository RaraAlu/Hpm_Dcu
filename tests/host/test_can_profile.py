import copy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/protocol"))
from generate_can_profile import header, load_yaml


class CanProfileTest(unittest.TestCase):
    def setUp(self):
        self.profile = load_yaml(ROOT / "firmware/app/profile/cfg/生产CAN配置.yaml")

    def test_production_profile_uses_only_can3_and_single_shot(self):
        self.assertEqual([channel["bus"] for channel in self.profile["channels"]], [3])
        self.assertEqual(len(self.profile["devices"]), 1)
        result = header(self.profile)
        self.assertIn("{ 3U, 1000000U, 5000000U, true, true }", result)
        self.assertIn('"hra55h100_can3", 3U, 2U, 0U', result)
        self.assertIn('DCU_MOTOR_PROTOCOL_BJCX, "HRA55H-100", true, 30000U', result)

    def test_neutral_mit_budget_is_mandatory_bounded_and_not_single_frame(self):
        device = self.profile["devices"][0]
        for limit in (0, 1, True, 900001):
            with self.subTest(limit=limit), self.assertRaisesRegex(ValueError, "neutral_mit_frame_limit"):
                header(self.profile | {"devices": [device | {"neutral_mit_frame_limit": limit}]})
        for limit in (2, 900000):
            self.assertIn(f'true, {limit}U', header(self.profile | {"devices": [device | {"neutral_mit_frame_limit": limit}]}))
        del device["neutral_mit_frame_limit"]
        with self.assertRaises(ValueError):
            header(self.profile)

    def test_classic_has_no_data_bitrate(self):
        self.profile["channels"] = [{"bus": 1, "mode": "classic", "nominal_bitrate_kbps": 1000}]
        self.profile["devices"] = []
        self.assertIn("{ 1U, 1000000U, 0U, false, false }", header(self.profile))
        self.profile["channels"][0]["data_bitrate_kbps"] = 5000
        with self.assertRaises(ValueError):
            header(self.profile)

    def test_missing_duplicate_unknown_and_unqualified_settings_fail(self):
        original = self.profile["channels"][0]
        for mutation in ({"bus": 4}, {"mode": "loopback"}, {"nominal_bitrate_kbps": 500},
                         {"data_bitrate_kbps": False}, {"enabled": True}, {"single_shot": 1}):
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                header(self.profile | {"channels": [original | mutation]})
        for channels in ([], [original, copy.copy(original)], [
                {"bus": 2, "mode": "fd", "nominal_bitrate_kbps": 1000}]):
            with self.subTest(channels=channels), self.assertRaises(ValueError):
                header(self.profile | {"channels": channels})

    def test_explicit_devices_keep_profile_order_without_pdo_slots(self):
        profile = load_yaml(ROOT / "tests/host/motor/cfg/单路ENCOS验证配置.yaml")
        entry = profile["devices"][0]
        profile["channels"].append({"bus": 0, "mode": "classic", "nominal_bitrate_kbps": 1000})
        profile["devices"] += [entry | {"id": "other_bus", "bus": 0}, entry | {"id": "next", "can_id": 1}]
        result = header(profile)
        self.assertIn('"fixture_encos", 2U, 3U, 0x8000U', result)
        self.assertIn('"other_bus", 0U, 3U, 0x8000U', result)
        self.assertIn('"next", 2U, 1U, 0x8000U', result)
        self.assertLess(result.index('"fixture_encos"'), result.index('"other_bus"'))
        self.assertLess(result.index('"other_bus"'), result.index('"next"'))
        self.assertIn("-9000000, 9000000, 18000000U, 5000U, 60U", result)

    def test_mixed_profile_preserves_bjcx_types_and_validates_shared_bus(self):
        profile = load_yaml(ROOT / "tests/host/motor/cfg/混挂电机验证配置.yaml")
        result = header(profile)
        self.assertIn('"fixture_hra40", 2U, 36U, 0U', result)
        self.assertIn('"fixture_hra55", 0U, 3U, 0U', result)
        for motor_type in ("HRA40H-101", "HRA55H-100"):
            self.assertIn(f'DCU_MOTOR_PROTOCOL_BJCX, "{motor_type}"', result)
        for change in ({"can_id": 3}, {"type": "unknown"}, {"status_poll_period_ms": 10},
                       {"model": "HRA40H-101"}, {"model_profile_id": 0x8000}):
            invalid = copy.deepcopy(profile)
            invalid["devices"][1].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                header(invalid)
        invalid = copy.deepcopy(profile)
        del invalid["devices"][1]["type"]
        with self.assertRaises(ValueError):
            header(invalid)
        profile["channels"][1] = {"bus": 0, "mode": "classic", "nominal_bitrate_kbps": 1000}
        with self.assertRaisesRegex(ValueError, "requires CAN FD"):
            header(profile)

    def test_missing_and_invalid_devices_fail_closed(self):
        profile = load_yaml(ROOT / "tests/host/motor/cfg/单路ENCOS验证配置.yaml")
        entry = profile["devices"][0]
        too_many = [entry | {"id": f"device_{i}", "can_id": i} for i in range(1, 9)]
        for devices in ([entry, entry], [entry, entry | {"id": "duplicate_route"}],
                        [entry | {"bus": 1}], [entry | {"model_profile_id": 0}], too_many,
                        [entry | {"limits": entry["limits"] | {"current_dA": 101}}]):
            with self.subTest(devices=devices), self.assertRaises(ValueError):
                header(profile | {"devices": devices})
        del profile["devices"]
        with self.assertRaises(ValueError):
            header(profile)
