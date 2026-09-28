"""合成反馈的 T1 测试；只导入标准库与新的纯状态模块。"""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import unittest

from nero_runtime.snapshot import build_snapshot


def messages():
    """全部时间都是合成的同一秒数时基，不是现场采样。"""
    return {
        "status": {
            "timestamp_s": 99.9, "ctrl_mode": 1, "arm_status": 0,
            "teach_status": 6, "motion_status": 0, "err_code": 0,
        },
        "joints": {"timestamp_s": 99.9, "joint_rad": [0.1 * i for i in range(7)]},
        "flange": {
            "timestamp_s": 99.9,
            "flange_pose_m_rad": [0.31, 0.02, 0.45, 0.1, -0.2, 0.3],
        },
        "drivers": [
            {"joint": i, "timestamp_s": 99.8, "enabled": True,
             "undervoltage": False, "driver_error": False, "voltage_v": 24.0}
            for i in range(1, 8)
        ],
    }


def snapshot(data=None, **overrides):
    feedback = messages() if data is None else data
    return build_snapshot(
        robot_id=overrides.pop("robot_id", "synthetic-arm"),
        observed_at_s=overrides.pop("observed_at_s", 100.0),
        time_basis=overrides.pop("time_basis", "synthetic_seconds"),
        max_status_age_s=overrides.pop("max_status_age_s", 0.25),
        max_joints_age_s=overrides.pop("max_joints_age_s", 0.25),
        max_flange_age_s=overrides.pop("max_flange_age_s", 0.25),
        max_driver_age_s=overrides.pop("max_driver_age_s", 0.5),
        **(feedback | overrides),
    )


class SnapshotTests(unittest.TestCase):
    def test_fresh_complete_feedback_preserves_fields_units_order_and_times(self):
        result = snapshot()
        self.assertEqual(result.robot_id, "synthetic-arm")
        self.assertEqual(result.time_basis, "synthetic_seconds")
        self.assertEqual((result.control_mode, result.arm_status,
                          result.teach_status, result.motion_status,
                          result.error_code), (1, 0, 6, 0, 0))
        self.assertEqual(result.joint_rad, tuple(0.1 * i for i in range(7)))
        self.assertEqual(result.flange_pose_m_rad,
                         (0.31, 0.02, 0.45, 0.1, -0.2, 0.3))
        self.assertEqual([item.joint for item in result.drivers], list(range(1, 8)))
        self.assertEqual(result.status_quality.timestamp_s, 99.9)
        self.assertEqual(result.joints_quality.timestamp_s, 99.9)
        self.assertEqual(result.flange_quality.timestamp_s, 99.9)
        self.assertEqual([item.quality.timestamp_s for item in result.drivers],
                         [99.8] * 7)
        self.assertTrue(all(item.quality.state == "valid" for item in result.drivers))
        self.assertEqual(result.issues, ())
        json.dumps(asdict(result), allow_nan=False)

    def test_fresh_fault_and_disabled_are_data_not_missing(self):
        data = messages()
        data["status"]["arm_status"] = 6
        data["status"]["err_code"] = 4097
        data["drivers"][2]["enabled"] = False
        data["drivers"][2]["driver_error"] = True
        result = snapshot(data)
        self.assertEqual(result.status_quality.state, "valid")
        self.assertEqual(result.error_code, 4097)
        self.assertEqual(result.arm_status, 6)
        self.assertEqual(result.drivers[2].quality.state, "valid")
        self.assertIs(result.drivers[2].enabled, False)
        self.assertIs(result.drivers[2].driver_error, True)
        self.assertFalse(hasattr(result, "ready_for_motion"))
        self.assertFalse(hasattr(result, "can_move"))

    def test_missing_sources_and_unknown_identity_are_not_filled(self):
        data = {"status": None, "joints": None, "flange": None, "drivers": None}
        result = snapshot(data, robot_id=None)
        self.assertIsNone(result.robot_id)
        self.assertEqual((result.status_quality.state, result.joints_quality.state,
                          result.flange_quality.state), ("missing",) * 3)
        self.assertIsNone(result.error_code)
        self.assertIsNone(result.joint_rad)
        self.assertIsNone(result.flange_pose_m_rad)
        self.assertEqual([d.quality.state for d in result.drivers], ["missing"] * 7)
        self.assertTrue(all(d.enabled is None for d in result.drivers))
        self.assertIn("driver[7]: missing", result.issues)

    def test_partial_missing_only_invalidates_affected_source(self):
        data = messages()
        data["flange"] = None
        data["drivers"] = data["drivers"][:-1]
        result = snapshot(data)
        self.assertEqual(result.status_quality.state, "valid")
        self.assertEqual(result.joints_quality.state, "valid")
        self.assertEqual(result.flange_quality.state, "missing")
        self.assertEqual(result.drivers[6].quality.state, "missing")
        self.assertIsNone(result.drivers[6].enabled)

    def test_bad_vectors_and_types_are_invalid_without_truncation(self):
        for bad in ([0.0] * 6, [0.0] * 8, [0.0] * 6 + [math.nan],
                    [0.0] * 6 + [math.inf], [0.0] * 6 + ["0"], "0000000"):
            with self.subTest(bad=bad):
                data = messages()
                data["joints"]["joint_rad"] = bad
                result = snapshot(data)
                self.assertEqual(result.joints_quality.state, "invalid")
                self.assertIsNone(result.joint_rad)
                self.assertIn("joints: joint_rad missing or invalid", result.issues)
        data = messages()
        data["flange"]["flange_pose_m_rad"] = [0.0] * 7
        self.assertEqual(snapshot(data).flange_quality.state, "invalid")

    def test_malformed_status_does_not_supply_default_healthy_codes(self):
        for field, bad in (("err_code", None), ("ctrl_mode", "1"),
                           ("arm_status", True)):
            with self.subTest(field=field):
                data = messages()
                data["status"][field] = bad
                result = snapshot(data)
                self.assertEqual(result.status_quality.state, "invalid")
                self.assertIsNone(result.error_code)
                self.assertIsNone(result.control_mode)
                self.assertIn(f"status: {field} missing or invalid", result.issues)

    def test_driver_number_identity_missing_duplicate_out_of_range(self):
        data = messages()
        data["drivers"] = [d for d in data["drivers"] if d["joint"] != 4]
        missing = snapshot(data)
        self.assertEqual(missing.drivers[3].quality.state, "missing")
        self.assertIsNone(missing.drivers[3].enabled)

        data = messages()
        data["drivers"][3]["joint"] = 3
        duplicate = snapshot(data)
        self.assertEqual(duplicate.drivers[2].quality.state, "invalid")
        self.assertEqual(duplicate.drivers[3].quality.state, "missing")
        self.assertIn("driver[3]: duplicate joint number", duplicate.issues)

        data = messages()
        data["drivers"][6]["joint"] = 8
        out_of_range = snapshot(data)
        self.assertEqual(out_of_range.drivers[6].quality.state, "missing")
        self.assertIn("drivers[6]: joint 8 outside 1..7", out_of_range.issues)

    def test_invalid_driver_value_is_unknown_not_false(self):
        data = messages()
        data["drivers"][0]["enabled"] = 1
        data["drivers"][1]["voltage_v"] = math.nan
        result = snapshot(data)
        for joint in (0, 1):
            self.assertEqual(result.drivers[joint].quality.state, "invalid")
            self.assertIsNone(result.drivers[joint].enabled)
            self.assertIsNone(result.drivers[joint].voltage_v)
        json.dumps(asdict(result), allow_nan=False)

    def test_age_boundary_expired_future_and_invalid_timestamp(self):
        data = messages()
        data["status"]["timestamp_s"] = 99.75
        self.assertEqual(snapshot(data).status_quality.state, "valid")
        data["status"]["timestamp_s"] = 99.749
        stale = snapshot(data)
        self.assertEqual(stale.status_quality.state, "stale")
        self.assertEqual(stale.status_quality.timestamp_s, 99.749)
        self.assertIsNone(stale.control_mode)
        data["status"]["timestamp_s"] = 100.001
        future = snapshot(data)
        self.assertEqual(future.status_quality.state, "invalid")
        self.assertIn("status: timestamp_s is in the future", future.issues)
        for bad in (None, math.nan, math.inf, "100", -1):
            with self.subTest(timestamp=bad):
                data["status"]["timestamp_s"] = bad
                self.assertEqual(snapshot(data).status_quality.state, "invalid")

    def test_driver_freshness_has_separate_age_limit(self):
        data = messages()
        data["drivers"][0]["timestamp_s"] = 99.5
        self.assertEqual(snapshot(data).drivers[0].quality.state, "valid")
        data["drivers"][0]["timestamp_s"] = 99.499
        result = snapshot(data)
        self.assertEqual(result.drivers[0].quality.state, "stale")
        self.assertIsNone(result.drivers[0].enabled)
        self.assertEqual(result.drivers[1].quality.state, "valid")

    def test_unknown_integer_status_codes_are_preserved_uninterpreted(self):
        data = messages()
        data["status"]["arm_status"] = 731
        data["status"]["motion_status"] = 99
        result = snapshot(data)
        self.assertEqual(result.status_quality.state, "valid")
        self.assertEqual((result.arm_status, result.motion_status), (731, 99))
        self.assertEqual(result.uninterpreted_status_fields,
                         ("arm_status", "motion_status"))

    def test_input_mutation_cannot_change_returned_snapshot(self):
        data = messages()
        result = snapshot(data)
        data["joints"]["joint_rad"][0] = 900.0
        data["flange"]["flange_pose_m_rad"][0] = 900.0
        data["drivers"][0]["enabled"] = False
        self.assertEqual(result.joint_rad[0], 0.0)
        self.assertEqual(result.flange_pose_m_rad[0], 0.31)
        self.assertIs(result.drivers[0].enabled, True)
        with self.assertRaises(AttributeError):
            result.robot_id = "other"

    def test_invalid_call_parameters_raise(self):
        for kwargs in ({"observed_at_s": math.nan}, {"observed_at_s": -1},
                       {"max_status_age_s": -0.01}, {"max_driver_age_s": math.inf},
                       {"robot_id": ""}, {"time_basis": " "}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                snapshot(**kwargs)

    def test_one_sample_claims_no_stability_fk_or_permission(self):
        result = snapshot()
        for absent in ("stable", "joint_variation_0_25s_rad", "fk_error_m",
                       "ready_for_motion", "can_start", "can_drag", "safe"):
            self.assertFalse(hasattr(result, absent), absent)

    def test_clean_interpreter_imports_no_sdk_can_or_experiments(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        code = (
            "import sys; import nero_runtime; import nero_runtime.snapshot; "
            "assert not ({'pyAgxArm', 'can', 'experiments'} & "
            "{name.split('.')[0] for name in sys.modules}); "
            "print('isolated import ok')"
        )
        run = subprocess.run(
            [sys.executable, "-B", "-S", "-c", code], cwd=root,
            env=environment, capture_output=True, text=True, check=False,
        )
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.strip(), "isolated import ok")
        self.assertEqual(run.stderr, "")


if __name__ == "__main__":
    unittest.main()
