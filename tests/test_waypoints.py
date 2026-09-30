"""F1：纯快照到单文件点位；只用合成内存数据和临时目录。"""

import json
import math
from pathlib import Path
import tempfile
import unittest

from nero_runtime.snapshot import build_snapshot
from nero_runtime.waypoints import (
    capture_waypoint, list_waypoints, load_waypoint, save_waypoint,
)


def snapshot(*, joints=None, joint_stamp=99.8, flange=True):
    if joints is None:
        joints = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
    return build_snapshot(
        robot_id=None, observed_at_s=100.0, time_basis="synthetic_seconds",
        status={"timestamp_s": 99.9, "ctrl_mode": 2, "arm_status": 6,
                "teach_status": 2, "motion_status": 0, "err_code": 9},
        joints=None if joints is False else
               {"timestamp_s": joint_stamp, "joint_rad": joints},
        flange=({"timestamp_s": 99.7,
                 "flange_pose_m_rad": [-0.18, 0.01, 0.7, 0, 0, 0]}
                if flange else None),
        drivers=None,
        max_status_age_s=1.0, max_joints_age_s=1.0,
        max_flange_age_s=1.0, max_driver_age_s=1.0,
    )


class WaypointTests(unittest.TestCase):
    def test_roundtrip_preserves_fault_negative_x_time_and_quality(self):
        record = capture_waypoint("READY", snapshot(), source="synthetic feedback")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "one.json"
            self.assertEqual(save_waypoint(record, path), path)
            loaded = load_waypoint(path)
            self.assertEqual(loaded, record)
            self.assertEqual(loaded["name"], "READY")
            self.assertEqual(loaded["kind"], "observed_joint_waypoint")
            self.assertEqual(loaded["snapshot"]["joint_rad"],
                             [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
            self.assertEqual(loaded["snapshot"]["flange_pose_m_rad"][0], -0.18)
            self.assertEqual(loaded["snapshot"]["arm_status"], 6)
            self.assertEqual(loaded["snapshot"]["error_code"], 9)
            self.assertEqual(loaded["snapshot"]["joints_quality"]["timestamp_s"], 99.8)
            self.assertEqual(loaded["captured_at_s"], 100.0)
            self.assertEqual(loaded["time_basis"], "synthetic_seconds")
            self.assertIsNone(loaded["robot_id"])
            self.assertIsNone(loaded["tool_id"])
            self.assertIsNone(loaded["flange_frame"])
            self.assertIn("multiple CAN frame caches", loaded["source_limitations"][0])
            self.assertEqual(list_waypoints(directory)[0]["record"], record)

    def test_missing_flange_and_drivers_do_not_erase_joints(self):
        record = capture_waypoint("edge", snapshot(flange=False), source="synthetic")
        self.assertIsNone(record["snapshot"]["flange_pose_m_rad"])
        self.assertEqual(record["snapshot"]["flange_quality"]["state"], "missing")
        self.assertTrue(all(item["quality"]["state"] == "missing"
                            for item in record["snapshot"]["drivers"]))

    def test_bad_joint_sources_never_capture(self):
        bad = (snapshot(joints=False), snapshot(joints=[0] * 6),
               snapshot(joints=[0] * 6 + [math.nan]),
               snapshot(joints=[0] * 6 + [math.inf]),
               snapshot(joint_stamp=97.0))
        for item in bad:
            with self.subTest(quality=item.joints_quality.state):
                with self.assertRaisesRegex(ValueError, "seven finite joint angles"):
                    capture_waypoint("bad", item, source="synthetic")

    def test_existing_file_unchanged_and_no_partial_file(self):
        record = capture_waypoint("A", snapshot(), source="synthetic")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "point.json"
            path.write_bytes(b"original")
            with self.assertRaises(FileExistsError):
                save_waypoint(record, path)
            self.assertEqual(path.read_bytes(), b"original")
            self.assertEqual(sorted(x.name for x in Path(directory).iterdir()), ["point.json"])

    def test_corrupt_unknown_schema_and_nonstandard_json_are_reported(self):
        record = capture_waypoint("A", snapshot(), source="synthetic")
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            save_waypoint(record, root / "good.json")
            (root / "broken.json").write_text("{", encoding="utf-8")
            old = dict(record, schema_version=99)
            (root / "unknown.json").write_text(json.dumps(old), encoding="utf-8")
            (root / "nan.json").write_text('{"value":NaN}', encoding="utf-8")
            (root / "duplicate.json").write_text('{"x":1,"x":2}', encoding="utf-8")
            for name in ("broken", "unknown", "nan", "duplicate"):
                with self.subTest(name=name):
                    with self.assertRaises(ValueError):
                        load_waypoint(root / f"{name}.json")
            entries = list_waypoints(root)
            self.assertEqual(sum(entry["record"] is not None for entry in entries), 1)
            self.assertEqual(sum(entry["error"] is not None for entry in entries), 4)

    def test_valid_quality_age_must_match_historical_observation(self):
        original = capture_waypoint("history", snapshot(), source="synthetic")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "history.json"
            for age in (-1, None, 2.0, math.inf):
                with self.subTest(age=age):
                    record = json.loads(json.dumps(original))
                    record["snapshot"]["joints_quality"]["age_s"] = age
                    path.write_text(json.dumps(record), encoding="utf-8")
                    with self.assertRaises(ValueError):
                        load_waypoint(path)

            record = json.loads(json.dumps(original))
            record["snapshot"]["status_quality"]["age_s"] = -1
            path.write_text(json.dumps(record), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_waypoint(path)

            record = json.loads(json.dumps(original))
            record["snapshot"]["joints_quality"]["timestamp_s"] = 100.1
            record["snapshot"]["joints_quality"]["age_s"] = 0.0
            path.write_text(json.dumps(record), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_waypoint(path)

            record = json.loads(json.dumps(original))
            record["snapshot"]["joints_quality"]["age_s"] = 0.2000000001
            path.write_text(json.dumps(record), encoding="utf-8")
            self.assertEqual(load_waypoint(path), record)
            self.assertEqual(load_waypoint(path)["captured_at_s"], 100.0)


if __name__ == "__main__":
    unittest.main()
