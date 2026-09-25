"""连续轨迹的离线完整性与门槛测试；不连接机械臂。"""

import json
from pathlib import Path
import tempfile
import unittest

from experiments.single_arm.continuous_replay.continuous_session import check_plan, create_plan
from experiments.single_arm.continuous_replay.recover_prefix_return import derive_return
from experiments.single_arm.continuous_replay.trajectory import (
    DEFAULT_SOURCE, EXPECTED_SOURCE_SHA256, make_plan, read_source,
)
from experiments.single_arm.teaching.teach_session import load_config


ROOT = Path(__file__).resolve().parents[4]


class ContinuousTrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        cls.source = read_source(ROOT / DEFAULT_SOURCE, cls.config)

    def test_complete_source_is_preserved_with_timed_round_trip(self):
        plan = make_plan(self.source, self.config)
        self.assertEqual(plan["source_sha256"], EXPECTED_SOURCE_SHA256)
        self.assertEqual(plan["source_used_rows"], 1022)
        self.assertEqual(plan["source_teaching_rows"], 998)
        self.assertGreater(plan["source_used_duration_s"], 20)
        self.assertEqual(plan["waypoints"][0]["joint_rad"], self.source["joints"][0])
        self.assertEqual(plan["waypoints"][-1]["joint_rad"], self.source["joints"][0])
        self.assertEqual(plan["waypoints"][-1]["t_s"], plan["total_duration_s"])
        self.assertLessEqual(plan["max_path_deviation_rad"], 0.01)
        self.assertLessEqual(plan["max_source_velocity_rad_s"] / plan["time_scale"], 0.20)
        self.assertLessEqual(plan["max_source_acceleration_rad_s2"] / plan["time_scale"]**2, 0.40)
        phases = [point["phase"] for point in plan["waypoints"]]
        self.assertEqual(phases.count("forward") + phases.count("return"), len(phases))
        self.assertTrue(all(a["t_s"] < b["t_s"] for a,b in zip(
            plan["waypoints"], plan["waypoints"][1:]
        )))

    def test_prefix_is_explicit_and_cannot_masquerade_as_complete(self):
        plan = make_plan(self.source, self.config, until_source_s=1.8)
        self.assertEqual(plan["source_used_rows"], 91)
        self.assertEqual(plan["source_total_rows"], 1022)
        self.assertAlmostEqual(plan["source_used_duration_s"], 1.8)
        self.assertEqual(plan["waypoints"][0]["joint_rad"], plan["waypoints"][-1]["joint_rad"])

    def test_candidate_prefix_holds_exact_recorded_joint_pose_then_returns(self):
        plan = create_plan(ROOT / DEFAULT_SOURCE, self.config,
                           until_source_s=3.2, turnaround_hold_s=3.0)
        self.assertEqual(plan["source_used_rows"], 161)
        forward = [item for item in plan["waypoints"] if item["phase"] == "forward"]
        hold = [item for item in plan["waypoints"] if item["phase"] == "hold"]
        self.assertEqual(forward[-1]["joint_rad"], self.source["joints"][160])
        self.assertEqual(len(hold), 60)
        self.assertTrue(all(item["joint_rad"] == forward[-1]["joint_rad"]
                            for item in hold))
        self.assertAlmostEqual(hold[-1]["t_s"] - forward[-1]["t_s"], 3.0)
        self.assertEqual(plan["waypoints"][-1]["joint_rad"], plan["waypoints"][0]["joint_rad"])
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "candidate_plan.json"
            path.write_text(json.dumps(plan), encoding="utf-8")
            self.assertEqual(check_plan(path, self.config)["waypoints"], plan["waypoints"])

    def test_saved_plan_rebuilds_and_rejects_modified_target(self):
        plan = create_plan(ROOT / DEFAULT_SOURCE, self.config, until_source_s=1.8)
        with tempfile.TemporaryDirectory() as name:
            path = Path(name) / "plan.json"
            path.write_text(json.dumps(plan), encoding="utf-8")
            self.assertEqual(len(check_plan(path, self.config)["waypoints"]), len(plan["waypoints"]))
            plan["waypoints"][5]["joint_rad"][0] += 0.01
            path.write_text(json.dumps(plan), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "重新计算结果不符"):
                check_plan(path, self.config)

    def test_failed_candidate_hold_can_only_resume_along_saved_reverse_path(self):
        plan = create_plan(ROOT / DEFAULT_SOURCE, self.config,
                           until_source_s=3.2, turnaround_hold_s=3.0)
        last = max(index for index, item in enumerate(plan["waypoints"])
                   if item["phase"] == "hold") - 1
        with tempfile.TemporaryDirectory() as name:
            root = Path(name)
            plan_path = root / "plan.json"
            report_path = root / "report.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            report = {
                "plan": str(plan_path), "completed": False,
                "stop_action": "move_j_hold_current_confirmed",
                "source_sha256": plan["source_sha256"],
                "error": "源前缀终点未连续停稳至少 0.5 秒，不反向",
                "last_published_index": last,
                "hold_joint_rad": plan["waypoints"][last]["joint_rad"],
            }
            report_path.write_text(json.dumps(report), encoding="utf-8")
            recovered = derive_return(report_path, self.config)
            self.assertEqual(recovered["waypoints"][0]["joint_rad"],
                             self.source["joints"][160])
            self.assertEqual(recovered["waypoints"][-1]["joint_rad"],
                             self.source["joints"][0])
            self.assertTrue(all(item["phase"] == "return"
                                for item in recovered["waypoints"]))
            report["stop_action"] = "electronic_emergency_stop"
            report_path.write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaises(ValueError):
                derive_return(report_path, self.config)


if __name__ == "__main__":
    unittest.main()
