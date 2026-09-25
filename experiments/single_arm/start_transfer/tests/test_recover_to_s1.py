"""急停恢复路线的离线验证；只使用已记录的关节角，不连接 CAN。"""

import copy
import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from experiments.single_arm.start_transfer import recover_to_s1  # noqa: E402
from experiments.single_arm.teaching.teach_session import load_config  # noqa: E402


class RecoverToS1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        # 10% 局部试验后的实测反馈；测试不得当作新的实时关节反馈。
        cls.recorded_start = [
            0.22877775835141673, 0.3475299606571109, 0.06733480254194124,
            0.03467969223712733, 0.010943214410004447, 0.11264354992371402,
            0.4489161369054615,
        ]

    def make_plan(self):
        stages = recover_to_s1.build_stages(
            self.recorded_start, self.config["safe_start_joint_rad"], self.config
        )
        return {
            "schema_version": 1,
            "mount": self.config["mount"],
            "flange_workspace_m": self.config["flange_workspace_m"],
            "start_joint_rad": self.recorded_start,
            "target_joint_rad": self.config["safe_start_joint_rad"],
            "stages": stages,
            "completed_stages": 0,
            "stage_results": [],
            "failed": False,
        }

    def test_route_stages_are_bounded_and_keep_j4_bent_when_j2_crosses_zero(self):
        stages = recover_to_s1.build_stages(
            self.recorded_start, self.config["safe_start_joint_rad"], self.config
        )
        self.assertEqual(len(stages), 6)
        self.assertEqual(stages[0]["waypoints_joint_rad"][0], self.recorded_start)
        self.assertEqual(stages[-1]["waypoints_joint_rad"][-1],
                         self.config["safe_start_joint_rad"])
        for stage in stages:
            route = stage["waypoints_joint_rad"]
            self.assertLessEqual(stage["joint_distance_rad"], recover_to_s1.MAX_TRAVEL_RAD)
            self.assertTrue(all(
                math.dist(a, b) <= 0.05 + 1e-9 for a, b in zip(route, route[1:])
            ))
        for stage in stages[1:3]:
            self.assertTrue(all(
                point[3] <= -0.2 for point in stage["waypoints_joint_rad"]
            ))

    def test_changed_waypoint_or_skipped_stage_is_rejected(self):
        altered = self.make_plan()
        altered["stages"][1]["waypoints_joint_rad"][1][1] += 0.01
        with self.assertRaisesRegex(ValueError, "关节路线"):
            recover_to_s1.validate_plan(altered, self.config)

        skipped = self.make_plan()
        skipped["completed_stages"] = 1
        with self.assertRaisesRegex(ValueError, "阶段执行记录"):
            recover_to_s1.validate_plan(skipped, self.config)

        completed = self.make_plan()
        completed["completed_stages"] = 1
        completed["stage_results"].append({"number": 1, "completed": True})
        self.assertEqual(len(recover_to_s1.validate_plan(completed, self.config)), 6)

    def test_unexpected_start_shape_is_rejected(self):
        changed = copy.deepcopy(self.recorded_start)
        changed[1] = -0.1
        with self.assertRaisesRegex(ValueError, "起点形态"):
            recover_to_s1.build_stages(changed, self.config["safe_start_joint_rad"],
                                       self.config)

    def test_coupled_recovery_has_no_j7_only_step(self):
        after_probe = [
            0.22383847656827277, 0.07852236304722487, 0.06729989595690135,
            -0.09161233243718235, 0.010838494654884786, 0.11309733552923257,
            0.43917719967933316,
        ]
        stages = recover_to_s1.build_coupled_stages(
            after_probe, self.config["safe_start_joint_rad"], self.config
        )
        self.assertEqual(len(stages), 4)
        self.assertEqual(stages[-1]["waypoints_joint_rad"][-1],
                         self.config["safe_start_joint_rad"])
        self.assertAlmostEqual(stages[0]["waypoints_joint_rad"][-1][3],
                               self.config["safe_start_joint_rad"][3])
        for stage in stages:
            self.assertLessEqual(stage["joint_distance_rad"], recover_to_s1.MAX_TRAVEL_RAD)
            for a, b in zip(stage["waypoints_joint_rad"],
                            stage["waypoints_joint_rad"][1:]):
                self.assertLessEqual(math.dist(a, b), recover_to_s1.COUPLED_STEP_RAD + 1e-9)
                self.assertGreaterEqual(abs(a[6] - b[6]), 0.008)
                self.assertGreaterEqual(max(abs(a[i] - b[i]) for i in range(6)), 0.005)
        for stage in stages[1:3]:
            self.assertTrue(all(q[3] <= -0.2 for q in stage["waypoints_joint_rad"]))

        plan = {
            "schema_version": 2,
            "strategy": "coupled-j7",
            "mount": self.config["mount"],
            "flange_workspace_m": self.config["flange_workspace_m"],
            "start_joint_rad": after_probe,
            "target_joint_rad": self.config["safe_start_joint_rad"],
            "stages": stages,
            "completed_stages": 0,
            "stage_results": [],
            "failed": False,
        }
        self.assertEqual(len(recover_to_s1.validate_plan(plan, self.config)), 4)

    def test_after_replay_sag_route_and_schema(self):
        start = [
            0.12997466939601773, 0.2565983066282063, -1.2248371624645806,
            -0.317649923862968, -0.05420992656694387,
            0.205599785884932, -0.46884779696323675,
        ]
        stages = recover_to_s1.build_after_replay_stages(
            start, self.config["safe_start_joint_rad"], self.config
        )
        self.assertEqual([len(s["waypoints_joint_rad"]) - 1 for s in stages],
                         [13, 13, 9, 9, 3])
        self.assertEqual(stages[-1]["waypoints_joint_rad"][-1],
                         self.config["safe_start_joint_rad"])
        for stage in stages:
            self.assertLess(stage["joint_distance_rad"], recover_to_s1.MAX_TRAVEL_RAD)
            self.assertTrue(all(
                math.dist(a, b) <= 0.05 + 1e-9
                for a, b in zip(stage["waypoints_joint_rad"],
                                stage["waypoints_joint_rad"][1:])
            ))
        plan = {
            "schema_version": 3, "strategy": "after-replay-sag",
            "mount": self.config["mount"],
            "flange_workspace_m": self.config["flange_workspace_m"],
            "start_joint_rad": start,
            "target_joint_rad": self.config["safe_start_joint_rad"],
            "stages": stages, "completed_stages": 0,
            "stage_results": [], "failed": False,
        }
        self.assertEqual(len(recover_to_s1.validate_plan(plan, self.config)), 5)
        invalid = copy.deepcopy(start)
        invalid[2] = -0.5
        with self.assertRaisesRegex(ValueError, "恢复姿态"):
            recover_to_s1.build_after_replay_stages(
                invalid, self.config["safe_start_joint_rad"], self.config
            )


if __name__ == "__main__":
    unittest.main()
