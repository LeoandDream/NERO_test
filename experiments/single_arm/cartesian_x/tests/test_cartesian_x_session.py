"""离线验证实际 Nero 正运动学的直线规划与规划文件保护。"""

import copy
import math
from pathlib import Path
import sys
import unittest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from experiments.single_arm.cartesian_x import cartesian_x_session as cartesian  # noqa: E402
from experiments.single_arm.teaching.teach_session import load_config  # noqa: E402


class CartesianXSessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        cls.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel="can0"
        ))
        cls.plan = cartesian.make_plan(cls.robot, cls.config["safe_start_joint_rad"], cls.config)

    def test_x_decreases_ten_cm_and_reverse_reaches_same_joints(self):
        route, summary = cartesian.check_plan(self.robot, self.plan, self.config)
        self.assertAlmostEqual(summary["target_xyz_m"][0] - summary["start_xyz_m"][0], -0.10)
        self.assertLess(math.dist(summary["predicted_endpoint_xyz_m"],
                                  summary["target_xyz_m"]), 0.001)
        self.assertLess(summary["max_yz_error_m"], 0.003)
        self.assertGreater(summary["joint_limit_clearance_rad"][3], 0.10)
        self.assertTrue(all(math.dist(a, b) <= cartesian.JOINT_STEP_RAD + 1e-9
                            for a, b in zip(route, route[1:])))
        self.assertEqual(list(reversed(route))[-1], self.config["safe_start_joint_rad"])

    def test_short_pilot_preserves_corridor_with_fewer_stops(self):
        pilot = cartesian.make_plan(
            self.robot, self.config["safe_start_joint_rad"], self.config, dx_m=-0.02
        )
        route, summary = cartesian.check_plan(self.robot, pilot, self.config)
        self.assertLessEqual(len(route) - 1, 4)
        self.assertAlmostEqual(summary["target_xyz_m"][0] - summary["start_xyz_m"][0], -0.02)
        self.assertLess(summary["max_yz_error_m"], cartesian.XYZ_TOLERANCE_M)
        self.assertGreater(summary["joint_limit_clearance_rad"][3], 0.10)

    def test_changed_or_attempted_plan_is_rejected(self):
        changed = copy.deepcopy(self.plan)
        changed["outbound_joint_rad"][1][0] += 0.01
        with self.assertRaisesRegex(ValueError, "关节路线被修改"):
            cartesian.check_plan(self.robot, changed, self.config)
        attempted = copy.deepcopy(self.plan)
        attempted["execution_attempts"] = [{"completed": False}]
        with self.assertRaisesRegex(ValueError, "已尝试执行"):
            cartesian.check_plan(self.robot, attempted, self.config)


if __name__ == "__main__":
    unittest.main()
