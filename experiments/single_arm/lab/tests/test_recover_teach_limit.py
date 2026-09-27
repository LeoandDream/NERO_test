"""Offline guards for the single stopped J4 teaching-limit recovery."""

import unittest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.lab.recover_teach_limit import _source_joint, plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


CURRENT = [0.4319864431611165, -0.3677408733952052,
           -1.159526941854953, -1.0152580258851016,
           0.15877260205392416, 0.20552997271485224,
           -0.4694237556163949]


class RecoverTeachLimitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(DEFAULT_CONFIG)
        cls.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
            channel="can0"))
        cls.source = _source_joint()

    def state(self, q=CURRENT):
        return {"control_mode": 2, "arm_status": 0, "teach_status": 0,
                "error_code": 0, "joint_variation_0_25s_rad": 0,
                "joint_rad": list(q),
                "flange_pose_m_rad": list(self.robot.fk(q)),
                "drivers": [{"enabled": True, "undervoltage": False,
                             "driver_error": False} for _ in range(7)]}

    def test_only_j4_moves_inward_into_valid_range(self):
        result = plan(self.robot, self.state(), self.config, self.source)
        self.assertEqual(result["target_joint_rad"][3], -0.98)
        self.assertEqual(result["target_joint_rad"][:3], CURRENT[:3])
        self.assertEqual(result["target_joint_rad"][4:], CURRENT[4:])
        self.assertGreater(result["flange_xyz_range_m"]["x"][0], 0.25)

    def test_wrong_mode_and_nonmatching_pose_are_rejected(self):
        state = self.state()
        state["control_mode"] = 1
        with self.assertRaisesRegex(RuntimeError, "空闲示教模式"):
            plan(self.robot, state, self.config, self.source)
        state = self.state()
        state["joint_rad"][0] += 0.1
        with self.assertRaisesRegex(ValueError, "偏离本次中止示教终点"):
            plan(self.robot, state, self.config, self.source)


if __name__ == "__main__":
    unittest.main()
