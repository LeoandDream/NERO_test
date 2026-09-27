"""Offline gates for the one-off H0 recovery route; no CAN connection."""

from pathlib import Path
import unittest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.lab.recover_h0_after_drag_jump import (
    START_Q, _observed_precontact, build_route,
)
from experiments.single_arm.teaching.teach_session import load_config


class H0JumpRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
            channel="can0", local_loopback=True))
        cls.config = load_config(Path("experiments/single_arm/config/nero_teach.json"))

    def state(self, q):
        return {"control_mode": 2, "arm_status": 0, "teach_status": 0,
                "error_code": 0, "joint_variation_0_25s_rad": 0,
                "drivers": [{"enabled": True, "undervoltage": False,
                             "driver_error": False} for _ in range(7)],
                "joint_rad": q, "flange_pose_m_rad": self.robot.fk(q)}

    def test_clearance_and_contact_use_observed_precontact(self):
        route, boxes = build_route(
            self.robot, self.state(START_Q), self.config, "clearance")
        self.assertEqual(len(route), 5)
        self.assertGreater(boxes[1]["x"][0], 0.035)
        self.assertEqual(route[-1], _observed_precontact())
        contact, _ = build_route(
            self.robot, self.state(route[-1]), self.config, "contact")
        self.assertLess(abs(contact[-1][3] - contact[0][3]), 0.08)

    def test_changed_start_or_premature_contact_is_rejected(self):
        moved = list(START_Q)
        moved[0] += 0.05
        with self.assertRaisesRegex(ValueError, "偏离本次异动"):
            build_route(self.robot, self.state(moved), self.config, "clearance")
        with self.assertRaisesRegex(ValueError, "未停在手拖实测"):
            build_route(self.robot, self.state(START_Q), self.config, "contact")


if __name__ == "__main__":
    unittest.main()
