"""Offline guard for the one-time H0 pressure relief command."""

import json
from pathlib import Path
import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.relieve_h0_contact import plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class ReliefTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")
        cls.config = load_config(DEFAULT_CONFIG)
        cls.h0 = json.loads(Path(
            "experiments/single_arm/lab/config/supported_home.json"
        ).read_text(encoding="utf-8"))["joint_rad"]

    def test_relief_is_outward_and_does_not_target_s1(self):
        state = {"ready_for_motion": True, "reasons": [], "joint_rad": self.h0,
                 "flange_pose_m_rad": self.robot.fk(self.h0)}
        start, target, _ = plan(self.robot, state, self.config)
        self.assertLess(self.robot.fk(target)[1], self.robot.fk(start)[1] - 0.004)
        self.assertNotEqual(target, self.config["safe_start_joint_rad"])

    def test_other_pose_rejected(self):
        s1 = self.config["safe_start_joint_rad"]
        state = {"ready_for_motion": True, "reasons": [], "joint_rad": s1,
                 "flange_pose_m_rad": self.robot.fk(s1)}
        with self.assertRaisesRegex(ValueError, "不是刚才"):
            plan(self.robot, state, self.config)


if __name__ == "__main__":
    unittest.main()
