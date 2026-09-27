"""Offline path guards for the light-contact home candidate."""

import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.disable_at_h0_candidate import check_pre_disable
from experiments.single_arm.lab.go_home_candidate import CONTACT_Q, plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class CandidateParkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")
        cls.config = load_config(DEFAULT_CONFIG)

    def test_high_detour_and_single_axis_final_approach(self):
        s1 = self.config["safe_start_joint_rad"]
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": s1, "flange_pose_m_rad": self.robot.fk(s1)}
        route, boxes = plan(self.robot, state, self.config)
        self.assertLess(self.robot.fk(route[1])[1],
                        self.robot.fk(route[0])[1] - 0.07)
        self.assertEqual(route[-1], CONTACT_Q)
        self.assertEqual(route[-2][1:], route[-1][1:])
        self.assertEqual(len(boxes), len(route)-1)

    def test_non_s1_start_rejected(self):
        q = list(self.config["safe_start_joint_rad"])
        q[1] += 0.03
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": q, "flange_pose_m_rad": self.robot.fk(q)}
        with self.assertRaisesRegex(ValueError, "不是 S1"):
            plan(self.robot, state, self.config)

    def test_home_target_matches_separate_disable_guard(self):
        # README 的回家→失能两条命令必须使用同一个轻触承托目标。
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": CONTACT_Q,
                 "flange_pose_m_rad": self.robot.fk(CONTACT_Q)}
        self.assertIs(check_pre_disable(self.robot, state, self.config), state)


if __name__ == "__main__":
    unittest.main()
