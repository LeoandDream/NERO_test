"""Offline guard for the single-axis table-edge release."""

import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.release_h0_j1 import SOURCE_Q, TARGET_J1, plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class ReleaseH0J1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")
        cls.config = load_config(DEFAULT_CONFIG)

    def test_only_j1_moves_away_from_table(self):
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": SOURCE_Q,
                 "flange_pose_m_rad": self.robot.fk(SOURCE_Q)}
        start, target, box = plan(self.robot, state, self.config)
        self.assertEqual(target[0], TARGET_J1)
        self.assertEqual(target[1:], start[1:])
        self.assertLess(box["y"][1], state["flange_pose_m_rad"][1] + 0.001)
        self.assertGreater(self.robot.fk(target)[0], state["flange_pose_m_rad"][0])

    def test_changed_pose_is_rejected(self):
        changed = list(SOURCE_Q)
        changed[1] += 0.01
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": changed,
                 "flange_pose_m_rad": self.robot.fk(changed)}
        with self.assertRaisesRegex(ValueError, "偏离"):
            plan(self.robot, state, self.config)


if __name__ == "__main__":
    unittest.main()
