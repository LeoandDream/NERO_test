"""Offline guard for the short, non-disabling table-edge approach."""

import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.approach_h0_j1 import (
    SOURCE_Q, TARGET_J1, STEP3_SOURCE_Q, STEP3_TARGET_J1, plan,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class ApproachH0J1Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")
        cls.config = load_config(DEFAULT_CONFIG)

    def test_short_approach_only_changes_j1(self):
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": SOURCE_Q,
                 "flange_pose_m_rad": self.robot.fk(SOURCE_Q)}
        start, target, _ = plan(self.robot, state, self.config)
        self.assertEqual(target[0], TARGET_J1)
        self.assertEqual(target[1:], start[1:])
        y_change = self.robot.fk(target)[1] - self.robot.fk(start)[1]
        self.assertGreaterEqual(y_change, 0.006)
        self.assertLessEqual(y_change, 0.011)

    def test_changed_pose_rejected(self):
        changed = list(SOURCE_Q)
        changed[1] += 0.01
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": changed,
                 "flange_pose_m_rad": self.robot.fk(changed)}
        with self.assertRaisesRegex(ValueError, "偏离"):
            plan(self.robot, state, self.config)

    def test_fine_approach_stays_single_axis(self):
        state = {"ready_for_motion": True, "reasons": [],
                 "joint_rad": STEP3_SOURCE_Q,
                 "flange_pose_m_rad": self.robot.fk(STEP3_SOURCE_Q)}
        start, target, _ = plan(self.robot, state, self.config,
                                source_q=STEP3_SOURCE_Q,
                                target_j1=STEP3_TARGET_J1)
        self.assertEqual(target[1:], start[1:])
        self.assertGreater(self.robot.fk(target)[1] - self.robot.fk(start)[1],
                           0.003)


if __name__ == "__main__":
    unittest.main()
