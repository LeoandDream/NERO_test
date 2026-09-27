"""Offline checks for the one-time table-edge release target."""

import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.edge_y_release_current import SOURCE_Q, plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class EdgeYReleaseCurrentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")  # FK only; no connect or CAN frames.
        cls.config = load_config(DEFAULT_CONFIG)

    def snapshot(self, joints):
        return {"joint_rad": joints,
                "flange_pose_m_rad": self.robot.fk(joints),
                "ready_for_motion": True}

    def test_route_moves_only_j1_outward_with_expected_envelope(self):
        _, target, box = plan(self.robot, self.snapshot(SOURCE_Q), self.config)
        self.assertEqual(target[1:], SOURCE_Q[1:])
        self.assertLess(box["y"][0], -0.075)
        self.assertGreater(box["x"][0], -0.18)

    def test_changed_start_is_rejected(self):
        changed = list(SOURCE_Q)
        changed[1] -= 0.01
        with self.assertRaisesRegex(ValueError, "实时姿态已偏离"):
            plan(self.robot, self.snapshot(changed), self.config)


if __name__ == "__main__":
    unittest.main()
