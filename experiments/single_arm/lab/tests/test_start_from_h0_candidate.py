"""Offline geometry and state gates for the supported-home startup trial."""

import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.start_from_h0_candidate import (
    SUPPORTED_Q, _check_supported_state, plan,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class CandidateStartupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")
        cls.config = load_config(DEFAULT_CONFIG)

    def test_first_target_only_moves_j1_away_from_edge(self):
        route, boxes = plan(self.robot, {"joint_rad": SUPPORTED_Q}, self.config)
        self.assertEqual(route[0][1:], route[1][1:])
        self.assertLess(self.robot.fk(route[1])[1],
                        self.robot.fk(route[0])[1] - 0.03)
        self.assertEqual(route[-1], self.config["safe_start_joint_rad"])
        self.assertEqual(len(boxes), len(route)-1)

    def test_supported_state_rejects_enabled_drive(self):
        state = {"control_mode": 1, "arm_status": 6, "error_code": 0,
                 "joint_rad": SUPPORTED_Q,
                 "flange_pose_m_rad": self.robot.fk(SUPPORTED_Q),
                 "joint_variation_0_25s_rad": 0,
                 "drivers": [{"enabled": False, "undervoltage": False,
                              "driver_error": False} for _ in range(7)]}
        self.assertIs(_check_supported_state(self.robot, state, self.config), state)
        state["drivers"][0]["enabled"] = True
        with self.assertRaisesRegex(ValueError, "驱动状态"):
            _check_supported_state(self.robot, state, self.config)


if __name__ == "__main__":
    unittest.main()
