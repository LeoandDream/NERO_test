"""Offline checks for the one step release from the contacted desk edge."""

import unittest
from unittest.mock import patch

from experiments.single_arm.lab.edge_release import (
    demonstrated_target, plan_from_snapshot,
)
from experiments.single_arm.lab.tests.test_supported_home import H0, state


class EdgeReleaseTests(unittest.TestCase):
    def test_target_is_recorded_outboard_pose(self):
        config = {"zero_exclusion_radius_rad": 0.25}
        target, xyz = demonstrated_target(config)
        self.assertEqual(len(target), 7)
        self.assertAlmostEqual(xyz[0], -0.164848)
        self.assertAlmostEqual(xyz[2], 0.696603)

    def test_wrong_start_rejected_before_assessment(self):
        config = {"zero_exclusion_radius_rad": 0.25}
        snapshot = {**state(H0), "flange_pose_m_rad":
                    [-0.1227, -0.0241, 0.7056, 0, 0, 0]}
        snapshot["joint_rad"][1] += 0.03
        with patch("experiments.single_arm.lab.edge_release._assess") as assess:
            with self.assertRaisesRegex(RuntimeError, "不在原 H0"):
                plan_from_snapshot(None, snapshot, config,
                                   {"name": "H0", "route_blocked": True,
                                    "joint_rad": H0})
        assess.assert_not_called()


if __name__ == "__main__":
    unittest.main()
