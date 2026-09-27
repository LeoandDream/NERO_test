"""Offline checks for the new site-specific high-Y-first H0 trial."""

import json
from pathlib import Path
import unittest

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.high_y_home import plan, run
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class HighYHomeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")  # FK only; never connects to CAN
        cls.config = load_config(DEFAULT_CONFIG)
        cls.home = json.loads(Path(
            "experiments/single_arm/lab/config/supported_home.json"
        ).read_text(encoding="utf-8"))["joint_rad"]

    def snapshot(self, joints):
        return {
            "ready_for_motion": True, "reasons": [], "joint_rad": joints,
            "flange_pose_m_rad": self.robot.fk(joints),
        }

    def test_first_move_is_outward_while_high_and_contact_is_separate(self):
        s1 = self.config["safe_start_joint_rad"]
        outward = plan(self.robot, self.snapshot(s1), self.config, "s1-to-outer")
        route = outward["route_joint_rad"]
        start_xyz = self.robot.fk(route[0])
        first_xyz = self.robot.fk(route[1])
        final_xyz = self.robot.fk(route[-1])
        self.assertLess(first_xyz[1], start_xyz[1] - 0.07)
        self.assertGreater(first_xyz[0], 0.3)
        self.assertLess(final_xyz[1], -0.07)
        self.assertEqual(len(route)-1, 7)
        self.assertFalse(outward["automatic_disable"])

        contact = plan(self.robot, self.snapshot(route[-1]), self.config,
                       "outer-to-h0")
        self.assertEqual(contact["route_joint_rad"][-1], self.home)
        self.assertEqual(contact["speed_percent_by_target"], [2, 2])
        self.assertLess(self.robot.fk(contact["route_joint_rad"][-2])[1],
                        self.robot.fk(self.home)[1])

    def test_startup_departs_h0_in_negative_y_before_body_motion(self):
        result = plan(self.robot, self.snapshot(self.home), self.config,
                      "h0-to-s1")
        route = result["route_joint_rad"]
        self.assertLess(self.robot.fk(route[1])[1],
                        self.robot.fk(route[0])[1] - 0.025)
        self.assertEqual(route[-1], self.config["safe_start_joint_rad"])
        self.assertEqual(result["speed_percent_by_target"][0], 2)

    def test_wrong_start_rejects_motion_plan(self):
        with self.assertRaisesRegex(ValueError, "不在该阶段起点"):
            plan(self.robot, self.snapshot(self.config["safe_start_joint_rad"]),
                 self.config, "outer-to-h0")

    def test_site_rejection_blocks_execution_before_device_connection(self):
        with self.assertRaisesRegex(RuntimeError, "全部锁定"):
            run("h0-to-s1", execute=True)


if __name__ == "__main__":
    unittest.main()
