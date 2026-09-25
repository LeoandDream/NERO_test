"""只用假机器人验证命令类型和参数；不会打开 CAN。"""

import math
import unittest

from experiments.single_arm.control_interfaces.publishers import (
    CartesianLinearPublisher, CartesianPointPublisher, JointTargetPublisher,
)


class FakeRobot:
    def __init__(self):
        self.calls = []

    def move_j(self, target):
        self.calls.append(("move_j", target))

    def move_l(self, target):
        self.calls.append(("move_l", target))

    def move_p(self, target):
        self.calls.append(("move_p", target))


class PublisherTests(unittest.TestCase):
    def test_joint_and_cartesian_use_different_sdk_commands(self):
        robot = FakeRobot()
        JointTargetPublisher().publish(robot, [0.0] * 7)
        CartesianLinearPublisher().publish(robot, [0.3, 0.0, 0.5, 0.0, 0.0, 0.0])
        CartesianPointPublisher().publish(robot, [0.3, 0.0, 0.5, 0.0, 0.0, 0.0])
        self.assertEqual([call[0] for call in robot.calls], ["move_j", "move_l", "move_p"])

    def test_invalid_target_is_rejected_before_publish(self):
        robot = FakeRobot()
        with self.assertRaises(ValueError):
            JointTargetPublisher().publish(robot, [0.0] * 6)
        with self.assertRaises(ValueError):
            CartesianLinearPublisher().publish(robot, [0.0, 0.0, 0.0, 0.0, math.pi, 0.0])
        with self.assertRaises(ValueError):
            CartesianLinearPublisher().publish(robot, [0.0, 0.0, float("nan"), 0.0, 0.0, 0.0])
        with self.assertRaises(ValueError):
            CartesianPointPublisher().publish(robot, [0.0, 0.0, 0.0, 0.0, math.pi, 0.0])
        self.assertEqual(robot.calls, [])


if __name__ == "__main__":
    unittest.main()
