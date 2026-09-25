"""验证小行程路线和可选速度确实进入 CAN/SDK 指令；不连接硬件。"""

import math
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import patch

from experiments.single_arm.local_probe.local_joint_probe import plan_probe
from experiments.single_arm.start_transfer import go_to_start
from experiments.single_arm.teaching.teach_session import load_config


ROOT = Path(__file__).resolve().parents[4]


class LocalProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        cls.current = [0.228743, 0.347582, 0.067335, 0.034767, 0.010943, 0.112644, 0.448916]

    def test_route_changes_only_j4_and_returns_to_actual_start(self):
        route = plan_probe(self.current, self.config)
        self.assertEqual(len(route), 3)
        self.assertEqual(route[0], route[-1])
        self.assertAlmostEqual(route[1][3] - route[0][3], 0.05)
        self.assertTrue(all(math.dist(a, b) <= 0.05 + 1e-9 for a, b in zip(route, route[1:])))

    def test_speed_10_is_sent_to_can_and_sdk(self):
        route = plan_probe(self.current, self.config)

        class FakeRobot:
            def __init__(self):
                self.joints = list(route[0])
                self.speed = None

            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time())

            def set_speed_percent(self, speed):
                self.speed = speed

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, target):
                self.joints = list(target)

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot()
        ready = SimpleNamespace(
            msg=SimpleNamespace(ctrl_mode=1, arm_status=0, mode_feedback=1, err_code=0),
            timestamp=time.time() + 0.01,
        )
        frames = []
        with patch.object(go_to_start.can, "Bus", return_value=FakeBus()), patch.object(
            go_to_start, "send_frame", side_effect=lambda _bus, can_id, payload: frames.append((can_id, payload))
        ), patch.object(go_to_start, "wait_status", return_value=ready), patch.object(
            go_to_start, "fresh", return_value=True
        ), patch.object(go_to_start, "healthy_status"), patch.object(
            go_to_start, "check_drivers"
        ), patch.object(go_to_start, "joint_point", side_effect=lambda _robot: list(robot.joints)), patch.object(
            go_to_start, "wait_waypoint"
        ):
            go_to_start.execute_route(robot, self.config, route, lambda: False, speed_percent=10)
        self.assertEqual(robot.speed, 10)
        self.assertEqual(frames[0][0], 0x151)
        self.assertEqual(frames[0][1][2], 10)


if __name__ == "__main__":
    unittest.main()
