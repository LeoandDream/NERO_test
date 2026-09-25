"""离线验证去 S1 的路线和规划文件校验；不连接 CAN。"""

import copy
import math
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from experiments.single_arm.start_transfer import go_to_start  # noqa: E402
from experiments.single_arm.teaching.teach_session import load_config  # noqa: E402


class GoToStartTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        cls.current = [
            0.033057, -0.772134, 0.063739, -0.037699,
            0.004887, -0.128194, -0.100409,
        ]

    def make_plan(self):
        route = go_to_start.plan_route(self.current, self.config["safe_start_joint_rad"], self.config)
        return {
            "schema_version": 1,
            "mount": self.config["mount"],
            "start_joint_rad": route[0],
            "target_joint_rad": route[-1],
            "waypoints_joint_rad": route,
            "route_kind": "joint_linear",
            "speed_percent": go_to_start.SPEED_PERCENT,
            "flange_workspace_m": self.config["flange_workspace_m"],
            "executed": False,
        }

    def test_route_has_bounded_steps_and_reaches_measured_s1(self):
        plan = self.make_plan()
        route = go_to_start.check_plan(plan, self.config)
        self.assertEqual(len(route) - 1, 12)
        self.assertEqual(route[-1], self.config["safe_start_joint_rad"])
        self.assertTrue(all(
            math.dist(a, b) <= go_to_start.PLAN_STEP_RAD + 1e-9
            for a, b in zip(route, route[1:])
        ))

    def test_rejects_large_unknown_transfer(self):
        far = list(self.current)
        far[0] += 1.0
        with self.assertRaisesRegex(ValueError, "人工指定中间路径"):
            go_to_start.plan_route(far, self.config["safe_start_joint_rad"], self.config)

    def test_near_zero_start_exits_by_j4_without_approaching_zero(self):
        near_zero = [
            0.024696, 0.034435, 0.013177, -0.078854,
            -0.021921, -0.066043, 0.114249,
        ]
        route = go_to_start.plan_route(near_zero, self.config["safe_start_joint_rad"], self.config)
        self.assertEqual(len(route) - 1, 23)
        self.assertEqual(route[-1], self.config["safe_start_joint_rad"])
        self.assertTrue(all(
            math.dist(a, b) <= go_to_start.PLAN_STEP_RAD + 1e-9
            for a, b in zip(route, route[1:])
        ))
        initial_norm = math.dist(near_zero, [0.0] * 7)
        self.assertGreaterEqual(min(math.dist(point, [0.0] * 7) for point in route), initial_norm - 1e-9)

    def test_rejects_modified_or_reused_plan(self):
        plan = self.make_plan()
        changed = copy.deepcopy(plan)
        changed["waypoints_joint_rad"][1][0] += 0.01
        with self.assertRaisesRegex(ValueError, "路线与当前规划算法不匹配"):
            go_to_start.check_plan(changed, self.config)
        plan["executed"] = True
        with self.assertRaisesRegex(ValueError, "不能重复使用"):
            go_to_start.check_plan(plan, self.config)

    def test_rejects_feedback_outside_small_step_path(self):
        route = self.make_plan()["waypoints_joint_rad"]
        deviated = list(route[0])
        deviated[0] += 0.20
        with patch.object(go_to_start, "healthy_status"), patch.object(
            go_to_start, "check_drivers"
        ), patch.object(go_to_start, "joint_point", return_value=deviated):
            with self.assertRaisesRegex(RuntimeError, "偏离规划小步路线"):
                go_to_start.wait_waypoint(object(), route[0], route[1], self.config, lambda: False)

    def test_motion_failure_sends_emergency_stop(self):
        route = self.make_plan()["waypoints_joint_rad"]

        class FakeRobot:
            def __init__(self):
                self.joints = list(route[0])
                self.moves = 0
                self.emergency_stopped = False

            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time())

            def set_speed_percent(self, _speed):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, _target):
                self.moves += 1

            def electronic_emergency_stop(self):
                self.emergency_stopped = True

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot()
        ready = SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1, arm_status=0,
                                                    mode_feedback=1, err_code=0), timestamp=time.time())
        with patch.object(go_to_start.can, "Bus", return_value=FakeBus()), patch.object(
            go_to_start, "send_frame"
        ), patch.object(go_to_start, "wait_status", return_value=ready), patch.object(
            go_to_start, "healthy_status"
        ), patch.object(go_to_start, "check_drivers"), patch.object(
            go_to_start, "joint_point", side_effect=lambda _robot: list(robot.joints)
        ), patch.object(go_to_start, "wait_waypoint", side_effect=RuntimeError("模拟路径故障")):
            with self.assertRaisesRegex(RuntimeError, "模拟路径故障"):
                go_to_start.execute_route(robot, self.config, route, lambda: False)
        self.assertEqual(robot.moves, 1)
        self.assertTrue(robot.emergency_stopped)

    def test_stationary_unexecuted_waypoint_pauses_without_emergency_stop(self):
        route = self.make_plan()["waypoints_joint_rad"]

        class FakeRobot:
            def __init__(self):
                self.joints = list(route[0])
                self.emergency_stopped = False

            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time())

            def set_speed_percent(self, _speed):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, _target):
                pass

            def electronic_emergency_stop(self):
                self.emergency_stopped = True

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot()
        ready = SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1, arm_status=0,
                                                    mode_feedback=1, err_code=0,
                                                    motion_status=1), timestamp=time.time())
        with patch.object(go_to_start.can, "Bus", return_value=FakeBus()), patch.object(
            go_to_start, "send_frame"
        ), patch.object(go_to_start, "wait_status", return_value=ready), patch.object(
            go_to_start, "healthy_status", return_value=ready
        ), patch.object(go_to_start, "check_drivers"), patch.object(
            go_to_start, "joint_point", side_effect=lambda _robot: list(robot.joints)
        ), patch.object(
            go_to_start, "stable_joint_feedback", return_value=route[0]
        ), patch.object(
            go_to_start, "wait_waypoint",
            side_effect=go_to_start.WaypointTimeout(5.0, 0.02, 0.02)
        ):
            with self.assertRaises(go_to_start.RoutePaused) as caught:
                go_to_start.execute_route(
                    robot, self.config, route, lambda: False,
                    pause_on_stationary_timeout=True,
                )
        self.assertEqual(caught.exception.reason, "stationary_waypoint_timeout")
        self.assertFalse(robot.emergency_stopped)

    def test_stationary_check_rejects_unknown_motion_state(self):
        route = self.make_plan()["waypoints_joint_rad"]
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=255))
        with patch.object(go_to_start, "healthy_status", return_value=status), patch.object(
            go_to_start, "joint_point", return_value=route[0]
        ):
            self.assertFalse(go_to_start.confirm_stationary_at_previous(
                object(), route[0], self.config,
            ))

    def test_stationary_timeout_retries_same_target_once(self):
        route = self.make_plan()["waypoints_joint_rad"][:2]

        class FakeRobot:
            def __init__(self):
                self.joints = list(route[0])
                self.moves = []
                self.emergency_stopped = False

            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time())

            def set_speed_percent(self, _speed):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, target):
                self.moves.append(list(target))
                if len(self.moves) == 2:
                    self.joints = list(target)

            def electronic_emergency_stop(self):
                self.emergency_stopped = True

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot()
        ready = SimpleNamespace(msg=SimpleNamespace(
            ctrl_mode=1, arm_status=0, mode_feedback=1, err_code=0,
            motion_status=0,
        ), timestamp=time.time())
        waits = [go_to_start.WaypointTimeout(5.0, 0.02, 0.02), None]
        retries = []

        def fake_wait(*_args, **_kwargs):
            result = waits.pop(0)
            if result is not None:
                raise result

        with patch.object(go_to_start.can, "Bus", return_value=FakeBus()), patch.object(
            go_to_start, "send_frame"
        ), patch.object(go_to_start, "wait_status", return_value=ready), patch.object(
            go_to_start, "healthy_status", return_value=ready
        ), patch.object(go_to_start, "check_drivers"), patch.object(
            go_to_start, "joint_point", side_effect=lambda _robot: list(robot.joints)
        ), patch.object(
            go_to_start, "confirm_stationary_at_previous", return_value=True
        ), patch.object(go_to_start, "wait_waypoint", side_effect=fake_wait):
            go_to_start.execute_route(
                robot, self.config, route, lambda: False,
                pause_on_stationary_timeout=True,
                stationary_retry_limit=1,
                on_stationary_retry=lambda step, retry: retries.append((step, retry)),
            )
        self.assertEqual(robot.moves, [route[1], route[1]])
        self.assertEqual(retries, [(1, 1)])
        self.assertFalse(robot.emergency_stopped)

    def test_live_feedback_corridor_failure_sends_emergency_stop(self):
        route = self.make_plan()["waypoints_joint_rad"]

        class FakeRobot:
            def __init__(self):
                self.joints = list(route[0])
                self.emergency_stopped = False

            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time())

            def set_speed_percent(self, _speed):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, _target):
                pass

            def electronic_emergency_stop(self):
                self.emergency_stopped = True

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot()
        ready = SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1, arm_status=0,
                                                    mode_feedback=1, err_code=0), timestamp=time.time())

        def simulated_wait(_robot, _previous, _target, _config, _aborted,
                           on_feedback=None, target_tolerance_rad=0.02,
                           timeout_s=None, max_timeout_s=None, on_status=None,
                           path_mode="line", joint_box_margin_rad=0.03):
            self.assertEqual(target_tolerance_rad, 0.005)
            on_feedback(list(route[0]))

        with patch.object(go_to_start.can, "Bus", return_value=FakeBus()), patch.object(
            go_to_start, "send_frame"
        ), patch.object(go_to_start, "wait_status", return_value=ready), patch.object(
            go_to_start, "healthy_status"
        ), patch.object(go_to_start, "check_drivers"), patch.object(
            go_to_start, "joint_point", side_effect=lambda _robot: list(robot.joints)
        ), patch.object(go_to_start, "wait_waypoint", side_effect=simulated_wait):
            with self.assertRaisesRegex(RuntimeError, "模拟法兰越界"):
                go_to_start.execute_route(
                    robot, self.config, route, lambda: False,
                    on_feedback=lambda _q: (_ for _ in ()).throw(RuntimeError("模拟法兰越界")),
                    target_tolerance_rad=0.005,
                )
        self.assertTrue(robot.emergency_stopped)

    def test_joint_box_accepts_actual_asynchronous_controller_feedback(self):
        report_path = ROOT / (
            "experiments/single_arm/start_transfer/data/runs/"
            "nero_controller_joint_return_20260925T070004Z.json"
        )
        import json
        report = json.loads(report_path.read_text(encoding="utf-8"))
        start, target = report["start_joint_rad"], report["target_joint_rad"]
        feedback = report["feedback_samples"][-1]["joint_rad"]
        self.assertGreater(go_to_start.point_segment_distance(feedback, start, target),
                           go_to_start.PATH_TOLERANCE_RAD)
        self.assertLess(go_to_start.joint_box_deviation(feedback, start, target), 0.001)
        outside = list(feedback)
        outside[2] += 0.04  # J3 本段目标不变；异常漂移仍需拒绝。
        self.assertGreater(go_to_start.joint_box_deviation(outside, start, target), 0.03)

        sequence = [list(item["joint_rad"]) for item in report["feedback_samples"]]
        sequence.extend([list(target), list(target)])
        with patch.object(go_to_start, "healthy_status"), patch.object(
            go_to_start, "check_drivers"
        ), patch.object(go_to_start, "joint_point", side_effect=sequence), patch.object(
            go_to_start.time, "sleep"
        ):
            go_to_start.wait_waypoint(
                object(), start, target, self.config, lambda: False,
                target_tolerance_rad=0.005, timeout_s=2, max_timeout_s=2,
                path_mode="joint_box",
            )

    def test_progress_allows_wait_past_base_timeout(self):
        route = self.make_plan()["waypoints_joint_rad"]
        start, target = route[:2]
        clock = SimpleNamespace(now=0.0)
        samples = [start, [(a + b) / 2 for a, b in zip(start, target)], target, target]

        def next_sample(_robot):
            return list(samples.pop(0))

        def advance(seconds):
            clock.now += seconds

        with patch.object(go_to_start.time, "monotonic", side_effect=lambda: clock.now), patch.object(
            go_to_start.time, "sleep", side_effect=advance
        ), patch.object(go_to_start, "healthy_status"), patch.object(
            go_to_start, "check_drivers"
        ), patch.object(go_to_start, "joint_point", side_effect=next_sample), patch.object(
            go_to_start, "validate_flange_workspace"
        ):
            go_to_start.wait_waypoint(object(), start, target, self.config, lambda: False,
                                      target_tolerance_rad=0.005, timeout_s=0.1,
                                      max_timeout_s=0.3)
        self.assertGreater(clock.now, 0.1)

    def test_stalled_waypoint_raises_diagnostic_timeout(self):
        route = self.make_plan()["waypoints_joint_rad"]
        start, target = route[:2]
        clock = SimpleNamespace(now=0.0)

        def advance(seconds):
            clock.now += seconds

        with patch.object(go_to_start.time, "monotonic", side_effect=lambda: clock.now), patch.object(
            go_to_start.time, "sleep", side_effect=advance
        ), patch.object(go_to_start, "healthy_status"), patch.object(
            go_to_start, "check_drivers"
        ), patch.object(go_to_start, "joint_point", return_value=start), patch.object(
            go_to_start, "validate_flange_workspace"
        ):
            with self.assertRaises(go_to_start.WaypointTimeout) as caught:
                go_to_start.wait_waypoint(object(), start, target, self.config, lambda: False,
                                          target_tolerance_rad=0.005, timeout_s=0.1,
                                          max_timeout_s=0.3)
        self.assertGreater(caught.exception.best_error_rad, 0.005)
        self.assertAlmostEqual(caught.exception.elapsed_s, 0.3)

    def test_confirmed_waypoint_pause_does_not_send_emergency_stop(self):
        route = self.make_plan()["waypoints_joint_rad"]

        class FakeRobot:
            def __init__(self):
                self.joints = list(route[0])
                self.moves = 0
                self.emergency_stopped = False

            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time())

            def set_speed_percent(self, _speed):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, target):
                self.moves += 1
                self.joints = list(target)

            def electronic_emergency_stop(self):
                self.emergency_stopped = True

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot()
        ready = SimpleNamespace(msg=SimpleNamespace(ctrl_mode=1, arm_status=0,
                                                    mode_feedback=1, err_code=0), timestamp=time.time())
        with patch.object(go_to_start.can, "Bus", return_value=FakeBus()), patch.object(
            go_to_start, "send_frame"
        ), patch.object(go_to_start, "wait_status", return_value=ready), patch.object(
            go_to_start, "healthy_status"
        ), patch.object(go_to_start, "check_drivers"), patch.object(
            go_to_start, "joint_point", side_effect=lambda _robot: list(robot.joints)
        ), patch.object(go_to_start, "wait_waypoint"):
            with self.assertRaises(go_to_start.RoutePaused):
                go_to_start.execute_route(robot, self.config, route, lambda: False,
                                          stop_after_waypoint=lambda _index, _target: True)
        self.assertEqual(robot.moves, 1)
        self.assertEqual(robot.joints, route[1])
        self.assertFalse(robot.emergency_stopped)


if __name__ == "__main__":
    unittest.main()
