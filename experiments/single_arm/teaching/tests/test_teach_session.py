"""Safety properties of planned Nero return paths; no CAN commands are sent."""

import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from experiments.single_arm.teaching import teach_session  # noqa: E402
from experiments.single_arm.teaching import return_session  # noqa: E402
from experiments.single_arm.teaching.return_session import load_recording  # noqa: E402
from experiments.single_arm.teaching.teach_session import (  # noqa: E402
    JOINT_LIMITS, load_config, plan_reverse_return, validate_flange_workspace,
    validate_joints, validate_route_workspace,
)


class ReturnPlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        cls.home = cls.config["safe_start_joint_rad"]

    def test_return_preflight_retries_only_transient_settling_and_records_states(self):
        base = {"control_mode": 2, "arm_status": 0, "teach_status": 0,
                "error_code": 0, "joint_rad": list(self.home),
                "joint_variation_0_25s_rad": 0.0,
                "drivers": [{"enabled": True, "undervoltage": False,
                             "driver_error": False} for _ in range(7)]}
        moving = dict(base, joint_variation_0_25s_rad=0.003)
        metadata = {}
        with patch("experiments.single_arm.lab.device.read_state",
                   side_effect=[moving, base]) as observed:
            self.assertIs(teach_session.return_preflight_snapshot(object(), metadata), base)
        self.assertEqual(observed.call_count, 2)
        self.assertEqual(metadata["return_preflight_states"], [moving, base])

        bad_driver = dict(base, drivers=[dict(driver) for driver in base["drivers"]])
        bad_driver["drivers"][3]["enabled"] = False
        with patch("experiments.single_arm.lab.device.read_state",
                   return_value=bad_driver) as observed:
            with self.assertRaisesRegex(RuntimeError, "关节 4 失能"):
                teach_session.return_preflight_snapshot(object(), {})
        observed.assert_called_once()

    def test_default_start_is_measured_nonzero_pose_and_p4_is_archived(self):
        self.assertEqual(len(self.home), 7)
        self.assertGreater(math.dist(self.home, [0.0] * 7), 0.25)
        self.assertEqual(self.config["safe_start_name"], "现场确认的起点 S1")
        p4_config = load_config(ROOT / "experiments/single_arm/config/nero_teach_p4.json")
        with (ROOT / p4_config["safe_start_source"]).open(newline="") as stream:
            source = next(row for row in csv.DictReader(stream) if row["candidate_id"] == "P4")
        measured = [float(source[f"joint_{index}_rad"]) for index in range(1, 8)]
        self.assertEqual(p4_config["safe_start_joint_rad"], measured)
        with self.assertRaises(ValueError):
            validate_joints([0.0] * 7, self.config["zero_exclusion_radius_rad"])

    def test_drag_entry_rejects_first_feedback_jump(self):
        class JumpedRobot:
            def get_arm_status(self):
                return SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(
                    teach_status=1, arm_status=0, err_code=0))

            def get_joint_angles(self):
                jumped = list(self_home)
                jumped[1] += 0.2
                return SimpleNamespace(timestamp=time.time(), msg=jumped)

        self_home = self.home
        with self.assertRaisesRegex(RuntimeError, "首 0.5 秒关节位移异常"):
            teach_session.wait_drag_entry_guarded(JumpedRobot(), self.home)

    def test_zero_target_return_still_switches_controller_mode(self):
        from experiments.single_arm.start_transfer import go_to_start

        planned = {"controller_targets": 0, "route_joint_rad": [self.home]}
        metadata = {}
        robot = object()
        with patch.object(go_to_start, "execute_route") as publish:
            teach_session.execute_planned_return(
                robot, self.config, planned, lambda: False, metadata)
        publish.assert_called_once()
        self.assertIs(publish.call_args.args[0], robot)
        self.assertEqual(publish.call_args.args[2], [self.home])

    def test_reverse_route_preserves_excursion_and_step_limit(self):
        start = self.home
        points = []
        for offset in (0.1, 0.2, 0.3):
            point = list(start)
            point[0] += offset
            points.append(point)
        middle = points[-1]
        for offset in (0.1, 0.2):
            point = list(middle)
            point[1] -= offset
            points.append(point)
        end = points[-1]
        route = plan_reverse_return(points, end, start, self.config)
        self.assertLess(math.dist(route[0], end), 1e-9)
        self.assertLess(math.dist(route[-1], start), 1e-9)
        self.assertTrue(any(math.dist(point, middle) < 0.03 for point in route))
        self.assertTrue(all(
            math.dist(a, b) <= self.config["return_max_joint_step_rad"] + 1e-9
            for a, b in zip(route, route[1:])
        ))

    def test_unknown_endpoint_cannot_trigger_automatic_return(self):
        last = list(self.home)
        last[0] += 0.2
        current = list(last)
        current[0] += 0.2
        with self.assertRaisesRegex(ValueError, "最后一个记录点"):
            plan_reverse_return([last], current, self.home, self.config)

    def test_sparse_feedback_cannot_be_replayed(self):
        jumped = list(self.home)
        jumped[0] += 0.2
        with self.assertRaisesRegex(ValueError, "轨迹不连续"):
            plan_reverse_return([jumped], jumped, self.home, self.config)

    def test_recorded_zero_pose_blocks_return(self):
        with self.assertRaisesRegex(ValueError, "全零位"):
            plan_reverse_return([[0.0] * 7], [0.0] * 7, self.home, self.config)

    def test_joint_limit_error_reports_angle_and_range(self):
        point = list(self.home)
        point[3] = JOINT_LIMITS[3][0] - 0.01
        with self.assertRaisesRegex(ValueError, "关节 4 当前 -1.022291 rad.*SDK 原始限位"):
            validate_joints(point, self.config["zero_exclusion_radius_rad"])

    def test_incomplete_error_record_cannot_be_returned(self):
        path = ROOT / "experiments/single_arm/teaching/data/recordings/nero_session_20260923T151241Z.csv"
        with self.assertRaisesRegex(ValueError, "缺少退出拖动后的停稳轨迹"):
            load_recording(path, self.config)

    def test_legacy_reverse_run_is_disabled_before_connect(self):
        with patch.object(sys, "argv", ["return_session", "old.csv", "--run"]), patch.object(
            return_session.AgxArmFactory, "create_arm"
        ) as factory:
            self.assertEqual(return_session.main(), 1)
        factory.assert_not_called()

    def test_fk_workspace_checks_path_between_waypoints(self):
        config = dict(self.config)
        config["flange_workspace_m"] = {
            "x": [-0.01, 0.10], "y": [-0.10, 0.10], "z": [0.50, 0.70],
        }

        class FakeRobot:
            @staticmethod
            def fk(joints):
                x = joints[0] * (1 - joints[0])
                return [x, 0.0, 0.6, 0.0, 0.0, 0.0]

        first = [0.0] * 7
        last = [1.0] + [0.0] * 6
        validate_flange_workspace(FakeRobot(), first, config)
        validate_flange_workspace(FakeRobot(), last, config)
        with self.assertRaisesRegex(ValueError, "法兰中心 x"):
            validate_route_workspace(FakeRobot(), [first, last], config)

    def test_ignored_return_command_triggers_stop(self):
        config = dict(self.config)
        config["return_waypoint_timeout_s"] = 0.15
        end = list(self.home)
        end[0] += 0.1

        class UnresponsiveRobot:
            emergency_stopped = False

            def get_arm_status(self):
                msg = SimpleNamespace(
                    ctrl_mode=1, arm_status=0, teach_status=6,
                    mode_feedback=1, err_code=0,
                )
                return SimpleNamespace(msg=msg, timestamp=time.time())

            def get_joint_angles(self):
                return SimpleNamespace(msg=end, timestamp=time.time())

            def get_driver_states(self, _index):
                foc = SimpleNamespace(
                    driver_enable_status=True, voltage_too_low=False, driver_error_status=False,
                )
                return SimpleNamespace(msg=SimpleNamespace(foc_status=foc), timestamp=time.time())

            def set_speed_percent(self, _percent):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, _joints):
                pass  # 模拟控制器接受指令却没有实际位移。

            def electronic_emergency_stop(self):
                self.emergency_stopped = True

        robot = UnresponsiveRobot()
        with patch.object(teach_session, "send_frame"):
            with self.assertRaises(TimeoutError):
                teach_session.return_to_start(robot, object(), [self.home, end], self.home, config, lambda: False)
        self.assertTrue(robot.emergency_stopped)

        # 只有连续反馈证明确实停在发送前位置时，超时才暂停而不急停。
        robot = UnresponsiveRobot()
        with patch.object(teach_session, "send_frame"), patch(
            "experiments.single_arm.start_transfer.go_to_start."
            "confirm_stationary_at_previous", return_value=True,
        ):
            from experiments.single_arm.start_transfer.go_to_start import RoutePaused
            with self.assertRaises(RoutePaused):
                teach_session.return_to_start(
                    robot, object(), [self.home, end], self.home, config, lambda: False
                )
        self.assertFalse(robot.emergency_stopped)

    def test_live_joint_feedback_jump_stops_drag_without_return(self):
        config = dict(self.config)
        frames = []

        class JumpingRobot:
            def __init__(self, home):
                self.home = list(home)
                self.mode = 2
                self.teach = 0
                self.samples = 0
                self.move_calls = 0

            def get_arm_status(self):
                msg = SimpleNamespace(
                    ctrl_mode=self.mode, arm_status=0, teach_status=self.teach,
                    motion_status=0, mode_feedback=1, err_code=0,
                )
                return SimpleNamespace(msg=msg, timestamp=time.time())

            def get_joint_angles(self):
                joints = list(self.home)
                if self.teach == 1:
                    self.samples += 1
                    joints[0] += 0.01 if self.samples < 3 else 0.30
                return SimpleNamespace(msg=joints, timestamp=time.time())

            def get_flange_pose(self):
                return SimpleNamespace(msg=[0.3, 0.0, 0.6, 0.0, 0.0, 0.0], timestamp=time.time())

            def get_driver_states(self, _index):
                foc = SimpleNamespace(
                    driver_enable_status=True, voltage_too_low=False, driver_error_status=False,
                )
                return SimpleNamespace(msg=SimpleNamespace(foc_status=foc), timestamp=time.time())

            def move_j(self, _joints):
                self.move_calls += 1

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = JumpingRobot(self.home)

        def apply_frame(_bus, can_id, data, **_kwargs):
            frames.append((can_id, data[2] if can_id == 0x150 else None))
            if can_id == 0x151:
                robot.mode = 1
            elif data[2] == 1:
                robot.mode = 2
                robot.teach = 1
            elif data[2] == 2:
                robot.teach = 0

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "session.csv"
            args = SimpleNamespace(max_seconds=1.0, output=output)
            with patch.object(teach_session.can, "Bus", return_value=FakeBus()), patch.object(
                teach_session, "send_frame", side_effect=apply_frame
            ), patch.object(teach_session, "authorize_s1_drag_start", return_value=object()), patch.object(
                teach_session, "wait_drag_entry_guarded", return_value=robot.get_arm_status()
            ):
                with self.assertRaisesRegex(ValueError, "相邻关节反馈跳变过大"):
                    teach_session.run_session(robot, config, args)
            metadata = json.loads(output.with_suffix(".json").read_text())

        self.assertEqual(metadata["stop_reason"], "error")
        self.assertFalse(metadata["return_completed"])
        self.assertEqual([frame for frame in frames if frame[0] == 0x150], [(0x150, 1), (0x150, 2)])
        self.assertEqual(robot.move_calls, 0)

    def test_time_limit_stops_teaching_before_return(self):
        config = dict(self.config)
        config["return_countdown_seconds"] = 0
        frames = []

        class FakeRobot:
            def __init__(self, joints):
                self.home = list(joints)
                self.joints = list(joints)
                self.mode = 2
                self.teach = 0
                self.teach_samples = 0
                self.move_calls = 0

            def get_arm_status(self):
                msg = SimpleNamespace(
                    ctrl_mode=self.mode, arm_status=0, teach_status=self.teach,
                    motion_status=0, mode_feedback=1, err_code=0,
                )
                return SimpleNamespace(msg=msg, timestamp=time.time())

            def get_joint_angles(self):
                if self.teach == 1:
                    self.teach_samples += 1
                    self.joints = list(self.home)
                    self.joints[0] += min(0.01 * self.teach_samples, 0.10)
                return SimpleNamespace(msg=self.joints, timestamp=time.time())

            def get_flange_pose(self):
                return SimpleNamespace(msg=[0.3, 0.0, 0.6, 0.0, 0.0, 0.0], timestamp=time.time())

            def get_driver_states(self, _index):
                foc = SimpleNamespace(
                    driver_enable_status=True, voltage_too_low=False, driver_error_status=False,
                )
                return SimpleNamespace(msg=SimpleNamespace(foc_status=foc, vol=23.8), timestamp=time.time())

            def fk(self, _joints):
                return [0.3, 0.0, 0.6, 0.0, 0.0, 0.0]

            def set_speed_percent(self, _percent):
                pass

            def set_motion_mode(self, _mode):
                pass

            def move_j(self, joints):
                self.move_calls += 1
                self.joints = list(joints)

        class FakeBus:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

        robot = FakeRobot(self.home)

        def apply_frame(_bus, can_id, data, **_kwargs):
            frames.append((can_id, data[2] if can_id == 0x150 else None))
            if can_id == 0x151:
                robot.mode = 1
                robot.teach = 6  # 固件切回 CAN 控制后可能保留“终止执行”反馈。
            elif data[2] == 1:
                robot.mode = 2
                robot.teach = 1
            elif data[2] == 2:
                robot.teach = 0
                # 模拟真实机械臂退出拖动后继续位移，超过旧版 0.05 rad 端点阈值。
                robot.joints[2] -= 0.078

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "session.csv"
            args = SimpleNamespace(max_seconds=0.2, output=output)
            with patch.object(teach_session.can, "Bus", return_value=FakeBus()), patch.object(
                teach_session, "send_frame", side_effect=apply_frame
            ), patch.object(teach_session, "authorize_s1_drag_start", return_value=object()), patch.object(
                teach_session, "wait_drag_entry_guarded", return_value=robot.get_arm_status()
            ), patch(
                "experiments.single_arm.start_transfer.go_to_start.send_frame",
                side_effect=apply_frame,
            ):
                teach_session.run_session(robot, config, args)
            with output.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            metadata = json.loads(output.with_suffix(".json").read_text())

        self.assertGreaterEqual(len(rows), 2)
        self.assertEqual([frame for frame in frames if frame[0] == 0x150], [(0x150, 1), (0x150, 2)])
        self.assertEqual(metadata["stop_reason"], "time_limit")
        self.assertTrue(metadata["return_completed"])
        self.assertGreater(metadata["post_stop_samples"], 1)
        self.assertEqual(rows[-1]["teach_status"], "0")
        self.assertGreater(robot.move_calls, 0)
        self.assertLess(math.dist(robot.joints, self.home), 1e-9)
        self.assertEqual(robot.teach, 6)


if __name__ == "__main__":
    unittest.main()
