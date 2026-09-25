"""离线验证单次法兰发布与预检拒绝条件；不会连接 CAN。"""

import math
import time
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.control_interfaces.cartesian_linear_session import (
    StationaryIncomplete, check_motion_drivers, main, preview_line, preview_point,
    read_anchor_recording, run_one_line,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class CartesianLinearSessionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(DEFAULT_CONFIG)
        cls.kinematics = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel="can0"
        ))
        cls.start = cls.config["safe_start_joint_rad"]
        cls.pose = list(cls.kinematics.fk(cls.start))

    def test_nearby_line_has_continuous_joint_preview(self):
        target = list(self.pose)
        target[0] -= 0.002
        route = preview_line(self.kinematics, self.start, self.pose, target, self.config)
        self.assertGreaterEqual(len(route), 2)
        self.assertLess(max(abs(a-b) for a,b in zip(route[-1], self.start)), 0.18)
        self.assertLess(math.dist(self.kinematics.fk(route[-1])[:3], target[:3]), 0.0002)

    def test_far_target_is_rejected_before_publication(self):
        target = list(self.pose)
        target[0] -= 0.1
        with self.assertRaisesRegex(ValueError, "30 mm"):
            preview_line(self.kinematics, self.start, self.pose, target, self.config)

    def test_point_preview_rejects_large_first_trial(self):
        target = list(self.pose)
        target[0] -= 0.01
        with self.assertRaisesRegex(ValueError, "1～5 mm"):
            preview_point(self.kinematics, self.start, self.pose, target, self.config)

    def test_stable_csv_can_be_temporary_anchor(self):
        path = Path("experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T140821Z.csv")
        anchor, pose = read_anchor_recording(path, self.kinematics, self.config)
        self.assertAlmostEqual(anchor[0], 0.04961971063419879)
        target = list(pose)
        target[0] -= 0.002
        route = preview_line(self.kinematics, anchor, pose, target, self.config, anchor)
        self.assertLess(max(abs(a-b) for a,b in zip(route[-1], anchor)), 0.18)

    def test_documented_anchor_cli_is_read_only(self):
        path = Path("experiments/single_arm/pose_recording/data/recordings/nero_poses_20260924T140821Z.csv")
        anchor, pose = read_anchor_recording(path, self.kinematics, self.config)
        robot = Mock()
        robot.fk.side_effect = self.kinematics.fk
        argv = ["cartesian_linear_session", "--anchor-recording", str(path),
                "--dx-m", "-0.002"]
        output = StringIO()
        with (patch("sys.argv", argv),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.AgxArmFactory.create_arm", return_value=robot),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.wait_status", return_value=object()),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status"),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_drivers"),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.stable_joint_feedback", return_value=anchor),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=pose),
              redirect_stdout(output)):
            self.assertEqual(main(), 0)
        self.assertIn("未发送运动指令", output.getvalue())
        robot.move_l.assert_not_called()
        robot.set_speed_percent.assert_not_called()

    def test_point_cli_default_is_read_only(self):
        robot = Mock()
        output = StringIO()
        with (patch("sys.argv", ["cartesian_point_session", "--dx-m", "-0.002"]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.AgxArmFactory.create_arm", return_value=robot),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.wait_status", return_value=object()),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status"),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_drivers"),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.stable_joint_feedback", return_value=self.start),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=self.pose),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.preview_point", return_value=[self.start, self.start]),
              redirect_stdout(output)):
            self.assertEqual(main(motion_mode="p"), 0)
        self.assertIn("一次 move_p", output.getvalue())
        self.assertIn("未发送运动指令", output.getvalue())
        robot.move_p.assert_not_called()
        robot.set_speed_percent.assert_not_called()

    def test_one_command_uses_move_l_only(self):
        target = list(self.pose)
        target[0] -= 0.002
        route = [list(self.start), list(self.start)]

        class FakeRobot:
            def __init__(self):
                self.calls = []

            def set_speed_percent(self, speed):
                self.calls.append(("speed", speed))

            def move_l(self, pose):
                self.calls.append(("move_l", pose))

            def get_flange_pose(self):
                return object()

            def get_ik_joint_angles(self):
                return None

            def get_driver_states(self, _index):
                # 状态帧暂时缺失时，运动监控应继续看高频关节与法兰反馈。
                return None

        robot = FakeRobot()
        report = {}
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=0))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status", return_value=status),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=self.start),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=target),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.route_error", return_value=0.0)):
            run_one_line(robot, self.pose, target, route, 5, report, lambda: False)
        self.assertEqual([name for name, _ in robot.calls], ["speed", "move_l"])
        self.assertTrue(report["completed"])
        self.assertEqual(len(report["feedback_samples"]), 1)
        self.assertEqual(report["feedback_samples"][0]["stale_driver_feedback"], list(range(1, 8)))

    def test_motion_driver_fault_is_not_ignored(self):
        foc = SimpleNamespace(driver_enable_status=True, voltage_too_low=False,
                              driver_error_status=False)
        bad_foc = SimpleNamespace(driver_enable_status=False, voltage_too_low=False,
                                  driver_error_status=False)
        messages = [SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(foc_status=foc))
                    for _ in range(7)]
        messages[3].msg.foc_status = bad_foc
        robot = SimpleNamespace(get_driver_states=lambda index: messages[index - 1])
        with self.assertRaisesRegex(RuntimeError, "关节 4 未使能"):
            check_motion_drivers(robot)

    def test_cancel_before_send_does_not_publish(self):
        class FakeRobot:
            def __init__(self):
                self.calls = []

            def set_speed_percent(self, speed):
                self.calls.append(("speed", speed))

            def move_l(self, pose):
                self.calls.append(("move_l", pose))

        robot = FakeRobot()
        with self.assertRaises(InterruptedError):
            run_one_line(robot, self.pose, self.pose, [self.start, self.start], 5, {}, lambda: True)
        self.assertEqual(robot.calls, [])

    def test_flange_return_with_wrong_joint_branch_is_not_success(self):
        target = list(self.pose)
        target[0] -= 0.002

        class FakeRobot:
            def __init__(self):
                self.calls = []

            def set_speed_percent(self, speed):
                self.calls.append("speed")

            def move_l(self, pose):
                self.calls.append("move_l")

            def get_flange_pose(self):
                return object()

            def get_ik_joint_angles(self):
                return None

            def electronic_emergency_stop(self):
                self.calls.append("stop")

        robot = FakeRobot()
        report = {}
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=0))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status", return_value=status),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_motion_drivers", return_value=[]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=self.start),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=target),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.route_error", return_value=0.0)):
            expected = list(self.start)
            expected[6] += 0.1
            with self.assertRaises(StationaryIncomplete):
                run_one_line(robot, self.pose, target, [self.start, self.start], 5, report,
                             lambda: False, expected_joint_rad=expected)
        self.assertEqual(robot.calls, ["speed", "move_l"])
        self.assertTrue(report["flange_reached"])
        self.assertFalse(report.get("completed", False))

    def test_stopped_controller_does_not_receive_extra_emergency_stop(self):
        target = list(self.pose)
        target[0] -= 0.002

        class FakeRobot:
            def __init__(self):
                self.calls = []

            def set_speed_percent(self, speed):
                self.calls.append("speed")

            def move_l(self, pose):
                self.calls.append("move_l")

            def get_flange_pose(self):
                return object()

            def get_ik_joint_angles(self):
                return None

            def electronic_emergency_stop(self):
                self.calls.append("stop")

        robot = FakeRobot()
        report = {}
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=0))
        clock = iter(range(100))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status", return_value=status),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_motion_drivers", return_value=[]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=self.start),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=self.pose),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.route_error", return_value=0.0),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.time.monotonic", side_effect=lambda: next(clock)),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.time.sleep")):
            with self.assertRaises(StationaryIncomplete):
                run_one_line(robot, self.pose, target, [self.start, self.start], 5, report, lambda: False)
        self.assertEqual(robot.calls, ["speed", "move_l"])
        self.assertFalse(report.get("electronic_emergency_stop_sent", False))

    def test_controller_ik_joint_path_difference_does_not_trigger_stop(self):
        """控制器沿法兰直线到位时，不把本机 IK 关节折线当作实际路线。"""
        target = list(self.pose)
        target[0] -= 0.002
        actual = list(self.start)
        actual[0] += 0.09
        robot = Mock()
        robot.get_ik_joint_angles.return_value = None
        report = {}
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=0))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status", return_value=status),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_motion_drivers", return_value=[]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=actual),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=target)):
            run_one_line(robot, self.pose, target, [self.start, self.start], 5, report,
                         lambda: False)
        self.assertTrue(report["completed"])
        self.assertGreater(report["feedback_samples"][0]["predicted_joint_path_error_rad"], 0.08)
        robot.electronic_emergency_stop.assert_not_called()

    def test_moving_flange_outside_line_triggers_stop(self):
        target = list(self.pose)
        target[0] -= 0.002
        off_line = list(self.pose)
        off_line[1] += 0.01
        robot = Mock()
        robot.get_ik_joint_angles.return_value = None
        report = {}
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=1))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status", return_value=status),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_motion_drivers", return_value=[]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=self.start),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=off_line)):
            with self.assertRaisesRegex(RuntimeError, "超出本次运动范围"):
                run_one_line(robot, self.pose, target, [self.start, self.start], 5, report,
                             lambda: False)
        robot.electronic_emergency_stop.assert_called_once()

    def test_point_mode_accepts_nonstraight_flange_position_inside_small_envelope(self):
        target = list(self.pose)
        target[0] -= 0.002
        bent_pose = list(self.pose)
        bent_pose[1] += 0.01
        robot = Mock()
        robot.get_ik_joint_angles.return_value = None
        report = {}
        statuses = iter((1, 0))
        poses = iter((bent_pose, target))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status",
                    side_effect=lambda *_args: SimpleNamespace(msg=SimpleNamespace(
                        motion_status=next(statuses)))),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_motion_drivers", return_value=[]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=self.start),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose",
                    side_effect=lambda *_args: next(poses)),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.time.sleep")):
            run_one_line(robot, self.pose, target, [self.start, self.start], 5, report,
                         lambda: False, motion_mode="p")
        robot.move_p.assert_called_once()
        robot.move_l.assert_not_called()
        self.assertTrue(report["completed"])
        self.assertGreater(report["feedback_samples"][0]["flange_line_error_m"], 0.005)
        robot.electronic_emergency_stop.assert_not_called()

    def test_moving_joint_limit_violation_requests_stop(self):
        target = list(self.pose)
        target[0] -= 0.002
        impossible = list(self.start)
        impossible[3] = -20.0
        robot = Mock()
        report = {}
        status = SimpleNamespace(msg=SimpleNamespace(motion_status=1))
        with (patch("experiments.single_arm.control_interfaces.cartesian_linear_session.healthy_status", return_value=status),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.check_motion_drivers", return_value=[]),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.joint_point", return_value=impossible),
              patch("experiments.single_arm.control_interfaces.cartesian_linear_session.checked_pose", return_value=self.pose)):
            with self.assertRaisesRegex(ValueError, "已发送阻尼电子急停"):
                run_one_line(robot, self.pose, target, [self.start, self.start], 5, report,
                             lambda: False, motion_mode="p")
        robot.electronic_emergency_stop.assert_called_once()


if __name__ == "__main__":
    unittest.main()
