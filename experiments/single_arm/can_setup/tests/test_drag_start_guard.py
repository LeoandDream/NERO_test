"""S1-only drag authorization and incident hold; no hardware access."""

import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.single_arm.can_setup import can_drag_teach
from experiments.single_arm.can_setup.drag_start_guard import (
    DragStartAuthorization, DragStartBlocked, S1_FLANGE_M, S1_JOINT_RAD,
    authorize_s1_drag_start,
)
from experiments.single_arm.lab import cli as lab_cli
from experiments.single_arm.teaching import recover_record_endpoint, teach_session


class FakeBus:
    def __init__(self):
        self.frames = []

    def send(self, frame, timeout):
        self.frames.append((frame, timeout))


class FakeRobot:
    def __init__(self, joints=S1_JOINT_RAD, mode=1):
        self.joints = joints
        self.mode = mode

    def get_arm_status(self):
        return SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(
            ctrl_mode=self.mode, arm_status=0, teach_status=0, err_code=0))

    def get_joint_angles(self):
        return SimpleNamespace(timestamp=time.time(), msg=self.joints)

    def get_flange_pose(self):
        return SimpleNamespace(timestamp=time.time(), msg=[*S1_FLANGE_M, 0, 0, 0])

    def get_driver_states(self, _index):
        return SimpleNamespace(timestamp=time.time(), msg=SimpleNamespace(
            foc_status=SimpleNamespace(driver_enable_status=True,
                                       voltage_too_low=False,
                                       driver_error_status=False)))


class DragStartGuardTests(unittest.TestCase):
    def test_drag_transition_frames_are_blocked_before_bus_write(self):
        bus = FakeBus()
        for frame_id, payload in (
            (0x151, [1, 0xFF, 50, 0, 0, 2, 0, 0]),
            (0x151, [1, 0xFF, 5, 0, 0, 2, 0, 0]),
            (0x150, [0, 0, 1, 0, 0, 0, 0, 0]),
        ):
            with self.assertRaises(DragStartBlocked):
                can_drag_teach.send_frame(bus, frame_id, payload)
        self.assertEqual(bus.frames, [])

    def test_stop_and_non_drag_mode_frame_remain_available(self):
        bus = FakeBus()
        can_drag_teach.send_frame(bus, 0x150, [0, 0, 2, 0, 0, 0, 0, 0])
        can_drag_teach.send_frame(bus, 0x151, [1, 1, 5, 0, 0, 2, 0, 0])
        self.assertEqual(len(bus.frames), 2)

    def test_direct_start_rejected_before_robot_creation(self):
        with patch.object(sys, "argv", ["can_drag_teach", "--start", "--mount", "left"]), patch.object(
            can_drag_teach.AgxArmFactory, "create_arm"
        ) as factory:
            self.assertEqual(can_drag_teach.main(), 2)
        factory.assert_not_called()

    def test_s1_authorization_allows_only_fresh_drag_transition_frames(self):
        bus = FakeBus()
        with patch("experiments.single_arm.can_setup.drag_start_guard.time.sleep"):
            authorization = authorize_s1_drag_start(FakeRobot(), mount="left")
        can_drag_teach.send_frame(bus, 0x151, [1, 0xFF, 50, 0, 0, 2, 0, 0],
                                  drag_authorization=authorization)
        can_drag_teach.send_frame(bus, 0x150, [0, 0, 1, 0, 0, 0, 0, 0],
                                  drag_authorization=authorization)
        self.assertEqual(len(bus.frames), 2)
        with self.assertRaises(DragStartBlocked):
            can_drag_teach.send_frame(bus, 0x150, [0, 0, 1, 0, 0, 0, 0, 0],
                                      drag_authorization=DragStartAuthorization(time.monotonic()-3))

    def test_low_pose_cannot_authorize_drag(self):
        low = list(S1_JOINT_RAD)
        low[1] += 0.2
        with self.assertRaisesRegex(DragStartBlocked, "不在已验证的 S1"):
            authorize_s1_drag_start(FakeRobot(low), mount="left")

    def test_other_mount_cannot_authorize_drag(self):
        with self.assertRaisesRegex(DragStartBlocked, "左侧安装"):
            authorize_s1_drag_start(FakeRobot(), mount="horizontal")

    def test_mount_frame_can_start_from_idle_mode_two_but_drag_frame_requires_can_mode(self):
        with patch("experiments.single_arm.can_setup.drag_start_guard.time.sleep"):
            authorize_s1_drag_start(FakeRobot(mode=2), mount="left")
            with self.assertRaisesRegex(DragStartBlocked, "CAN 空闲"):
                authorize_s1_drag_start(FakeRobot(mode=2), mount="left", require_can_mode=True)

    def test_manual_endpoint_recovery_rejected_before_robot_creation(self):
        with patch.object(sys, "argv", ["recover_record_endpoint", "missing.csv", "--run"]), patch.object(
            recover_record_endpoint.AgxArmFactory, "create_arm"
        ) as factory:
            self.assertEqual(recover_record_endpoint.main(), 1)
        factory.assert_not_called()

    def test_lab_drag_entry_rejected_before_api_creation(self):
        with patch.object(lab_cli, "LabAPI") as api:
            self.assertEqual(lab_cli.main([
                "quick", "home-demo", "--duration-s", "10", "--rate-hz", "20", "--yes"
            ]), 1)
        api.assert_not_called()


if __name__ == "__main__":
    unittest.main()
