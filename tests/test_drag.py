"""F4 限定离线：真实 F1 采集 + 可注入 Drag 内核 + 无动作 Fake。"""

import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest

from nero_runtime.acquisition import collect_snapshot
from nero_runtime.drag import enter_drag, exit_drag, inspect_drag_state


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def wall(self):
        return 100.0 + self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeNero:
    def __init__(self, clock):
        self.clock = clock
        self.mode = "normal"
        self.enter_after = 2
        self.exit_after = 2
        self.pending = None
        self.pending_reads = 0
        self.calls = []
        self.status_reads = 0
        self.fault_code = 0
        self.control_mode_override = None
        self.fault_after = None
        self.disabled_joint = None
        self.joint_missing = False
        self.joint_stale = False
        self.flange_x = -0.18
        self.joint_shift_after = None
        self.enter_error = None
        self.exit_error = None
        self.control_calls = {name: 0 for name in
                              ("move_j", "enable", "disable", "reset", "hold",
                               "electronic_emergency_stop", "recording")}

    def set_leader_mode(self):
        self.calls.append("set_leader_mode")
        if self.enter_error:
            raise self.enter_error
        self.pending, self.pending_reads = "drag", 0

    def set_normal_mode(self):
        self.calls.append("set_normal_mode")
        if self.exit_error:
            raise self.exit_error
        self.pending, self.pending_reads = "normal", 0

    def get_arm_status(self):
        self.calls.append("get_arm_status")
        self.status_reads += 1
        if self.pending is not None:
            self.pending_reads += 1
            threshold = self.enter_after if self.pending == "drag" else self.exit_after
            if threshold is not None and self.pending_reads >= threshold:
                self.mode, self.pending = self.pending, None
        code = self.fault_code
        if self.fault_after is not None and self.status_reads >= self.fault_after:
            code = 9
        return SimpleNamespace(
            timestamp=self.clock.wall() - 0.1,
            msg=SimpleNamespace(ctrl_mode=self.control_mode_override if self.control_mode_override is not None else
                                (1 if self.mode == "normal" else 2),
                                arm_status=0, teach_status=0 if self.mode == "normal" else 1,
                                motion_status=0, err_code=code))

    def get_joint_angles(self):
        self.calls.append("get_joint_angles")
        if self.joint_missing:
            return None
        joint = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]
        if self.joint_shift_after is not None and self.status_reads >= self.joint_shift_after:
            joint[0] += 0.2
        timestamp = self.clock.wall() - (3.0 if self.joint_stale else 0.1)
        return SimpleNamespace(timestamp=timestamp, msg=joint)

    def get_flange_pose(self):
        self.calls.append("get_flange_pose")
        return SimpleNamespace(timestamp=self.clock.wall() - 0.1,
                               msg=[self.flange_x, 0.01, 0.7, 0, 0, 0])

    def get_driver_states(self, joint):
        self.calls.append(f"get_driver_states({joint})")
        return SimpleNamespace(
            timestamp=self.clock.wall() - 0.1,
            msg=SimpleNamespace(vol=24.0,
                                foc_status=SimpleNamespace(
                                    driver_enable_status=joint != self.disabled_joint,
                                    voltage_too_low=False, driver_error_status=False)))

    def __getattr__(self, name):
        if name in self.control_calls:
            raise AssertionError(f"forbidden control method reached: {name}")
        raise AttributeError(name)


def synthetic_active(snapshot):
    if snapshot.control_mode == 2 and snapshot.teach_status == 1:
        return True
    if snapshot.control_mode == 1 and snapshot.teach_status == 0:
        return False
    return None


class DragTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.robot = FakeNero(self.clock)

    def read(self):
        return collect_snapshot(
            self.robot, robot_id="fake-nero", clock_s=self.clock.wall,
            time_basis="unix_epoch_s", max_status_age_s=1,
            max_joints_age_s=1, max_flange_age_s=1, max_driver_age_s=1)

    def enter(self, **changes):
        kwargs = dict(read_snapshot=self.read,
                      request_enter=self.robot.set_leader_mode,
                      request_exit=self.robot.set_normal_mode,
                      drag_active_predicate=synthetic_active,
                      monotonic_s=self.clock.monotonic,
                      sleep_s=self.clock.sleep,
                      cancel_requested=lambda: False,
                      timeout_s=0.3, poll_interval_s=0.05)
        kwargs.update(changes)
        return enter_drag(**kwargs)

    def exit(self, **changes):
        kwargs = dict(read_snapshot=self.read,
                      request_exit=self.robot.set_normal_mode,
                      drag_active_predicate=synthetic_active,
                      monotonic_s=self.clock.monotonic,
                      sleep_s=self.clock.sleep,
                      cancel_requested=lambda: False,
                      timeout_s=0.3, poll_interval_s=0.05)
        kwargs.update(changes)
        return exit_drag(**kwargs)

    def assert_no_other_controls(self):
        self.assertTrue(all(count == 0 for count in self.robot.control_calls.values()))

    def test_real_f1_enter_inspect_exit_positive_chain(self):
        entered = self.enter()
        self.assertEqual(entered.outcome, "confirmed")
        self.assertTrue(entered.enter_requested and entered.enter_confirmed)
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 1)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 0)
        self.assertEqual(entered.start_joint_rad,
                         (0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7))
        self.assertEqual(entered.max_joint_delta_rad, 0)
        self.assertGreater(entered.observations[-1].snapshot.status_quality.timestamp_s,
                           entered.observations[0].snapshot.status_quality.timestamp_s)
        inspected = inspect_drag_state(read_snapshot=self.read,
                                       drag_active_predicate=synthetic_active)
        self.assertEqual(inspected.state, "active")
        self.assertEqual(inspected.controller_fault, False)
        exited = self.exit()
        self.assertEqual(exited.outcome, "confirmed")
        self.assertTrue(exited.exit_request_attempted and exited.exit_requested and exited.exit_confirmed)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertEqual(inspect_drag_state(read_snapshot=self.read,
                                            drag_active_predicate=synthetic_active).state, "inactive")
        self.assert_no_other_controls()
        self.assertNotIn("pyAgxArm", sys.modules)
        self.assertNotIn("can", sys.modules)
        self.assertNotIn("nero_runtime.recording", sys.modules)

    def test_controller_fault_rejects_before_control(self):
        self.robot.fault_code = 9
        result = self.enter()
        self.assertEqual(result.outcome, "rejected")
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 0)

    def test_disabled_driver_rejects_before_control(self):
        self.robot.disabled_joint = 4
        result = self.enter()
        self.assertEqual(result.outcome, "rejected")
        self.assertIn("J4", result.reason)
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)

    def test_missing_and_stale_joints_reject_before_control(self):
        for name in ("joint_missing", "joint_stale"):
            with self.subTest(name=name):
                setattr(self.robot, name, True)
                result = self.enter()
                self.assertEqual(result.outcome, "rejected")
                self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)
                setattr(self.robot, name, False)

    def test_negative_flange_x_does_not_reject(self):
        self.robot.flange_x = -0.4
        result = self.enter()
        self.assertEqual(result.outcome, "confirmed")
        self.assertEqual(result.observations[0].snapshot.flange_pose_m_rad[0], -0.4)

    def test_enter_request_exception_reports_attempt_and_rolls_back(self):
        self.robot.enter_error = OSError("synthetic leader write failed")
        result = self.enter()
        self.assertEqual(result.outcome, "fault")
        self.assertTrue(result.enter_request_attempted)
        self.assertFalse(result.enter_requested or result.enter_confirmed)
        self.assertIn("synthetic leader write failed", result.request_error)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertTrue(result.exit_request_attempted)

    def test_enter_unconfirmed_requests_one_exit_rollback(self):
        self.robot.enter_after = None
        result = self.enter()
        self.assertEqual(result.outcome, "fault")
        self.assertTrue(result.enter_requested)
        self.assertFalse(result.enter_confirmed)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertEqual(result.exit_request_attempted, True)

    def test_enter_fault_after_request_requests_exit(self):
        self.robot.fault_after = 2
        result = self.enter()
        self.assertEqual(result.outcome, "fault")
        self.assertIn("controller", result.reason)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)

    def test_explicit_transition_delta_limit_requests_exit(self):
        self.robot.joint_shift_after = 2
        result = self.enter(transition_joint_delta_limit_rad=0.05,
                            transition_limit_source="synthetic test only")
        self.assertEqual(result.outcome, "fault")
        self.assertGreater(result.max_joint_delta_rad, 0.19)
        self.assertEqual(result.transition_joint_delta_limit_rad, 0.05)
        self.assertEqual(result.transition_limit_source, "synthetic test only")
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        with self.assertRaises(ValueError):
            self.enter(transition_joint_delta_limit_rad=0.05)

    def test_exit_request_exception_is_not_confirmed(self):
        self.robot.mode = "drag"
        self.robot.exit_error = OSError("synthetic follower write failed")
        result = self.exit()
        self.assertEqual(result.outcome, "fault")
        self.assertTrue(result.exit_request_attempted)
        self.assertFalse(result.exit_requested or result.exit_confirmed)
        self.assertIn("synthetic follower write failed", result.request_error)

    def test_exit_request_return_without_feedback_is_unconfirmed(self):
        self.robot.mode = "drag"
        self.robot.exit_after = None
        result = self.exit()
        self.assertEqual(result.outcome, "unconfirmed")
        self.assertTrue(result.exit_requested)
        self.assertFalse(result.exit_confirmed)

    def test_enter_and_rollback_errors_keep_both_attempts(self):
        self.robot.enter_error = OSError("synthetic leader send failed")
        self.robot.exit_error = OSError("synthetic exit send failed")
        result = self.enter()
        self.assertEqual(result.outcome, "fault")
        self.assertTrue(result.enter_request_attempted and result.exit_request_attempted)
        self.assertFalse(result.enter_requested or result.exit_requested or result.exit_confirmed)
        self.assertIn("leader send failed", result.request_error)
        self.assertIn("exit send failed", result.rollback_error)

    def test_keyboard_interrupt_during_enter_confirmation_rolls_back(self):
        initial_read = self.read
        def interrupted_read():
            if self.robot.status_reads >= 1:
                raise KeyboardInterrupt
            return initial_read()
        result = self.enter(read_snapshot=interrupted_read)
        self.assertEqual(result.outcome, "cancelled")
        self.assertTrue(result.enter_requested)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertFalse(result.exit_confirmed)
        self.assert_no_other_controls()

    def test_cancel_during_enter_confirmation_rolls_back(self):
        checks = [False]
        def cancel():
            checks.append(True)
            return len(checks) >= 3
        result = self.enter(cancel_requested=cancel)
        self.assertEqual(result.outcome, "cancelled")
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assert_no_other_controls()

    def test_repeated_enter_rejects_and_inactive_exit_is_noop(self):
        self.robot.mode = "drag"
        result = self.enter()
        self.assertEqual(result.outcome, "rejected")
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)
        self.robot.mode = "normal"
        result = self.exit()
        self.assertEqual(result.outcome, "confirmed")
        self.assertFalse(result.exit_request_attempted or result.exit_requested)
        self.assertTrue(result.exit_confirmed)

    def test_conflicting_mode_rejects_and_import_is_isolated(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        command = [sys.executable, "-B", "-S", "-c",
                   "import sys; import nero_runtime.drag; "
                   "assert 'pyAgxArm' not in sys.modules; "
                   "assert 'can' not in sys.modules; "
                   "assert 'experiments' not in sys.modules; "
                   "assert 'nero_runtime.recording' not in sys.modules; "
                   "assert 'nero_runtime.motion' not in sys.modules"]
        completed = subprocess.run(command, cwd=root, env=environment,
                                   capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.robot.control_mode_override = 3  # SDK enum: Ethernet owner, not CAN control.
        result = self.enter(drag_active_predicate=lambda snapshot: False)
        self.assertEqual(result.outcome, "rejected")
        self.assertIn("conflicting", result.reason)
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)


if __name__ == "__main__":
    unittest.main()
