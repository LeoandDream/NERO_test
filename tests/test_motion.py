"""F2：只在合成设备/时钟/临时文件上运行真实路线准备与执行内核。"""

from contextlib import redirect_stderr, redirect_stdout
import io
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from nero_runtime.cli import main
from nero_runtime.motion import (
    RouteParameters, StopEvidence, execute_joint_route, prepare_joint_route,
)
from nero_runtime.snapshot import build_snapshot
from nero_runtime.waypoints import capture_waypoint, save_waypoint


START = (0.0,) * 7
FIRST = (0.1,) + (0.0,) * 6
SECOND = (0.1, -0.2) + (0.0,) * 5
PARAMS = RouteParameters(0.01, 0.01, 0.5, 0.01, 0.05, 0.002)


def feedback(joints=START, stamp=100.0, *, observed=None, status_error=0,
             joints_missing=False, driver_fault=False, motion_status=0):
    observed = stamp + 0.01 if observed is None else observed
    return build_snapshot(
        robot_id="synthetic-nero", observed_at_s=observed,
        time_basis="synthetic_seconds",
        status={"timestamp_s": observed, "ctrl_mode": 1, "arm_status": 0,
                "teach_status": 0, "motion_status": motion_status,
                "err_code": status_error},
        joints=None if joints_missing else {"timestamp_s": stamp,
                                             "joint_rad": list(joints)},
        flange={"timestamp_s": observed, "flange_pose_m_rad":
                [-0.2, 0.0, 0.5, 0.0, 0.0, 0.0]},
        drivers=[{"joint": index, "timestamp_s": observed,
                  "enabled": True, "undervoltage": False,
                  "driver_error": driver_fault and index == 4, "voltage_v": 24.0}
                 for index in range(1, 8)],
        max_status_age_s=1, max_joints_age_s=1,
        max_flange_age_s=1, max_driver_age_s=1,
    )


class FakeRig:
    def __init__(self, samples):
        self.samples = list(samples)
        self.last = self.samples[0]
        self.now = 0.0
        self.events = []
        self.published = []
        self.stops = []
        self.fail_publish = False
        self.fail_stop = False

    def preflight(self):
        self.events.append("preflight")

    def read(self):
        if self.samples:
            self.last = self.samples.pop(0)
        self.events.append(("read", self.last.joints_quality.timestamp_s))
        return self.last

    def publish(self, target):
        self.events.append(("publish", tuple(target)))
        self.published.append(tuple(target))
        if self.fail_publish:
            raise OSError("synthetic partial-send uncertainty")

    def stop(self, reason):
        self.events.append(("stop", reason))
        self.stops.append(reason)
        if self.fail_stop:
            raise OSError("synthetic stop request failed")
        return StopEvidence(True, False, "synthetic request only; no stop feedback")

    def sleep(self, seconds):
        self.now += seconds

    def run(self, route, cancel=None, preflight=None):
        return execute_joint_route(
            route, parameters=PARAMS,
            preflight_check=self.preflight if preflight is None else preflight,
            read_feedback=self.read, publish_target=self.publish,
            cancel_requested=(lambda: False) if cancel is None else cancel,
            stop_motion=self.stop, monotonic_s=lambda: self.now,
            sleep_s=self.sleep,
        )


class MotionTests(unittest.TestCase):
    def route(self, *targets):
        return prepare_joint_route(START, list(targets or (FIRST, SECOND)), raw_unit="rad")

    def test_saved_f1_points_prepare_and_execute_in_exact_order(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for index, (name, angles) in enumerate(
                    (("READY", START), ("negative-X-A", FIRST), ("PARK", SECOND))):
                path = Path(directory) / f"{index}.json"
                record = capture_waypoint(name, feedback(angles, 99.0 + index),
                                          source="synthetic F1 observation")
                save_waypoint(record, path)
                paths.append(path)
            route = prepare_joint_route(paths[0], paths[1:])
            self.assertEqual([point.name for point in route.targets], ["negative-X-A", "PARK"])
            self.assertEqual(route.targets[0].robot_id, "synthetic-nero")
            self.assertIsNone(route.targets[0].tool_id)
            self.assertIn("synthetic F1 observation", route.targets[0].source)
            self.assertEqual(route.targets[0].joint_rad, FIRST)

            rig = FakeRig([
                feedback(START, 100.0), feedback(START, 100.0),
                feedback((0.05,) + START[1:], 100.1),
                feedback(FIRST, 100.2), feedback(FIRST, 100.3),
                feedback((0.1, -0.1) + START[2:], 100.4),
                feedback(SECOND, 100.5), feedback(SECOND, 100.6),
            ])
            result = rig.run(route)
            self.assertEqual((result.status, result.attempted, result.published,
                              result.reached), ("completed", 2, 2, 2))
            self.assertEqual(result.last_target_index, 2)
            self.assertEqual(rig.published, [FIRST, SECOND])
            self.assertEqual(rig.stops, [])
            first_reached = rig.events.index(("read", 100.3))
            second_sent = rig.events.index(("publish", SECOND))
            self.assertLess(first_reached, second_sent)
            self.assertEqual(sum(event[0] == "publish" for event in rig.events
                                 if isinstance(event, tuple)), 2)

            def forbidden_source(_channel):
                raise AssertionError("route show reached a device source")

            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                code = main(["route", "show", *map(str, paths)],
                            source_factory=forbidden_source)
            self.assertEqual((code, stderr.getvalue()), (0, ""))
            self.assertIn("目标 1：negative-X-A", stdout.getvalue())
            self.assertIn("目标 2：PARK", stdout.getvalue())
            self.assertIn("与前点逐轴差", stdout.getvalue())
            self.assertIn("不是碰撞检查", stdout.getvalue())
            self.assertIn("工具=未知", stdout.getvalue())
            self.assertNotIn("pyAgxArm", sys.modules)

    def test_last_point_invalid_or_units_wrong_reject_before_any_publish(self):
        rig = FakeRig([feedback()])
        with self.assertRaisesRegex(ValueError, "non-finite"):
            prepare_joint_route(START, [FIRST, [0.0] * 6 + [math.nan]], raw_unit="rad")
        with self.assertRaisesRegex(ValueError, "raw_unit"):
            prepare_joint_route(START, [FIRST])
        with self.assertRaisesRegex(ValueError, "unit"):
            prepare_joint_route(START, [FIRST], raw_unit="deg")
        self.assertEqual(rig.published, [])

    def test_input_mutation_cannot_change_prepared_target(self):
        mutable = list(FIRST)
        route = prepare_joint_route(START, [mutable], raw_unit="rad")
        mutable[0] = 1.0
        self.assertEqual(route.targets[0].joint_rad, FIRST)
        rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1),
                       feedback(FIRST, 100.2)])
        result = rig.run(route)
        self.assertEqual(result.status, "completed")
        self.assertEqual(rig.published, [FIRST])

        later = list(SECOND)
        route = prepare_joint_route(START, [FIRST, later], raw_unit="rad")
        rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1),
                       feedback(FIRST, 100.2), feedback(SECOND, 100.3),
                       feedback(SECOND, 100.4)])
        original_publish = rig.publish
        def mutate_after_first_send(target):
            original_publish(target)
            if len(rig.published) == 1:
                later[1] = 1.5
        result = execute_joint_route(
            route, parameters=PARAMS, preflight_check=rig.preflight,
            read_feedback=rig.read, publish_target=mutate_after_first_send,
            cancel_requested=lambda: False, stop_motion=rig.stop,
            monotonic_s=lambda: rig.now, sleep_s=rig.sleep)
        self.assertEqual(result.status, "completed")
        self.assertEqual(rig.published, [FIRST, SECOND])

    def test_start_mismatch_preflight_rejection_and_precancel_send_nothing(self):
        route = self.route(FIRST)
        rig = FakeRig([feedback((0.2,) + START[1:])])
        result = rig.run(route)
        self.assertEqual((result.status, result.attempted, result.published,
                          result.reached), ("fault", 0, 0, 0))
        self.assertIn("expected_start", result.reason)
        self.assertEqual(rig.stops, [])
        self.assertEqual(rig.published, [])

        rig = FakeRig([feedback()])
        result = rig.run(route, cancel=lambda: True)
        self.assertEqual(result.status, "cancelled")
        self.assertEqual(rig.events, [])
        self.assertEqual(rig.stops, [])

        rig = FakeRig([feedback()])
        def reject():
            raise ValueError("installation and route not authorized")
        with self.assertRaisesRegex(ValueError, "not authorized"):
            rig.run(route, preflight=reject)
        self.assertEqual(rig.events, [])

        with self.assertRaisesRegex(ValueError, "waypoint_timeout_s"):
            RouteParameters(0.01, 0.01, 0.0, 0.01, 0.05, 0.002)
        with self.assertRaisesRegex(TypeError, "stop_motion"):
            execute_joint_route(
                route, parameters=PARAMS, preflight_check=rig.preflight,
                read_feedback=rig.read, publish_target=rig.publish,
                cancel_requested=lambda: False, stop_motion=None,
                monotonic_s=lambda: 0.0, sleep_s=rig.sleep)
        self.assertEqual(rig.events, [])

    def test_feedback_faults_after_publish_stop_without_next_target(self):
        cases = (
            (feedback(FIRST, 100.1, status_error=9), "controller error"),
            (feedback(FIRST, 100.1, driver_fault=True), "driver"),
            (feedback(FIRST, 100.1, joints_missing=True), "joint feedback missing"),
            (feedback(FIRST, 98.0, observed=100.1), "joint feedback stale"),
        )
        for bad, reason in cases:
            with self.subTest(reason=reason):
                rig = FakeRig([feedback(START, 100.0), bad])
                result = rig.run(self.route())
                self.assertEqual((result.status, result.attempted, result.published,
                                  result.reached), ("fault", 1, 1, 0))
                self.assertIn(reason, result.reason)
                self.assertEqual(rig.published, [FIRST])
                self.assertEqual(len(rig.stops), 1)
                self.assertTrue(result.stop_request_attempted)
                self.assertTrue(result.stop_requested)
                self.assertFalse(result.stop_feedback_confirmed)

    def test_repeated_cache_and_misleading_idle_status_timeout(self):
        for sample in (feedback(FIRST, 100.0),
                       feedback(START, 100.1, motion_status=0)):
            with self.subTest(stamp=sample.joints_quality.timestamp_s,
                              joints=sample.joint_rad):
                rig = FakeRig([feedback(START, 100.0), sample])
                result = rig.run(self.route())
                self.assertEqual(result.status, "fault")
                self.assertIn("timeout", result.reason)
                self.assertEqual((result.attempted, result.published, result.reached),
                                 (1, 1, 0))
                self.assertEqual(rig.published, [FIRST])
                self.assertEqual(len(rig.stops), 1)

    def test_post_publish_cancel_publish_error_and_stop_error_preserve_cause(self):
        route = self.route()
        rig = FakeRig([feedback(START, 100.0)])
        result = rig.run(route, cancel=lambda: bool(rig.published))
        self.assertEqual((result.status, result.attempted, result.published,
                          result.reached), ("cancelled", 1, 1, 0))
        self.assertEqual(len(rig.stops), 1)

        rig = FakeRig([feedback(START, 100.0)])
        rig.fail_publish = True
        result = rig.run(route)
        self.assertEqual((result.status, result.attempted, result.published),
                         ("fault", 1, 0))
        self.assertIn("partial-send uncertainty", result.reason)
        self.assertEqual(rig.published, [FIRST])
        self.assertEqual(len(rig.stops), 1)

        rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1),
                       feedback(FIRST, 100.2)])
        original_publish = rig.publish
        def fail_second(target):
            original_publish(target)
            if len(rig.published) == 2:
                raise OSError("second target may have been partially sent")
        result = execute_joint_route(
            route, parameters=PARAMS, preflight_check=rig.preflight,
            read_feedback=rig.read, publish_target=fail_second,
            cancel_requested=lambda: False, stop_motion=rig.stop,
            monotonic_s=lambda: rig.now, sleep_s=rig.sleep)
        self.assertEqual((result.status, result.attempted, result.published,
                          result.reached, result.last_target_index),
                         ("fault", 2, 1, 1, 2))
        self.assertIn("partially sent", result.reason)
        self.assertEqual(len(rig.stops), 1)

        rig = FakeRig([feedback(START, 100.0), feedback(START, 100.1)])
        rig.fail_stop = True
        result = rig.run(route)
        self.assertEqual(result.status, "fault")
        self.assertIn("feedback timeout", result.reason)
        self.assertTrue(result.stop_request_attempted)
        self.assertFalse(result.stop_requested)
        self.assertFalse(result.stop_feedback_confirmed)
        self.assertIn("synthetic stop request failed", result.stop_detail)
        self.assertEqual(len(rig.stops), 1)

    def test_keyboard_interrupt_during_feedback_cancels_and_requests_stop(self):
        rig = FakeRig([feedback(START, 100.0)])
        original_read = rig.read
        def interrupted_read():
            if rig.published:
                raise KeyboardInterrupt
            return original_read()
        rig.read = interrupted_read
        result = rig.run(self.route())
        self.assertEqual((result.status, result.attempted, result.published,
                          result.reached), ("cancelled", 1, 1, 0))
        self.assertIn("KeyboardInterrupt", result.reason)
        self.assertEqual(len(rig.stops), 1)
        self.assertFalse(result.stop_feedback_confirmed)

    def test_route_identity_conflict_and_old_timestamp_is_not_revalidated_now(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = []
            for index, robot_id in enumerate(("robot-A", "robot-B")):
                snap = build_snapshot(
                    robot_id=robot_id, observed_at_s=100.0,
                    time_basis="synthetic_seconds",
                    status=None, joints={"timestamp_s": 99.9,
                                         "joint_rad": list(START)},
                    flange=None, drivers=None,
                    max_status_age_s=1, max_joints_age_s=1,
                    max_flange_age_s=1, max_driver_age_s=1)
                path = Path(directory) / f"{index}.json"
                save_waypoint(capture_waypoint(f"old-{index}", snap,
                                               source="historical synthetic"), path)
                paths.append(path)
            with self.assertRaisesRegex(ValueError, "robot_id"):
                prepare_joint_route(paths[0], [paths[1]])
            route = prepare_joint_route(paths[0], [FIRST], raw_unit="rad")
            self.assertEqual(route.expected_start.name, "old-0")
            self.assertEqual(route.expected_start.joint_rad, START)

    def test_cli_help_imports_no_sdk_can_or_experiments(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        script = ("import sys; import nero_runtime.cli, nero_runtime.motion; "
                  "assert 'pyAgxArm' not in sys.modules; "
                  "assert 'can' not in sys.modules; "
                  "assert 'experiments' not in sys.modules")
        imported = subprocess.run(
            [sys.executable, "-B", "-S", "-c", script], cwd=root,
            env=environment, capture_output=True, text=True)
        self.assertEqual(imported.returncode, 0, imported.stderr)
        helped = subprocess.run(
            [sys.executable, "-B", "-S", "-m", "nero_runtime.cli", "route", "show", "--help"],
            cwd=root, env=environment, capture_output=True, text=True)
        self.assertEqual(helped.returncode, 0, helped.stderr)
        self.assertIn("expected_start", helped.stdout)


if __name__ == "__main__":
    unittest.main()
