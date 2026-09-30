"""F7：真实 F1 文件/Profile/工作流/F2，外部仅 FakeRig 和合成 F5。"""

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from nero_runtime.acquisition import collect_snapshot
from nero_runtime.environment import load_environment_profile
from nero_runtime.motion import prepare_joint_route
from nero_runtime.recording import RecordingParameters
from nero_runtime.teaching import run_guided_recording
from nero_runtime.workflows import (
    EnvironmentIdentity, execute_park, execute_return_ready, execute_start,
    execute_teach_return, prepare_park, prepare_return_ready, prepare_start,
    prepare_teach_return,
)
from test_drag import FakeClock, synthetic_active
from test_environment import EnvironmentFixture
from test_motion import FakeRig, FIRST, PARAMS, SECOND, START, feedback
from test_teaching import GuidedFakeNero


IDENTITY = EnvironmentIdentity("site-a", "v1", "left", "bare_flange")


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.fixture = EnvironmentFixture(self.temporary.name)
        self.fixture.add_point("READY", "ready", START)
        self.fixture.add_point("PARK", "park", FIRST)
        self.fixture.add_point("WORK", "waypoint", SECOND)
        self.fixture.add_route("park-ready", "PARK", "READY", "start")
        self.fixture.add_route("ready-park", "READY", "PARK", "park")
        self.fixture.add_route("work-ready", "WORK", "READY", "return_ready")

    def save(self):
        return self.fixture.save()

    def execute(self, method, plan, rig, *, cancel=None, preflight=None):
        return method(plan, parameters=PARAMS,
                      preflight_check=rig.preflight if preflight is None else preflight,
                      read_feedback=rig.read, publish_target=rig.publish,
                      cancel_requested=(lambda: False) if cancel is None else cancel,
                      stop_motion=rig.stop, monotonic_s=lambda: rig.now,
                      sleep_s=rig.sleep)

    def test_start_park_and_explicit_return_positive_f1_profile_f2(self):
        path = self.save()
        start = prepare_start(path, feedback(FIRST), IDENTITY,
                              match_tolerance_rad=0.01, route_id="park-ready")
        self.assertTrue(start.executable_under_profile)
        self.assertEqual((start.from_point, start.to_point, start.route_id),
                         ("PARK", "READY", "park-ready"))
        self.assertEqual(start.route.expected_start.joint_rad, FIRST)
        self.assertEqual(start.route.targets[-1].joint_rad, START)
        self.assertEqual(len(start.route_source_hashes), 2)
        rig = FakeRig([feedback(FIRST, 100.0), feedback(START, 100.1), feedback(START, 100.2)])
        result = self.execute(execute_start, start, rig)
        self.assertEqual((result.status, result.motion_result.status,
                          result.motion_result.reached), ("completed", "completed", 1))
        self.assertEqual(rig.published, [START])

        park = prepare_park(path, feedback(START), IDENTITY,
                            match_tolerance_rad=0.01, route_id="ready-park")
        rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1), feedback(FIRST, 100.2)])
        result = self.execute(execute_park, park, rig)
        self.assertEqual(result.status, "completed")
        self.assertEqual(rig.published, [FIRST])
        self.assertFalse(result.physical_support_confirmed or result.disable_permission)

        ret = prepare_return_ready(path, feedback(SECOND), IDENTITY,
                                   match_tolerance_rad=0.01, route_id="work-ready")
        rig = FakeRig([feedback(SECOND, 100.0), feedback(START, 100.1), feedback(START, 100.2)])
        result = self.execute(execute_return_ready, ret, rig)
        self.assertEqual(result.status, "completed")
        self.assertEqual(rig.published, [START])
        self.assertTrue(all(point.source.startswith(str(Path(self.temporary.name)))
                            for point in (ret.route.expected_start, *ret.route.targets)))

    def test_route_states_candidate_locked_retired_preview_only(self):
        for state in ("candidate", "locked", "retired"):
            with self.subTest(state=state), tempfile.TemporaryDirectory() as directory:
                f = EnvironmentFixture(directory)
                f.add_point("READY", "ready", START)
                f.add_point("PARK", "park", FIRST)
                f.add_route("park-ready", "PARK", "READY", "start", state=state)
                plan = prepare_start(f.save(), feedback(FIRST), IDENTITY,
                                     match_tolerance_rad=0.01, route_id="park-ready")
                self.assertIsNotNone(plan.route)
                self.assertFalse(plan.executable_under_profile)
                self.assertIn(f"route_{state}", plan.reasons)
                rig = FakeRig([feedback(FIRST)])
                self.assertEqual(self.execute(execute_start, plan, rig).status, "rejected")
                self.assertEqual(rig.events, [])

    def test_wrong_operation_direction_and_point_state_rejected(self):
        for kind in ("operation", "direction", "point"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                f = EnvironmentFixture(directory)
                f.add_point("READY", "ready", START)
                f.add_point("PARK", "park", FIRST)
                f.add_route("park-ready", "PARK", "READY", "start")
                if kind == "operation":
                    f.routes[0]["allowed_operations"] = ["park"]
                elif kind == "direction":
                    f.routes[0]["directional"] = False
                else:
                    f.points[1]["state"] = "candidate"
                plan = prepare_start(f.save(), feedback(FIRST), IDENTITY,
                                     match_tolerance_rad=0.01, route_id="park-ready")
                self.assertFalse(plan.executable_under_profile)

    def test_robot_mount_tool_and_unknown_identity_rejected(self):
        path = self.save()
        cases = ((replace(feedback(FIRST), robot_id="other"), IDENTITY, "profile_mismatch"),
                 (feedback(FIRST), replace(IDENTITY, mount="right"), "profile_mismatch"),
                 (feedback(FIRST), replace(IDENTITY, tool_id="dh116"), "profile_mismatch"),
                 (feedback(FIRST), replace(IDENTITY, tool_id=None), "tool_unknown"),
                 (replace(feedback(FIRST), robot_id=None), IDENTITY, "identity_unknown"),
                 (feedback(FIRST), replace(IDENTITY, environment_version="v2"), "profile_mismatch"))
        for snapshot, identity, reason in cases:
            with self.subTest(reason=reason):
                plan = prepare_start(path, snapshot, identity, match_tolerance_rad=0.01)
                self.assertEqual(plan.reasons, (reason,))
                self.assertFalse(plan.executable_under_profile)

    def test_wrong_live_start_is_zero_publish(self):
        path = self.save()
        plan = prepare_start(path, feedback(FIRST), IDENTITY,
                             match_tolerance_rad=0.01, route_id="park-ready")
        rig = FakeRig([feedback(SECOND, 100.0)])
        result = self.execute(execute_start, plan, rig)
        self.assertEqual(result.status, "fault")
        self.assertEqual(result.motion_result.attempted, 0)
        self.assertEqual(rig.published, [])

    def test_already_ready_start_still_runs_preflight_and_live_read(self):
        path = self.save()
        plan = prepare_start(path, feedback(START), IDENTITY, match_tolerance_rad=0.01)
        self.assertTrue(plan.executable_under_profile)
        self.assertIsNone(plan.route)
        rig = FakeRig([feedback(START)])
        result = self.execute(execute_start, plan, rig)
        self.assertEqual((result.status, result.no_motion), ("completed", True))
        self.assertEqual(rig.events[0], "preflight")
        self.assertEqual(rig.published, [])
        rig = FakeRig([feedback(START)])
        self.assertEqual(self.execute(execute_start, plan, rig,
                                      cancel=lambda: True).status, "cancelled")
        self.assertEqual(rig.events, [])
        rig = FakeRig([feedback(SECOND)])
        self.assertEqual(self.execute(execute_start, plan, rig).status, "rejected")
        rig = FakeRig([feedback(START)])
        with self.assertRaisesRegex(ValueError, "site permission"):
            self.execute(execute_start, plan, rig,
                         preflight=lambda: (_ for _ in ()).throw(ValueError("site permission")))
        self.assertEqual(rig.events, [])

    def test_disabled_park_start_requires_enable_but_never_calls_it(self):
        path = self.save()
        snap = feedback(FIRST)
        drivers = list(snap.drivers)
        drivers[3] = replace(drivers[3], enabled=False)
        plan = prepare_start(path, replace(snap, drivers=tuple(drivers)), IDENTITY,
                             match_tolerance_rad=0.01, route_id="park-ready")
        self.assertTrue(plan.requires_enable)
        self.assertFalse(plan.executable_under_profile)
        self.assertIn("requires_enable_adapter", plan.reasons)
        rig = FakeRig([snap])
        self.assertEqual(self.execute(execute_start, plan, rig).status, "rejected")
        self.assertEqual(rig.events, [])

    def test_negative_x_is_not_a_profile_or_workflow_rejection(self):
        path = self.save()
        snap = feedback(FIRST)
        self.assertLess(snap.flange_pose_m_rad[0], 0)
        self.assertTrue(prepare_start(path, snap, IDENTITY,
                                      match_tolerance_rad=0.01).executable_under_profile)

    def test_return_no_route_or_wrong_destination_never_calls_motion(self):
        path = self.save()
        plan = prepare_return_ready(path, feedback(SECOND), IDENTITY,
                                    match_tolerance_rad=0.01)
        self.assertEqual(plan.reasons, ("no_applicable_route",))
        rig = FakeRig([feedback(SECOND)])
        self.assertEqual(self.execute(execute_return_ready, plan, rig).status,
                         "no_applicable_route")
        self.assertEqual(rig.events, [])
        plan = prepare_return_ready(path, feedback(SECOND), IDENTITY,
                                    match_tolerance_rad=0.01, route_id="ready-park")
        self.assertEqual(plan.reasons, ("no_applicable_route",))

    def test_explicit_joint_route_candidate_must_match_registered_asset(self):
        path = self.save()
        candidate = prepare_joint_route(self.fixture.paths["WORK"],
                                        [self.fixture.paths["READY"]])
        plan = prepare_return_ready(path, feedback(SECOND), IDENTITY,
                                    match_tolerance_rad=0.01,
                                    route_id="work-ready", route_candidate=candidate)
        self.assertTrue(plan.executable_under_profile)
        inferred = prepare_return_ready(path, feedback(SECOND), IDENTITY,
                                        match_tolerance_rad=0.01,
                                        route_candidate=candidate)
        self.assertEqual(inferred.route_id, "work-ready")
        self.assertTrue(inferred.executable_under_profile)
        wrong = prepare_joint_route(SECOND, [FIRST], raw_unit="rad")
        plan = prepare_return_ready(path, feedback(SECOND), IDENTITY,
                                    match_tolerance_rad=0.01,
                                    route_id="work-ready", route_candidate=wrong)
        self.assertIn("route_candidate_mismatch", plan.reasons)
        self.assertFalse(plan.executable_under_profile)

    def test_profile_or_plan_tamper_rejected_before_preflight(self):
        path = self.save()
        plan = prepare_start(path, feedback(FIRST), IDENTITY,
                             match_tolerance_rad=0.01, route_id="park-ready")
        rig = FakeRig([feedback(FIRST)])
        with self.assertRaisesRegex(ValueError, "differs"):
            self.execute(execute_start, replace(plan, route_id="ready-park"), rig)
        self.assertEqual(rig.events, [])
        path.write_text(path.read_text() + " ")
        with self.assertRaisesRegex(ValueError, "changed"):
            self.execute(execute_start, plan, rig)
        self.assertEqual(rig.events, [])

    def test_cancel_fault_and_stop_failure_propagate_from_f2(self):
        path = self.save()
        plan = prepare_park(path, feedback(START), IDENTITY,
                            match_tolerance_rad=0.01, route_id="ready-park")
        rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1)])
        result = self.execute(execute_park, plan, rig,
                              cancel=lambda: len(rig.published) >= 1)
        self.assertEqual(result.status, "cancelled")
        self.assertEqual(result.motion_result.attempted, 1)
        self.assertEqual(len(rig.stops), 1)
        rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1, status_error=9)])
        rig.fail_stop = True
        result = self.execute(execute_park, plan, rig)
        self.assertEqual(result.status, "fault")
        self.assertIn("controller error", result.motion_result.reason)
        self.assertIn("stop handling failed", result.motion_result.stop_detail)
        self.assertFalse(result.motion_result.stop_feedback_confirmed)

    def _actual_f5_result(self):
        clock = FakeClock()
        robot = GuidedFakeNero(clock)
        root = Path(self.temporary.name)
        def read():
            return collect_snapshot(robot, robot_id="guided-fake", clock_s=clock.wall,
                                    time_basis="unix_epoch_s", max_status_age_s=1,
                                    max_joints_age_s=1, max_flange_age_s=1,
                                    max_driver_age_s=1)
        result = run_guided_recording(
            recording_id="guided-f7", output_directory=root,
            parameters=RecordingParameters(0.25, 10), read_snapshot=read,
            request_enter_drag=robot.set_leader_mode,
            request_exit_drag=robot.set_normal_mode,
            drag_active_predicate=synthetic_active,
            monotonic_s=clock.monotonic, sleep_s=clock.sleep,
            cancel_requested=lambda: False, finish_requested=lambda: False,
            wall_clock_s=clock.wall, enter_timeout_s=0.3,
            exit_timeout_s=0.3, poll_interval_s=0.05)
        self.assertEqual(result.workflow_outcome, "completed")
        return result, robot

    def test_f5_completed_without_return_route_is_not_teach_v1(self):
        guided, robot = self._actual_f5_result()
        f = EnvironmentFixture(Path(self.temporary.name) / "guided_profile", robot_id="guided-fake")
        f.add_point("READY", "ready", START,
                    snapshot=replace(feedback(START), robot_id="guided-fake"))
        f.add_point("WORK", "waypoint", guided.final_joint_rad,
                    snapshot=guided.final_snapshot)
        path = f.save()
        prepared = prepare_teach_return(guided, path, IDENTITY,
                                        match_tolerance_rad=0.01)
        self.assertEqual(prepared.reason, "no_applicable_route")
        rig = FakeRig([replace(feedback(guided.final_joint_rad), robot_id="guided-fake")])
        result = execute_teach_return(prepared, parameters=PARAMS,
                                      preflight_check=rig.preflight,
                                      read_feedback=rig.read,
                                      publish_target=rig.publish,
                                      cancel_requested=lambda: False,
                                      stop_motion=rig.stop,
                                      monotonic_s=lambda: rig.now, sleep_s=rig.sleep)
        self.assertFalse(result.teach_v1_completed)
        self.assertEqual(rig.events, [])
        self.assertEqual(robot.control_calls["move_j"], 0)

    def test_actual_f5_plus_valid_return_and_f2_completes_offline_teach_v1(self):
        guided, robot = self._actual_f5_result()
        f = EnvironmentFixture(Path(self.temporary.name) / "guided_profile", robot_id="guided-fake")
        f.add_point("READY", "ready", START,
                    snapshot=replace(feedback(START), robot_id="guided-fake"))
        f.add_point("WORK", "waypoint", guided.final_joint_rad,
                    snapshot=guided.final_snapshot)
        f.add_route("work-ready", "WORK", "READY", "return_ready")
        path = f.save()
        prepared = prepare_teach_return(guided, path, IDENTITY,
                                        match_tolerance_rad=0.01,
                                        route_id="work-ready")
        self.assertIsNone(prepared.reason)
        rig = FakeRig([replace(feedback(guided.final_joint_rad, 100.0), robot_id="guided-fake"),
                       replace(feedback(START, 100.1), robot_id="guided-fake"),
                       replace(feedback(START, 100.2), robot_id="guided-fake")])
        result = execute_teach_return(prepared, parameters=PARAMS,
                                      preflight_check=rig.preflight,
                                      read_feedback=rig.read,
                                      publish_target=rig.publish,
                                      cancel_requested=lambda: False,
                                      stop_motion=rig.stop,
                                      monotonic_s=lambda: rig.now, sleep_s=rig.sleep)
        self.assertTrue(result.teach_v1_completed)
        self.assertEqual(result.return_result.status, "completed")
        self.assertEqual(rig.published, [START])
        self.assertEqual(robot.control_calls["move_j"], 0)

        rig = FakeRig([replace(feedback(guided.final_joint_rad), robot_id="guided-fake")])
        forged = replace(prepared, guided_result=replace(guided, workflow_outcome="fault"))
        rejected = execute_teach_return(
            forged, parameters=PARAMS, preflight_check=rig.preflight,
            read_feedback=rig.read, publish_target=rig.publish,
            cancel_requested=lambda: False, stop_motion=rig.stop,
            monotonic_s=lambda: rig.now, sleep_s=rig.sleep)
        self.assertFalse(rejected.teach_v1_completed)
        self.assertEqual(rig.events, [])


if __name__ == "__main__":
    unittest.main()
