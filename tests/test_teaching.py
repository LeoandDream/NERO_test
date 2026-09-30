"""F5：F1→F4→F3→F4→F1 真实合成链，绝不连接 SDK/CAN。"""

from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from nero_runtime.acquisition import collect_snapshot
from nero_runtime.recording import RecordingParameters, load_recording_summary
from nero_runtime.teaching import run_guided_recording
from test_drag import FakeClock, FakeNero, synthetic_active


class GuidedFakeNero(FakeNero):
    def __init__(self, clock):
        super().__init__(clock)
        self.recording_reads = 0
        self.lose_drag_at = None
        self.controller_fault_at = None
        self.driver_fault_at = None
        self.final_joint_missing = False
        self.normal_after_exit_reads = 0

    def get_arm_status(self):
        if self.mode == "drag" and self.pending is None:
            self.recording_reads += 1
            if self.lose_drag_at == self.recording_reads:
                self.mode = "normal"
            if self.controller_fault_at == self.recording_reads:
                self.fault_code = 9
            if self.driver_fault_at == self.recording_reads:
                self.disabled_joint = 4
        result = super().get_arm_status()
        if self.mode == "normal" and self.calls.count("set_normal_mode"):
            self.normal_after_exit_reads += 1
        return result

    def get_joint_angles(self):
        if self.final_joint_missing and self.normal_after_exit_reads >= 2:
            self.calls.append("get_joint_angles")
            return None
        return super().get_joint_angles()


class GuidedRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.clock = FakeClock()
        self.robot = GuidedFakeNero(self.clock)

    def read(self):
        return collect_snapshot(
            self.robot, robot_id="guided-fake", clock_s=self.clock.wall,
            time_basis="unix_epoch_s", max_status_age_s=1,
            max_joints_age_s=1, max_flange_age_s=1, max_driver_age_s=1)

    def run_workflow(self, **changes):
        kwargs = dict(
            recording_id="guided-1", output_directory=self.directory,
            parameters=RecordingParameters(0.25, 10),
            read_snapshot=self.read,
            request_enter_drag=self.robot.set_leader_mode,
            request_exit_drag=self.robot.set_normal_mode,
            drag_active_predicate=synthetic_active,
            monotonic_s=self.clock.monotonic, sleep_s=self.clock.sleep,
            cancel_requested=lambda: False,
            finish_requested=lambda: False, wall_clock_s=self.clock.wall,
            enter_timeout_s=0.3, exit_timeout_s=0.3, poll_interval_s=0.05,
        )
        kwargs.update(changes)
        return run_guided_recording(**kwargs)

    def raw_rows(self):
        return [json.loads(line) for line in
                (self.directory / "guided-1.jsonl").read_text().splitlines()]

    def assert_no_forbidden_controls(self):
        self.assertTrue(all(count == 0 for count in self.robot.control_calls.values()))

    def test_positive_f1_f4_f3_f4_f1_chain(self):
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "completed")
        self.assertEqual(result.phases,
                         ("before_enter", "drag_entered", "recording", "exiting_drag", "finished"))
        self.assertTrue(result.enter_result.enter_confirmed)
        self.assertTrue(result.exit_result.exit_confirmed)
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 1)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        rows = self.raw_rows()
        self.assertGreaterEqual(len(rows), 2)
        self.assertEqual([row["sample_index"] for row in rows], list(range(len(rows))))
        self.assertEqual([row["snapshot"]["flange_pose_m_rad"][0] for row in rows],
                         [-0.18] * len(rows))
        self.assertEqual(result.recording_summary["sample_count"], len(rows))
        self.assertEqual(result.recording_summary["stop_reason"], "duration_complete")
        self.assertEqual(load_recording_summary(result.summary_path), result.recording_summary)
        self.assertEqual(result.final_state, "known")
        self.assertIsNotNone(result.final_joint_rad)
        self.assertEqual(result.final_flange_pose[0], -0.18)
        self.assertGreater(result.final_snapshot.status_quality.timestamp_s,
                           result.exit_result.observations[-1].snapshot.status_quality.timestamp_s)
        self.assertFalse(result.return_attempted or result.motion_attempted)
        self.assert_no_forbidden_controls()
        self.assertNotIn("pyAgxArm", sys.modules)
        self.assertNotIn("can", sys.modules)
        self.assertNotIn("nero_runtime.motion", sys.modules)

    def test_invalid_parameters_reject_before_drag_or_file(self):
        with self.assertRaises(ValueError):
            self.run_workflow(enter_timeout_s=0)
        with self.assertRaises(ValueError):
            self.run_workflow(parameters=RecordingParameters(0, 10))
        rejected = self.run_workflow(recording_id="../bad")
        self.assertEqual(rejected.workflow_outcome, "rejected")
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 0)
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_recording_id_conflict_rejects_before_drag(self):
        raw = self.directory / "guided-1.jsonl"
        raw.write_text("original", encoding="utf-8")
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "rejected")
        self.assertEqual(raw.read_text(), "original")
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)
        self.assertFalse((self.directory / "guided-1.summary.json").exists())

    def test_drag_preflight_rejected_creates_no_recording(self):
        self.robot.disabled_joint = 4
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "rejected")
        self.assertFalse(result.enter_result.enter_request_attempted)
        self.assertIsNone(result.recording_summary)
        self.assertIsNone(result.exit_result)
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 0)
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_enter_unconfirmed_uses_f4_single_rollback(self):
        self.robot.enter_after = None
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "fault")
        self.assertFalse(result.enter_result.enter_confirmed)
        self.assertEqual(self.robot.calls.count("set_leader_mode"), 1)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertIsNone(result.recording_summary)
        self.assertIsNone(result.exit_result)
        self.assertEqual(list(self.directory.iterdir()), [])

    def test_user_finish_is_completed_core_only(self):
        result = self.run_workflow(finish_requested=lambda: self.robot.recording_reads >= 2)
        self.assertEqual(result.workflow_outcome, "completed")
        self.assertEqual(result.recording_summary["stop_reason"], "user_finish")
        self.assertTrue(result.exit_result.exit_confirmed)
        self.assertFalse(result.return_attempted)

    def test_cancel_during_recording_keeps_raw_and_exits_once(self):
        result = self.run_workflow(cancel_requested=lambda:
                                   self.robot.mode == "drag" and self.robot.recording_reads >= 2)
        self.assertEqual(result.workflow_outcome, "cancelled")
        self.assertEqual(result.recording_summary["stop_reason"], "cancelled")
        self.assertEqual(len(self.raw_rows()), 2)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertTrue(result.exit_result.exit_confirmed)
        self.assert_no_forbidden_controls()

    def test_keyboard_interrupt_during_recording_keeps_raw_and_exits_once(self):
        normal_read = self.read
        def interrupted_read():
            if self.robot.mode == "drag" and self.robot.recording_reads >= 2:
                raise KeyboardInterrupt
            return normal_read()
        result = self.run_workflow(read_snapshot=interrupted_read)
        self.assertEqual(result.workflow_outcome, "cancelled")
        self.assertEqual(result.recording_summary["stop_reason"], "cancelled")
        self.assertGreaterEqual(len(self.raw_rows()), 2)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)

    def test_recording_read_fault_keeps_raw_and_exits_once(self):
        normal_read = self.read
        def failed_read():
            if self.robot.mode == "drag" and self.robot.recording_reads >= 2:
                raise RuntimeError("synthetic recording source failed")
            return normal_read()
        result = self.run_workflow(read_snapshot=failed_read)
        self.assertEqual(result.workflow_outcome, "fault")
        self.assertEqual(result.recording_summary["stop_reason"], "fault")
        self.assertIn("synthetic recording source failed", result.reason)
        self.assertGreaterEqual(len(self.raw_rows()), 2)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)

    def test_lost_drag_sample_is_preserved_then_faults(self):
        self.robot.lose_drag_at = 2
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "fault")
        self.assertEqual(result.recording_summary["stop_reason"], "fault")
        rows = self.raw_rows()
        self.assertEqual(rows[-1]["snapshot"]["control_mode"], 1)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assert_no_forbidden_controls()

    def test_exit_request_failure_is_unconfirmed(self):
        self.robot.exit_error = OSError("synthetic exit failure")
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "unconfirmed_exit")
        self.assertTrue(result.exit_result.exit_request_attempted)
        self.assertFalse(result.exit_result.exit_requested or result.exit_result.exit_confirmed)
        self.assertIn("synthetic exit failure", result.reason)
        self.assertIsNone(result.final_snapshot)
        self.assertEqual(len(self.raw_rows()), result.recording_summary["sample_count"])

    def test_exit_feedback_unconfirmed_never_completes(self):
        self.robot.exit_after = None
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "unconfirmed_exit")
        self.assertTrue(result.exit_result.exit_requested)
        self.assertFalse(result.exit_result.exit_confirmed)
        self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
        self.assertIsNone(result.final_snapshot)

    def test_final_snapshot_missing_does_not_fake_endpoint(self):
        self.robot.final_joint_missing = True
        result = self.run_workflow()
        self.assertEqual(result.workflow_outcome, "completed")
        self.assertEqual(result.final_state, "unknown")
        self.assertIsNotNone(result.final_snapshot)
        self.assertIsNone(result.final_joint_rad)
        self.assertIsNone(result.final_flange_pose)
        self.assertEqual(result.recording_summary["stop_reason"], "duration_complete")

    def test_controller_and_driver_fault_samples_preserved(self):
        for name in ("controller_fault_at", "driver_fault_at"):
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temp:
                self.clock = FakeClock()
                self.robot = GuidedFakeNero(self.clock)
                self.directory = Path(temp)
                setattr(self.robot, name, 2)
                result = self.run_workflow()
                self.assertEqual(result.workflow_outcome, "fault")
                self.assertEqual(result.recording_summary["stop_reason"], "fault")
                rows = self.raw_rows()
                self.assertGreaterEqual(len(rows), 2)
                if name == "controller_fault_at":
                    self.assertEqual(rows[-1]["snapshot"]["error_code"], 9)
                else:
                    self.assertFalse(rows[-1]["snapshot"]["drivers"][3]["enabled"])
                self.assertEqual(self.robot.calls.count("set_normal_mode"), 1)
                self.assert_no_forbidden_controls()

    def test_final_stale_feedback_keeps_unknown_endpoint(self):
        normal_read = self.read
        def stale_final():
            result = normal_read()
            if self.robot.normal_after_exit_reads >= 2:
                snapshot = replace(result.snapshot,
                                   joints_quality=replace(result.snapshot.joints_quality,
                                                          state="stale"))
                return replace(result, snapshot=snapshot)
            return result
        result = self.run_workflow(read_snapshot=stale_final)
        self.assertEqual(result.workflow_outcome, "completed")
        self.assertEqual(result.final_state, "unknown")
        self.assertIsNone(result.final_joint_rad)
        self.assertEqual(len(self.raw_rows()), result.recording_summary["sample_count"])

    def test_import_isolation(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        completed = subprocess.run(
            [sys.executable, "-B", "-S", "-c",
             "import sys, nero_runtime.teaching; "
             "assert 'pyAgxArm' not in sys.modules; "
             "assert 'can' not in sys.modules; "
             "assert 'experiments' not in sys.modules; "
             "assert 'nero_runtime.device' not in sys.modules; "
             "assert 'nero_runtime.motion' not in sys.modules; "
             "assert 'nero_runtime.cli' not in sys.modules"],
            cwd=root, env=environment, capture_output=True, text=True)
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
