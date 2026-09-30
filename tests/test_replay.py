"""F6：只用合成反馈、临时 F3 原件和 F2 注入执行边界。"""

from dataclasses import replace
from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from nero_runtime.acquisition import collect_snapshot
from nero_runtime.cli import main
from nero_runtime.motion import StopEvidence
from nero_runtime.recording import RecordingParameters, record_trajectory
from nero_runtime.replay import execute_replay, prepare_replay, replay_route
from test_motion import FakeRig, PARAMS, feedback
from test_recording import FakeSource, FakeTime


START = (0.0,) * 7
FIRST = (0.1,) + (0.0,) * 6
SECOND = (0.1, -0.2) + (0.0,) * 5


class ReplaySource(FakeSource):
    def get_joint_angles(self):
        if self.sample.get("joints_missing"):
            return None
        return SimpleNamespace(
            timestamp=self.sample.get("joint_timestamp", self.sample.get("timestamp", 99.9)),
            msg=list(self.sample.get("joints", START)))


class ReplayTests(unittest.TestCase):
    def make_recording(self, directory, samples=None, *, duration=0.35):
        if samples is None:
            samples = [{"timestamp": 99.9, "joints": START},
                       {"timestamp": 100.0, "joints": FIRST},
                       {"timestamp": 100.1, "joints": SECOND},
                       {"timestamp": 100.2, "joints": SECOND}]
        clock = FakeTime()
        source = ReplaySource(samples)
        summary = record_trajectory(
            recording_id="replay-1", output_directory=directory,
            parameters=RecordingParameters(duration, 10),
            read_snapshot=lambda: collect_snapshot(
                source, robot_id="synthetic-nero", clock_s=clock.wall,
                time_basis="unix_epoch_s", max_status_age_s=1,
                max_joints_age_s=1, max_flange_age_s=1, max_driver_age_s=1),
            monotonic_s=clock.monotonic, sleep_s=clock.sleep,
            cancel_requested=lambda: False, finish_requested=lambda: False,
            wall_clock_s=clock.wall)
        return Path(directory) / "replay-1.summary.json", summary

    def change(self, path, edit):
        raw_path = path.with_name("replay-1.jsonl")
        rows = [json.loads(line) for line in raw_path.read_text().splitlines()]
        summary = json.loads(path.read_text())
        edit(rows, summary)
        raw_path.write_text("".join(json.dumps(row, separators=(",", ":")) + "\n"
                                    for row in rows))
        summary["raw_sha256"] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        for name, field in (("joint", "joints_quality"),
                            ("flange", "flange_quality"), ("status", "status_quality")):
            summary[f"{name}_quality_counts"] = {
                state: sum(row["snapshot"][field]["state"] == state for row in rows)
                for state in ("valid", "stale", "missing", "invalid")}
        summary["controller_fault_sample_count"] = sum(
            row["snapshot"]["error_code"] not in (None, 0) for row in rows)
        path.write_text(json.dumps(summary))

    def run_plan(self, plan, rig, *, cancel=None, preflight=None):
        return execute_replay(
            plan, parameters=PARAMS,
            preflight_check=rig.preflight if preflight is None else preflight,
            read_feedback=rig.read, publish_target=rig.publish,
            cancel_requested=(lambda: False) if cancel is None else cancel,
            stop_motion=rig.stop, monotonic_s=lambda: rig.now, sleep_s=rig.sleep)

    def test_real_f1_f3_f6_f2_chain_preserves_order_and_waits_for_feedback(self):
        with tempfile.TemporaryDirectory() as directory:
            path, summary = self.make_recording(directory)
            plan = prepare_replay(path)
            self.assertEqual(summary["sample_count"], 4)
            self.assertEqual(plan.expected_start_joint_rad, START)
            self.assertEqual(plan.target_joint_rad, (FIRST, SECOND, SECOND))
            self.assertEqual(plan.source_sample_indices, (0, 1, 2, 3))
            self.assertEqual(replay_route(plan).targets[1].joint_rad, SECOND)
            self.assertFalse(plan.timing_preserved)
            rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1),
                           feedback(FIRST, 100.2), feedback(SECOND, 100.3),
                           feedback(SECOND, 100.4), feedback(SECOND, 100.5),
                           feedback(SECOND, 100.6)])
            result = self.run_plan(plan, rig)
            self.assertEqual((result.status, result.motion_result.status,
                              result.motion_result.attempted, result.motion_result.published,
                              result.motion_result.reached), ("completed", "completed", 3, 3, 3))
            self.assertEqual(rig.published, [FIRST, SECOND, SECOND])
            self.assertLess(rig.events.index(("read", 100.2)),
                            rig.events.index(("publish", SECOND)))
            self.assertEqual(rig.stops, [])
            self.assertFalse(result.timing_preserved)

    def test_cancelled_and_fault_recording_rejected(self):
        for reason in ("cancelled", "fault"):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as directory:
                path, _ = self.make_recording(directory)
                self.change(path, lambda rows, summary: summary.update(
                    stop_reason=reason, complete=False))
                with self.assertRaisesRegex(ValueError, "did not finish"):
                    prepare_replay(path)

    def test_one_sample_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory, duration=0.05)
            with self.assertRaisesRegex(ValueError, "at least two"):
                prepare_replay(path)

    def test_bad_joint_samples_never_dropped(self):
        for state in ("stale", "missing", "invalid"):
            with self.subTest(state=state), tempfile.TemporaryDirectory() as directory:
                path, _ = self.make_recording(directory)
                self.change(path, lambda rows, summary: rows[1]["snapshot"].update(
                    joints_quality={**rows[1]["snapshot"]["joints_quality"], "state": state},
                    joint_rad=None if state != "stale" else rows[1]["snapshot"]["joint_rad"]))
                with self.assertRaisesRegex(ValueError, "joint feedback"):
                    prepare_replay(path)

    def test_controller_and_driver_faults_rejected(self):
        for field, value in (("error_code", 7), ("enabled", False),
                             ("undervoltage", True), ("driver_error", True),
                             ("quality", {"state": "invalid"})):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as directory:
                path, _ = self.make_recording(directory)
                def edit(rows, summary):
                    target = rows[2]["snapshot"] if field == "error_code" else rows[2]["snapshot"]["drivers"][3]
                    target[field] = value
                self.change(path, edit)
                with self.assertRaises(ValueError):
                    prepare_replay(path)

    def test_hash_index_count_and_identity_tamper_rejected(self):
        for kind in ("hash", "index", "count", "identity", "time_basis"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                path, _ = self.make_recording(directory)
                if kind == "hash":
                    path.with_name("replay-1.jsonl").write_text("changed\n")
                elif kind == "index":
                    self.change(path, lambda rows, summary: rows[1].update(sample_index=8))
                elif kind == "count":
                    self.change(path, lambda rows, summary: summary.update(sample_count=8))
                elif kind == "identity":
                    self.change(path, lambda rows, summary: rows[1]["snapshot"].update(robot_id="other"))
                else:
                    self.change(path, lambda rows, summary: rows[1]["snapshot"].update(time_basis="other"))
                with self.assertRaises(ValueError):
                    prepare_replay(path)

    def test_negative_flange_x_is_not_replay_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            rows = [json.loads(line) for line in path.with_name("replay-1.jsonl").read_text().splitlines()]
            self.assertTrue(all(row["snapshot"]["flange_pose_m_rad"][0] < 0 for row in rows))
            self.assertEqual(prepare_replay(path).sample_count, 4)

    def test_wrong_live_start_rejects_without_publish(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            rig = FakeRig([feedback(FIRST, 100.0)])
            result = self.run_plan(prepare_replay(path), rig)
            self.assertEqual(result.status, "fault")
            self.assertEqual(result.motion_result.attempted, 0)
            self.assertEqual(rig.published, [])

    def test_cancel_stops_later_targets(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1)])
            result = self.run_plan(prepare_replay(path), rig,
                                   cancel=lambda: len(rig.published) >= 1)
            self.assertEqual(result.status, "cancelled")
            self.assertEqual(len(rig.published), 1)
            self.assertEqual(len(rig.stops), 1)

    def test_feedback_fault_and_stop_failure_preserve_evidence(self):
        for fail_stop in (False, True):
            with self.subTest(fail_stop=fail_stop), tempfile.TemporaryDirectory() as directory:
                path, _ = self.make_recording(directory)
                rig = FakeRig([feedback(START, 100.0), feedback(FIRST, 100.1, status_error=9)])
                rig.fail_stop = fail_stop
                result = self.run_plan(prepare_replay(path), rig)
                self.assertEqual(result.status, "fault")
                self.assertEqual(result.motion_result.attempted, 1)
                self.assertTrue(result.motion_result.stop_request_attempted)
                self.assertIn("controller error", result.motion_result.reason)
                self.assertEqual(result.motion_result.stop_requested, not fail_stop)
                if fail_stop:
                    self.assertIn("stop handling failed", result.motion_result.stop_detail)

    def test_publish_exception_retains_f2_counts(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            rig = FakeRig([feedback(START, 100.0)])
            rig.fail_publish = True
            result = self.run_plan(prepare_replay(path), rig)
            self.assertEqual((result.status, result.motion_result.attempted,
                              result.motion_result.published, result.motion_result.reached),
                             ("fault", 1, 0, 0))
            self.assertEqual(len(rig.stops), 1)

    def test_plan_and_source_tamper_rejected_before_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            plan = prepare_replay(path)
            rig = FakeRig([feedback(START, 100.0)])
            with self.assertRaisesRegex(ValueError, "differs"):
                self.run_plan(replace(plan, target_joint_rad=(SECOND,)), rig)
            self.assertEqual(rig.events, [])
            path.with_name("replay-1.jsonl").write_text("tampered\n")
            with self.assertRaises(ValueError):
                self.run_plan(plan, rig)
            self.assertEqual(rig.events, [])

    def test_complete_recording_does_not_bypass_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            rig = FakeRig([feedback(START, 100.0)])
            def reject():
                raise ValueError("site permission absent")
            with self.assertRaisesRegex(ValueError, "site permission absent"):
                self.run_plan(prepare_replay(path), rig, preflight=reject)
            self.assertEqual(rig.published, [])

    def test_cli_show_is_offline_and_rejection_is_nonzero(self):
        with tempfile.TemporaryDirectory() as directory:
            path, _ = self.make_recording(directory)
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                code = main(["replay", "show", str(path)])
            self.assertEqual(code, 0)
            self.assertIn("Replay candidate: yes", out.getvalue())
            self.assertIn("Timing preserved: no", out.getvalue())
            self.assertIn("Candidate does not grant motion permission.", out.getvalue())
            path.with_name("replay-1.jsonl").write_text("tampered\n")
            out, err = io.StringIO(), io.StringIO()
            with redirect_stdout(out), redirect_stderr(err):
                code = main(["replay", "show", str(path)])
            self.assertEqual(code, 1)
            self.assertIn("Replay candidate: no", err.getvalue())


if __name__ == "__main__":
    unittest.main()
