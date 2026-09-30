"""F3：合成反馈经真实 F1 采集与记录内核持久化，不触碰 SDK/CAN。"""

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from nero_runtime.acquisition import collect_snapshot
from nero_runtime.cli import main
from nero_runtime.device import connected_nero
from nero_runtime.recording import (
    RecordingParameters, format_recording_summary, list_recordings,
    load_recording_summary, record_trajectory,
)


class FakeTime:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def wall(self):
        return 100.0 + self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeSource:
    def __init__(self, samples=()):
        self.samples = list(samples)
        self.round = 0
        self.sample = {}
        self.calls = []

    def connect(self):
        self.calls.append("connect")

    def disconnect(self):
        self.calls.append("disconnect")

    def get_arm_status(self):
        self.sample = self.samples[min(self.round, len(self.samples) - 1)] if self.samples else {}
        self.round += 1
        self.calls.append("get_arm_status")
        if self.sample.get("status_missing"):
            return None
        return SimpleNamespace(
            timestamp=self.sample.get("timestamp", 99.9),
            msg=SimpleNamespace(ctrl_mode=2, arm_status=6, teach_status=2,
                                motion_status=0, err_code=self.sample.get("error_code", 0)))

    def get_joint_angles(self):
        self.calls.append("get_joint_angles")
        if self.sample.get("joints_missing"):
            return None
        return SimpleNamespace(
            timestamp=self.sample.get("joint_timestamp", self.sample.get("timestamp", 99.9)),
            msg=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])

    def get_flange_pose(self):
        self.calls.append("get_flange_pose")
        if self.sample.get("flange_error"):
            raise OSError("synthetic flange read error")
        return SimpleNamespace(timestamp=self.sample.get("timestamp", 99.9),
                               msg=[-0.18, 0.01, 0.7, 0, 0, 0])

    def get_driver_states(self, joint):
        self.calls.append(f"get_driver_states({joint})")
        return SimpleNamespace(
            timestamp=self.sample.get("timestamp", 99.9),
            msg=SimpleNamespace(vol=24.0,
                                foc_status=SimpleNamespace(
                                    driver_enable_status=not self.sample.get("disabled", False),
                                    voltage_too_low=False,
                                    driver_error_status=False)))

    def __getattr__(self, name):
        if name in {"move_j", "hold", "disable", "reset", "electronic_emergency_stop",
                    "enable", "move_l", "set_motion_mode"}:
            raise AssertionError(f"control method reached: {name}")
        raise AttributeError(name)


class RecordingTests(unittest.TestCase):
    def collect(self, source, clock):
        return collect_snapshot(
            source, robot_id="synthetic-nero", clock_s=clock.wall,
            time_basis="unix_epoch_s", max_status_age_s=1,
            max_joints_age_s=1, max_flange_age_s=1, max_driver_age_s=1)

    def run_recording(self, directory, samples, *, duration=0.35, rate=10,
                      cancel=None, finish=None, clock=None, read=None,
                      recording_id="record-1"):
        clock = FakeTime() if clock is None else clock
        source = FakeSource(samples)
        reader = (lambda: self.collect(source, clock)) if read is None else read
        summary = record_trajectory(
            recording_id=recording_id, output_directory=directory,
            parameters=RecordingParameters(duration, rate),
            read_snapshot=reader, monotonic_s=clock.monotonic,
            sleep_s=clock.sleep,
            cancel_requested=(lambda: False) if cancel is None else cancel,
            finish_requested=(lambda: False) if finish is None else finish,
            wall_clock_s=clock.wall)
        return summary, source, clock

    def test_real_acquisition_to_streamed_raw_and_verified_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            summary, source, clock = self.run_recording(
                directory, [{"timestamp": 99.9}, {"timestamp": 100.0},
                            {"timestamp": 100.1}, {"timestamp": 100.2}])
            raw = Path(directory) / "record-1.jsonl"
            summary_path = Path(directory) / "record-1.summary.json"
            rows = [json.loads(line) for line in raw.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(len(rows), 4)
            self.assertEqual([row["sample_index"] for row in rows], list(range(4)))
            self.assertEqual([round(row["elapsed_s"], 3) for row in rows],
                             [0.0, 0.1, 0.2, 0.3])
            self.assertEqual(rows[0]["snapshot"]["joint_rad"],
                             [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7])
            self.assertEqual(rows[0]["snapshot"]["flange_pose_m_rad"][0], -0.18)
            self.assertEqual(rows[0]["snapshot"]["arm_status"], 6)
            self.assertEqual(rows[0]["snapshot"]["joints_quality"]["timestamp_s"], 99.9)
            self.assertEqual(summary["raw_sha256"], hashlib.sha256(raw.read_bytes()).hexdigest())
            self.assertEqual(summary["sample_count"], 4)
            self.assertAlmostEqual(summary["actual_duration_s"], 0.35)
            self.assertAlmostEqual(summary["effective_rate_hz"], 4 / 0.35)
            self.assertEqual(summary["stop_reason"], "duration_complete")
            self.assertTrue(summary["complete"])
            self.assertEqual(summary["joint_quality_counts"]["valid"], 4)
            self.assertEqual(load_recording_summary(summary_path), summary)
            self.assertEqual(list_recordings(directory)[0]["summary"], summary)
            shown = format_recording_summary(summary, summary_path)
            self.assertIn("Joint feedback:", shown)
            self.assertIn("Raw file:", shown)
            self.assertIn("no Replay approval", shown)
            self.assertEqual(source.round, 4)
            self.assertEqual(clock.now, 0.35)

    def test_stale_fault_disabled_negative_x_and_read_error_are_stored(self):
        with tempfile.TemporaryDirectory() as directory:
            summary, _, _ = self.run_recording(
                directory,
                [{"timestamp": 99.9, "error_code": 9, "disabled": True},
                 {"timestamp": 100.0, "joint_timestamp": 97.0,
                  "flange_error": True}], duration=0.15)
            rows = [json.loads(line) for line in
                    (Path(directory) / "record-1.jsonl").read_text().splitlines()]
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["snapshot"]["error_code"], 9)
            self.assertFalse(rows[0]["snapshot"]["drivers"][3]["enabled"])
            self.assertEqual(rows[0]["snapshot"]["flange_pose_m_rad"][0], -0.18)
            self.assertEqual(rows[1]["snapshot"]["joints_quality"]["state"], "stale")
            self.assertEqual(rows[1]["snapshot"]["flange_quality"]["state"], "missing")
            self.assertIn("synthetic flange read error", rows[1]["read_errors"][0])
            self.assertEqual(summary["joint_quality_counts"]["stale"], 1)
            self.assertEqual(summary["flange_quality_counts"]["missing"], 1)
            self.assertEqual(summary["controller_fault_sample_count"], 1)
            self.assertTrue(summary["complete"])
            self.assertEqual(load_recording_summary(Path(directory) / "record-1.summary.json"), summary)

    def test_keyboard_interrupt_keeps_raw_and_writes_cancelled_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeTime()
            source = FakeSource([{"timestamp": 99.9}, {"timestamp": 100.0}])
            def reader():
                if source.round == 2:
                    raise KeyboardInterrupt
                return self.collect(source, clock)
            summary, _, _ = self.run_recording(
                directory, [], duration=0.4, clock=clock, read=reader)
            self.assertEqual(summary["stop_reason"], "cancelled")
            self.assertFalse(summary["complete"])
            self.assertEqual(summary["sample_count"], 2)
            self.assertEqual(len((Path(directory) / "record-1.jsonl").read_text().splitlines()), 2)
            self.assertEqual(load_recording_summary(Path(directory) / "record-1.summary.json"), summary)

    def test_unexpected_read_error_retains_rows_and_names_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeTime()
            source = FakeSource([{"timestamp": 99.9}])
            def reader():
                if source.round:
                    raise AssertionError("synthetic source contract broke")
                return self.collect(source, clock)
            summary, _, _ = self.run_recording(
                directory, [], duration=0.3, clock=clock, read=reader)
            self.assertEqual((summary["stop_reason"], summary["sample_count"],
                              summary["complete"]), ("fault", 1, False))
            self.assertIn("AssertionError: synthetic source contract broke", summary["error"])
            self.assertEqual(load_recording_summary(Path(directory) / "record-1.summary.json"), summary)

    def test_user_finish_is_normal_recording_end_only(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeTime()
            source = FakeSource([{"timestamp": 99.9}])
            summary = record_trajectory(
                recording_id="finish", output_directory=directory,
                parameters=RecordingParameters(1.0, 10.0),
                read_snapshot=lambda: self.collect(source, clock),
                monotonic_s=clock.monotonic, sleep_s=clock.sleep,
                cancel_requested=lambda: False,
                finish_requested=lambda: source.round >= 1,
                wall_clock_s=clock.wall)
            self.assertEqual((summary["stop_reason"], summary["sample_count"],
                              summary["complete"]), ("user_finish", 1, True))
            self.assertEqual(load_recording_summary(Path(directory) / "finish.summary.json"), summary)

    def test_clock_backwards_is_fault_with_previous_rows_preserved(self):
        class BackwardsTime(FakeTime):
            sleeps = 0
            def sleep(self, seconds):
                self.sleeps += 1
                self.now += seconds if self.sleeps == 1 else -0.05

        with tempfile.TemporaryDirectory() as directory:
            clock = BackwardsTime()
            summary, _, _ = self.run_recording(
                directory, [{"timestamp": 99.9}, {"timestamp": 100.0}],
                duration=0.4, clock=clock)
            self.assertEqual(summary["stop_reason"], "fault")
            self.assertFalse(summary["complete"])
            self.assertIn("moved backwards", summary["error"])
            self.assertEqual(summary["sample_count"], 2)
            self.assertEqual(len((Path(directory) / "record-1.jsonl").read_text().splitlines()), 2)

    def test_output_conflict_and_invalid_args_reject_before_source(self):
        def forbidden_source(_channel):
            raise AssertionError("invalid trajectory record reached device source")

        with tempfile.TemporaryDirectory() as directory:
            base = ["trajectory", "record", "--recording-id", "one",
                    "--output-directory", directory, "--duration-seconds", "0.2",
                    "--rate-hz", "10"]
            for changes in (("--rate-hz", "nan"), ("--duration-seconds", "0"),
                            ("--rate-hz", "101")):
                argv = base[:]
                argv[argv.index(changes[0]) + 1] = changes[1]
                with self.subTest(argv=argv):
                    stdout, stderr = io.StringIO(), io.StringIO()
                    with redirect_stdout(stdout), redirect_stderr(stderr):
                        code = main(argv, source_factory=forbidden_source)
                    self.assertEqual(code, 1)
                    self.assertIn("未完成", stderr.getvalue())
            raw = Path(directory) / "one.jsonl"
            raw.write_text("original", encoding="utf-8")
            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                code = main(base, source_factory=forbidden_source)
            self.assertEqual(code, 1)
            self.assertEqual(raw.read_text(), "original")
            self.assertFalse((Path(directory) / "one.summary.json").exists())

    def test_write_oserror_preserves_committed_rows_and_fault_summary(self):
        original_open = Path.open
        class FailingStream:
            def __init__(self, real):
                self.real = real
                self.writes = 0
            def __enter__(self):
                return self
            def __exit__(self, *args):
                return self.real.__exit__(*args)
            def write(self, value):
                self.writes += 1
                if self.writes == 2:
                    self.real.write(value[:5])  # 模拟第二行只写出一截。
                    raise OSError("synthetic disk write failure")
                return self.real.write(value)
            def flush(self):
                return self.real.flush()
            def fileno(self):
                return self.real.fileno()
            def tell(self):
                return self.real.tell()

        def patched_open(path, mode="r", *args, **kwargs):
            real = original_open(path, mode, *args, **kwargs)
            return FailingStream(real) if path.suffix == ".jsonl" and mode == "xb" else real

        with tempfile.TemporaryDirectory() as directory, patch.object(Path, "open", patched_open):
            summary, _, _ = self.run_recording(
                directory, [{"timestamp": 99.9}, {"timestamp": 100.0}], duration=0.3)
            raw = Path(directory) / "record-1.jsonl"
            self.assertEqual(summary["stop_reason"], "fault")
            self.assertIn("synthetic disk write failure", summary["error"])
            self.assertEqual(summary["sample_count"], 1)
            self.assertEqual(len(raw.read_text().splitlines()), 1)
            self.assertEqual(load_recording_summary(Path(directory) / "record-1.summary.json"), summary)

    def test_summary_hash_or_count_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            self.run_recording(directory, [{"timestamp": 99.9}], duration=0.15)
            path = Path(directory) / "record-1.summary.json"
            original = path.read_text()
            altered = json.loads(original)
            altered["sample_count"] += 1
            path.write_text(json.dumps(altered), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "sample count mismatch"):
                load_recording_summary(path)
            path.write_text(original, encoding="utf-8")
            raw = Path(directory) / "record-1.jsonl"
            raw.write_bytes(raw.read_bytes() + b" ")
            with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                load_recording_summary(path)

    def test_cli_record_fake_source_then_offline_show_and_list(self):
        with tempfile.TemporaryDirectory() as directory:
            clock = FakeTime()
            source = FakeSource([{"timestamp": 99.9}, {"timestamp": 100.0},
                                 {"timestamp": 100.1}])
            def opener(channel):
                return connected_nero(
                    channel, config_factory=lambda **kwargs: kwargs,
                    driver_factory=lambda config: source)

            stdout, stderr = io.StringIO(), io.StringIO()
            with redirect_stdout(stdout), redirect_stderr(stderr):
                code = main([
                    "trajectory", "record", "--recording-id", "cli-fake",
                    "--output-directory", directory,
                    "--duration-seconds", "0.25", "--rate-hz", "10",
                    "--robot-id", "synthetic-nero"],
                    source_factory=opener, clock_s=clock.wall,
                    monotonic_s=clock.monotonic, sleep_s=clock.sleep)
            self.assertEqual((code, stderr.getvalue()), (0, ""))
            self.assertIn("Recording: cli-fake", stdout.getvalue())
            self.assertEqual(source.calls[0], "connect")
            self.assertEqual(source.calls[-1], "disconnect")
            self.assertEqual(load_recording_summary(
                Path(directory) / "cli-fake.summary.json")["sample_count"], 3)

            def forbidden_opener(_channel):
                raise AssertionError("trajectory show/list opened device")
            for argv in (["trajectory", "show", str(Path(directory) / "cli-fake.summary.json")],
                         ["trajectory", "list", directory]):
                stdout, stderr = io.StringIO(), io.StringIO()
                with redirect_stdout(stdout), redirect_stderr(stderr):
                    code = main(argv, source_factory=forbidden_opener)
                self.assertEqual((code, stderr.getvalue()), (0, ""))
                self.assertIn("cli-fake", stdout.getvalue())
            self.assertNotIn("pyAgxArm", sys.modules)
            self.assertNotIn("can", sys.modules)

    def test_clean_cli_help_imports_no_sdk_can_experiments(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        imported = subprocess.run(
            [sys.executable, "-B", "-S", "-c",
             "import sys; import nero_runtime.recording, nero_runtime.cli; "
             "assert 'pyAgxArm' not in sys.modules; "
             "assert 'can' not in sys.modules; "
             "assert 'experiments' not in sys.modules"],
            cwd=root, env=environment, capture_output=True, text=True)
        self.assertEqual(imported.returncode, 0, imported.stderr)
        helped = subprocess.run(
            [sys.executable, "-B", "-S", "-m", "nero_runtime.cli", "trajectory", "--help"],
            cwd=root, env=environment, capture_output=True, text=True)
        self.assertEqual(helped.returncode, 0, helped.stderr)
        self.assertIn("record", helped.stdout)


if __name__ == "__main__":
    unittest.main()
