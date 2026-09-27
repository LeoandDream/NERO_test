"""边界拖动的一条命令生命周期；全部使用替身，不访问 CAN。"""

import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.single_arm.lab.api import LabAPI
from experiments.single_arm.lab.cli import parser_create, quick_dispatch
from experiments.single_arm.lab.cube_sweep import run_sweep


class CubeSweepTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)

    @staticmethod
    def ready():
        return {"ready_for_motion": True, "reasons": [],
                "joint_rad": [0.1] * 7, "flange_pose_m_rad": [0.3] * 6}

    def test_one_command_stops_drag_and_preserves_candidate(self):
        api = LabAPI()
        commands = []

        def runner(command, **_kwargs):
            commands.append(command)
            return SimpleNamespace(returncode=0, stdout="ok", stderr="")

        candidate = {"map_path": "map.csv", "candidate_path": "candidate.json",
                     "candidate": {"status": "EXTREMA_CANDIDATE_ONLY"}}
        with (patch.object(api, "status", return_value=self.ready()),
              patch.object(api, "record_poses", return_value={
                  "recording_path": "new.csv", "samples": 600}) as record,
              patch.object(api, "cube_extrema_from_recording",
                           return_value=candidate) as analyze):
            result = run_sweep(api, duration_s=30, rate_hz=20,
                               command_runner=runner, report_dir=self.folder)
        self.assertEqual(["--start", "--stop"],
                         [command[3] for command in commands])
        self.assertEqual(record.call_args.kwargs["required_state"], "drag")
        analyze.assert_called_once_with("new.csv", margin_m=0.01)
        self.assertFalse(result["automatic_return"])
        report = json.loads(Path(result["report_path"]).read_text(encoding="utf-8"))
        self.assertTrue(report["completed"])
        self.assertEqual(report["candidate_status"], "EXTREMA_CANDIDATE_ONLY")

    def test_record_failure_still_exits_drag(self):
        api = LabAPI()
        commands = []

        def runner(command, **_kwargs):
            commands.append(command)
            return SimpleNamespace(returncode=0, stdout="ok", stderr="")

        with (patch.object(api, "status", return_value=self.ready()),
              patch.object(api, "record_poses", side_effect=RuntimeError(
                  "部分原始 CSV 保留在 new.csv")),
              patch.object(api, "cube_extrema_from_recording") as analyze):
            with self.assertRaisesRegex(RuntimeError, "报告"):
                run_sweep(api, command_runner=runner, report_dir=self.folder)
        self.assertEqual(["--start", "--stop"],
                         [command[3] for command in commands])
        analyze.assert_not_called()
        report = json.loads(next(self.folder.glob("*.json")).read_text(encoding="utf-8"))
        self.assertFalse(report["completed"])
        self.assertIn("new.csv", report["error"])

    def test_quick_sweep_requires_confirmation_and_full_script_parameters(self):
        class API:
            def __init__(self):
                self.calls = []

            def status(self):
                return CubeSweepTests.ready()

            def cube_sweep(self, **kwargs):
                self.calls.append(kwargs)
                return {"candidate_status": "EXTREMA_CANDIDATE_ONLY"}

        api = API()
        args = parser_create().parse_args(["quick", "cube-sweep"])
        with self.assertRaisesRegex(ValueError, "非交互"):
            quick_dispatch(args, api, is_tty=False)
        with self.assertRaises(InterruptedError):
            quick_dispatch(args, api, is_tty=True, input_fn=lambda _: "取消")
        self.assertEqual(api.calls, [])
        quick_dispatch(args, api, is_tty=True, input_fn=lambda _: "开始")
        self.assertEqual(api.calls[-1], {
            "duration_s": 30, "rate_hz": 20, "margin_m": 0.01})
        scripted = parser_create().parse_args(["quick", "cube-sweep", "--yes"])
        with self.assertRaisesRegex(ValueError, "--yes 须明确指定"):
            quick_dispatch(scripted, api, is_tty=False)

    def test_drag_recorder_rejects_wrong_control_state(self):
        now = time.time()

        class Robot:
            def connect(self):
                pass

            def disconnect(self):
                pass

            def get_joint_angles(self):
                return SimpleNamespace(timestamp=now, msg=[0.1] * 7)

            def get_flange_pose(self):
                return SimpleNamespace(timestamp=now, msg=[0.3] * 6)

            def get_arm_status(self):
                return SimpleNamespace(timestamp=now, msg=SimpleNamespace(
                    ctrl_mode=1, arm_status=0, teach_status=0, err_code=0))

        api = LabAPI(robot_factory=lambda _channel: Robot())
        with self.assertRaisesRegex(RuntimeError, "拖动状态异常"):
            api.record_poses(duration_s=0.1, rate_hz=10,
                             output_dir=self.folder, required_state="drag")
