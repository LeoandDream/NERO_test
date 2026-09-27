"""The drag demonstration must always exit drag and preserve its evidence."""

import csv
import hashlib
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from experiments.single_arm.lab.api import LabAPI
from experiments.single_arm.lab.cli import parser_create, quick_dispatch
from experiments.single_arm.lab.home_demo import reassess_saved_demo


class HomeDemoTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)

    @staticmethod
    def ready():
        return {"ready_for_motion": True, "reasons": [],
                "control_mode": 1, "arm_status": 0, "error_code": 0,
                "teach_status": 6, "joint_variation_0_25s_rad": 0,
                "drivers": [{"enabled": True, "undervoltage": False,
                             "driver_error": False} for _ in range(7)],
                "joint_rad": [0] * 7, "flange_pose_m_rad": [0] * 6}

    def _csv(self, folder):
        path = folder / "route.csv"
        keys = (["elapsed_s"] + [f"joint_{i}_rad" for i in range(1, 8)]
                + ["x_m", "y_m", "z_m"])
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader()
            for t, q, x in ((0, 0, 0), (1, 0.1, 0.03),
                            (2, 0.2, 0.06), (3, 0.2, 0.06)):
                writer.writerow({"elapsed_s": t, "joint_2_rad": q,
                                 "x_m": x, "y_m": 0, "z_m": 0,
                                 **{f"joint_{i}_rad": 0 for i in (1, 3, 4, 5, 6, 7)}})
        return path

    def test_success_stops_drag_without_return_or_disable(self):
        api = LabAPI()
        commands = []

        def runner(command, **_kwargs):
            commands.append(command)
            return SimpleNamespace(returncode=0, stdout="ok", stderr="")

        def record(*, output_dir, **kwargs):
            self.assertEqual(kwargs["required_state"], "drag")
            path = self._csv(output_dir)
            return {"recording_path": str(path), "samples": 4,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

        final_state = {**self.ready(), "joint_rad": [0, 0.2, 0, 0, 0, 0, 0],
                       "control_mode": 2, "teach_status": 0,
                       "ready_for_motion": False,
                       "reasons": ["控制器未处于 CAN 控制模式"]}
        with (patch.object(api, "status", side_effect=[self.ready(), final_state]) as status,
              patch.object(api, "record_poses", side_effect=record)):
            result = api.home_demo(duration_s=30, rate_hz=20,
                                   command_runner=runner, data_dir=self.folder)
        self.assertEqual(["--start", "--stop"], [item[3] for item in commands])
        self.assertEqual(status.call_count, 2)
        self.assertFalse(result["automatic_return"])
        self.assertFalse(result["automatic_disable"])
        self.assertTrue(result["requires_can_mode_before_motion"])
        report = json.loads(Path(result["report_path"]).read_text(encoding="utf-8"))
        self.assertTrue(report["completed"])
        self.assertEqual(report["trajectory_summary"]["max_joint_excursion_rad"], 0.2)
        self.assertEqual(report["trajectory_summary"]["final_joint_variation_last_1s_rad"], 0)
        self.assertFalse(report["physical_support_confirmed"])
        reassessed = reassess_saved_demo(result["report_path"], assessment_dir=self.folder)
        self.assertTrue(reassessed["recording_accepted"])
        self.assertTrue(reassessed["requires_can_mode_before_motion"])

    def test_record_failure_still_exits_drag_and_saves_report(self):
        api = LabAPI()
        commands = []

        def runner(command, **_kwargs):
            commands.append(command)
            return SimpleNamespace(returncode=0, stdout="ok", stderr="")

        with (patch.object(api, "status", return_value=self.ready()),
              patch.object(api, "record_poses", side_effect=RuntimeError("feedback lost"))):
            with self.assertRaisesRegex(RuntimeError, "报告"):
                api.home_demo(command_runner=runner, data_dir=self.folder)
        self.assertEqual(["--start", "--stop"], [item[3] for item in commands])
        report = json.loads(next(self.folder.glob("*.json")).read_text(encoding="utf-8"))
        self.assertFalse(report["completed"])
        self.assertEqual(report["error"], "feedback lost")

    def test_cli_requires_confirmation_and_script_parameters(self):
        class API:
            def __init__(self):
                self.calls = []

            def home_demo(self, **kwargs):
                self.calls.append(kwargs)
                return {}

        api = API()
        args = parser_create().parse_args(["quick", "home-demo"])
        with self.assertRaises(InterruptedError):
            quick_dispatch(args, api, is_tty=True, input_fn=lambda _: "取消")
        self.assertEqual(api.calls, [])
        quick_dispatch(args, api, is_tty=True, input_fn=lambda _: "开始")
        self.assertEqual(api.calls[-1], {"duration_s": 30, "rate_hz": 20})
        scripted = parser_create().parse_args(["quick", "home-demo", "--yes"])
        with self.assertRaisesRegex(ValueError, "--yes 须明确指定"):
            quick_dispatch(scripted, api, is_tty=False)


if __name__ == "__main__":
    unittest.main()
