"""Supported-home planning and CLI gating, with no hardware commands."""

from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from experiments.single_arm.lab.cli import parser_create, quick_dispatch
from experiments.single_arm.lab.supported_home import (
    _forward_route, _from_snapshot, load_home, run,
)


S1 = [0.017383479349863524, -0.5787511799613198, 0.028047441079548877,
      -0.2995159529347469, 0.011239920382843483, 0.2056695990550118,
      -0.47343801289598186]
H0 = [0.20149826214274535, 0.31012755478687243, -0.028082347664588763,
      -0.11386528040011006, 1.6763189333704738, 0.13177235852557187,
      0.5924345612969553]


def state(q, mode=1):
    return {"control_mode": mode, "arm_status": 0, "teach_status": 0,
            "error_code": 0, "joint_variation_0_25s_rad": 0,
            "drivers": [{"enabled": True, "undervoltage": False,
                         "driver_error": False} for _ in range(7)],
            "ready_for_motion": mode == 1,
            "reasons": [] if mode == 1 else ["控制器未处于 CAN 控制模式"],
            "joint_rad": list(q)}


class SupportedHomeTests(unittest.TestCase):
    def test_collided_home_route_rejected_before_connect(self):
        with tempfile.TemporaryDirectory() as folder:
            config_path = Path(folder) / "config.json"
            home_path = Path(folder) / "home.json"
            config_path.write_text("{}", encoding="utf-8")
            home_path.write_text(json.dumps({
                "route_blocked": True,
                "route_block_reason": "首次回位碰撞",
            }), encoding="utf-8")
            with patch("experiments.single_arm.lab.supported_home.load_config",
                       return_value={}):
                with self.assertRaisesRegex(RuntimeError, "首次回位碰撞"):
                    load_home(config_path, home_path)

    def test_sparse_route_lifts_first_on_departure_and_approaches_last(self):
        config = {"safe_start_joint_rad": S1, "zero_exclusion_radius_rad": 0.25,
                  "flange_workspace_m": None}
        home = {"joint_rad": H0}
        boxes = [{"x": [lower, lower + 0.02], "y": [0, 0], "z": [0, 0]}
                 for lower in (0.38, 0.39, 0.35, -0.10, -0.122)]
        with patch("experiments.single_arm.lab.supported_home._assess",
                   side_effect=[boxes, list(reversed(boxes))]):
            forward = _from_snapshot(None, state(S1), config, home, "H0")
            backward = _from_snapshot(None, state(H0, mode=2), config, home, "S1")
        self.assertEqual(forward["controller_targets"], 5)
        self.assertEqual(forward["speed_percent_by_target"], [5, 5, 5, 5, 2])
        self.assertEqual(backward["speed_percent_by_target"], [2, 5, 5, 5, 5])
        self.assertEqual(forward["route_joint_rad"][-2][1], H0[1] - 0.04)
        self.assertEqual(backward["route_joint_rad"][1][1], H0[1] - 0.04)
        self.assertEqual(backward["route_joint_rad"][-1], S1)
        self.assertEqual(_forward_route(S1, H0)[-1], H0)

    def test_unknown_pose_rejected_before_route_assessment(self):
        config = {"safe_start_joint_rad": S1, "zero_exclusion_radius_rad": 0.25}
        home = {"joint_rad": H0}
        with patch("experiments.single_arm.lab.supported_home._assess") as assess:
            with self.assertRaisesRegex(ValueError, "不在已记录"):
                _from_snapshot(None, state([0.5] * 7), config, home, "H0")
        assess.assert_not_called()

    def test_cli_cancel_never_executes_route(self):
        class API:
            def __init__(self):
                self.runs = []

            def plan_start(self, target):
                self.target = target
                return {"plan_path": "plan.json", "plan": {
                    "controller_targets": 5, "already_at_target": False}}

            def run_start(self, path):
                self.runs.append(path)
                return {"report_path": "run.json"}

        api = API()
        args = parser_create().parse_args(["quick", "park", "--target", "H0"])
        with self.assertRaises(InterruptedError):
            quick_dispatch(args, api, is_tty=True, input_fn=lambda _: "取消")
        self.assertEqual(api.runs, [])
        quick_dispatch(args, api, is_tty=True, input_fn=lambda _: "开始")
        self.assertEqual(api.target, "H0")
        self.assertEqual(api.runs, ["plan.json"])

    def test_executor_lifts_from_support_before_faster_targets(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            config_path, home_path = folder / "config.json", folder / "home.json"
            config_path.write_text("{}", encoding="utf-8")
            home_path.write_text("{}", encoding="utf-8")
            route = list(reversed(_forward_route(S1, H0)))
            plan = {"schema_version": 1, "kind": "supported_home_move_j",
                    "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
                    "home_config_sha256": hashlib.sha256(home_path.read_bytes()).hexdigest(),
                    "source_recording": "demo.json", "reversed_recording": False,
                    "automatic_disable": False, "target_name": "S1",
                    "source_name": "H0", "controller_targets": 5,
                    "route_joint_rad": route, "speed_percent_by_target": [2, 5, 5, 5, 5],
                    "start_state": state(H0, mode=2)}
            path = folder / "plan.json"
            path.write_text(json.dumps(plan), encoding="utf-8")
            config = {"safe_start_joint_rad": S1}
            home = {"source_report": "demo.json", "joint_rad": H0}
            current = {"source_name": "H0", "target_name": "S1",
                       "controller_targets": 5, "route_joint_rad": route,
                       "speed_percent_by_target": [2, 5, 5, 5, 5]}
            calls = []

            @contextmanager
            def connected(_config, _factory):
                yield object()

            def execute(_robot, _config, points, _aborted, **kwargs):
                calls.append((points, kwargs["speed_percent"]))
                for index, target in enumerate(points[1:], 1):
                    kwargs["on_waypoint"](index, target)

            final = {**state(S1), "ready_for_motion": True}
            with (patch("experiments.single_arm.lab.supported_home.load_home",
                        return_value=(config, home, H0)),
                  patch("experiments.single_arm.lab.supported_home.connected", connected),
                  patch("experiments.single_arm.lab.supported_home.read_state",
                        side_effect=[state(H0, mode=2), final]),
                  patch("experiments.single_arm.lab.supported_home._from_snapshot",
                        return_value=current),
                  patch("experiments.single_arm.lab.supported_home.execute_route",
                        side_effect=execute),
                  patch("experiments.single_arm.lab.supported_home.joint_point",
                        return_value=S1),
                  patch("experiments.single_arm.lab.supported_home.RUN_DIR", folder)):
                report_path = run(path, config_path=config_path,
                                  home_path=home_path, countdown_s=0)
            self.assertEqual([speed for _points, speed in calls], [2, 5])
            self.assertEqual(len(calls[0][0]), 2)
            self.assertEqual(calls[0][0][-1][1], H0[1] - 0.04)
            self.assertEqual(calls[1][0][-1], S1)
            saved = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertTrue(saved["completed"])
            self.assertFalse(saved["automatic_disable"])


if __name__ == "__main__":
    unittest.main()
