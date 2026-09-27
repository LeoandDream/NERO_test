"""Offline checks for the unified S1/H0 named-target CLI and API."""

import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

from experiments.single_arm.lab.api import LabAPI, h0_start_strategy
from experiments.single_arm.lab.cli import parser_create, quick_dispatch
from experiments.single_arm.lab.optimized_home import HOME_Q
from experiments.single_arm.lab.start_from_h0_candidate import SUPPORTED_Q


class NamedTargetTests(unittest.TestCase):
    def test_quick_init_and_park_share_named_planner(self):
        for action, target in (("init", "S1"), ("init", "H0"),
                               ("park", "H0")):
            args = parser_create().parse_args(
                ["quick", action, "--target", target])
            api = Mock()
            api.plan_start.return_value = {
                "plan_path": "saved.json",
                "plan": {"already_at_target": False, "target_name": target}}
            api.run_start.return_value = {"report_path": "run.json"}
            result = quick_dispatch(args, api, is_tty=True,
                                    input_fn=lambda _: "开始")
            api.plan_start.assert_called_once_with(target)
            api.run_start.assert_called_once_with("saved.json")
            self.assertEqual(result["result"]["report_path"], "run.json")

    def test_cancel_keeps_plan_read_only(self):
        args = parser_create().parse_args(
            ["quick", "init", "--target", "H0"])
        api = Mock()
        api.plan_start.return_value = {
            "plan_path": "saved.json",
            "plan": {"already_at_target": False, "target_name": "H0"}}
        with self.assertRaisesRegex(InterruptedError, "用户取消"):
            quick_dispatch(args, api, is_tty=True,
                           input_fn=lambda _: "取消")
        api.run_start.assert_not_called()

    def test_h0_state_recognition_distinguishes_enabled_and_supported(self):
        for joints, enabled, expected in (
                (HOME_Q, True, "enabled"),
                (SUPPORTED_Q, False, "supported")):
            state = {"control_mode": 1, "arm_status": 0 if enabled else 6,
                     "error_code": 0, "ready_for_motion": enabled,
                     "joint_variation_0_25s_rad": 0.0,
                     "joint_rad": list(joints),
                     "drivers": [{"enabled": enabled,
                                  "undervoltage": False,
                                  "driver_error": False} for _ in range(7)]}
            self.assertEqual(h0_start_strategy(state), expected)
            state["joint_variation_0_25s_rad"] = 0.003
            state["ready_for_motion"] = False
            self.assertEqual(h0_start_strategy(state), expected)
            state["drivers"][0]["driver_error"] = True
            self.assertIsNone(h0_start_strategy(state))

    def test_plan_s1_from_enabled_h0_saves_named_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "config.json"
            config.write_text("{}", encoding="utf-8")
            preview = root / "preview.json"
            route = [list(HOME_Q), [v+0.01 for v in HOME_Q]]
            preview.write_text(json.dumps({"kind": "optimized_sparse_s1_h0_trial",
                                           "direction": "start",
                                           "execution_requested": False,
                                           "preflight_passed": True,
                                           "completed": False,
                                           "strategy": "reduced-j3",
                                           "route_joint_rad": route}),
                               encoding="utf-8")
            api = LabAPI(config_path=config)
            state = {"control_mode": 1, "arm_status": 0, "error_code": 0,
                     "ready_for_motion": True,
                     "joint_variation_0_25s_rad": 0.0,
                     "joint_rad": list(HOME_Q),
                     "drivers": [{"enabled": True, "undervoltage": False,
                                  "driver_error": False} for _ in range(7)]}
            with patch.object(api, "status", return_value=state), patch(
                    "experiments.single_arm.lab.optimized_home.run",
                    return_value=preview) as optimized, patch(
                    "experiments.single_arm.lab.starts.save_start_plan",
                    return_value=root / "saved.json"):
                planned = api.plan_start("S1")
            self.assertEqual(planned["plan"]["kind"],
                             "named_joint_target_plan")
            self.assertEqual(planned["plan"]["direction"], "start")
            optimized.assert_called_once()

    def test_plan_h0_from_s1_uses_the_same_saved_plan_kind(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "config.json"
            config.write_text("{}", encoding="utf-8")
            preview = root / "preview.json"
            route = [[0.017, -0.579, 0.028, -0.300, 0.011, 0.206, -0.473],
                     list(HOME_Q)]
            preview.write_text(json.dumps({
                "kind": "optimized_sparse_s1_h0_trial", "direction": "home",
                "execution_requested": False, "preflight_passed": True,
                "completed": False, "strategy": "reduced-j3",
                "route_joint_rad": route}), encoding="utf-8")
            api = LabAPI(config_path=config)
            with patch.object(api, "status", return_value={
                    "control_mode": 1, "arm_status": 0, "error_code": 0,
                    "ready_for_motion": True,
                    "joint_variation_0_25s_rad": 0.0,
                    "joint_rad": route[0],
                    "drivers": [{"enabled": True, "undervoltage": False,
                                 "driver_error": False} for _ in range(7)]}), patch(
                    "experiments.single_arm.lab.optimized_home.run",
                    return_value=preview) as optimized, patch(
                    "experiments.single_arm.lab.starts.save_start_plan",
                    return_value=root / "saved.json"):
                planned = api.plan_start("H0")
            self.assertEqual(planned["plan"]["kind"],
                             "named_joint_target_plan")
            self.assertEqual(planned["plan"]["direction"], "home")
            optimized.assert_called_once()

    def test_saved_home_plan_rechecks_live_route_before_execution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "config.json"
            config.write_text("{}", encoding="utf-8")
            route = [[0.0]*7, [0.1]*7]
            preview = root / "preview.json"
            preview.write_text(json.dumps({
                "kind": "optimized_sparse_s1_h0_trial",
                "direction": "home", "strategy": "reduced-j3",
                "execution_requested": False, "preflight_passed": True,
                "completed": False, "route_joint_rad": route}),
                               encoding="utf-8")
            saved = root / "saved.json"
            saved.write_text(json.dumps({
                "schema_version": 1, "kind": "named_joint_target_plan",
                "target_name": "H0", "route_kind": "optimized_home",
                "direction": "home", "strategy": "reduced-j3",
                "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                "preview_report_path": str(preview),
                "preview_sha256": hashlib.sha256(preview.read_bytes()).hexdigest(),
                "start_joint_rad": route[0], "route_joint_rad": route,
                "controller_targets": 1, "already_at_target": False,
            }), encoding="utf-8")
            api = LabAPI(config_path=config)
            connector = Mock()
            connector.__enter__ = Mock(return_value=Mock())
            connector.__exit__ = Mock(return_value=False)
            with patch("experiments.single_arm.lab.api.load_config",
                       return_value={"mount": "left"}), patch(
                       "experiments.single_arm.lab.api.connected",
                       return_value=connector), patch(
                       "experiments.single_arm.lab.api.read_state",
                       return_value={"joint_rad": route[0]}), patch(
                       "experiments.single_arm.lab.optimized_home.plan",
                       return_value={"route_joint_rad": route}), patch(
                       "experiments.single_arm.lab.optimized_home.run",
                       return_value=root / "run.json") as execute:
                result = api.run_start(saved)
            self.assertEqual(result["target_name"], "H0")
            execute.assert_called_once()
            self.assertTrue(execute.call_args.kwargs["execute"])

    def test_saved_start_plan_accepts_encoder_noise_in_single_joint_release(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / "config.json"
            config.write_text("{}", encoding="utf-8")
            start = list(HOME_Q)
            release = list(start)
            release[0] = 0.45
            target = [0.0] * 7
            route = [start, release, target]
            preview = root / "preview.json"
            preview_bytes = json.dumps({
                "kind": "optimized_sparse_s1_h0_trial",
                "direction": "start", "strategy": "reduced-j3",
                "execution_requested": False, "preflight_passed": True,
                "completed": False, "route_joint_rad": route,
            }).encode()
            preview.write_bytes(preview_bytes)
            saved = root / "saved.json"
            saved.write_text(json.dumps({
                "schema_version": 1, "kind": "named_joint_target_plan",
                "target_name": "S1", "route_kind": "optimized_home",
                "direction": "start", "strategy": "reduced-j3",
                "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                "preview_report_path": str(preview),
                "preview_sha256": hashlib.sha256(preview_bytes).hexdigest(),
                "route_joint_rad": route, "controller_targets": 2,
            }), encoding="utf-8")
            live_start = [v + 0.0001 for v in start]
            live_release = list(live_start)
            live_release[0] = release[0]
            connector = Mock()
            connector.__enter__ = Mock(return_value=Mock())
            connector.__exit__ = Mock(return_value=False)
            api = LabAPI(config_path=config)
            with patch("experiments.single_arm.lab.api.load_config",
                       return_value={}), patch(
                       "experiments.single_arm.lab.api.connected",
                       return_value=connector), patch(
                       "experiments.single_arm.lab.api.read_state",
                       return_value={"joint_rad": live_start}), patch(
                       "experiments.single_arm.lab.optimized_home.plan",
                       return_value={"route_joint_rad": [
                           live_start, live_release, target]}), patch(
                       "experiments.single_arm.lab.optimized_home.run",
                       return_value=root / "run.json") as execute:
                api.run_start(saved)
            execute.assert_called_once()


if __name__ == "__main__":
    unittest.main()
