"""Step 3C：真实旧规划函数在合成 FK 下的候选/执行边界。"""

from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

# lab 包入口会提前导入 recordings → teach_session → SDK；在包导入前
# 安装拒绝型无设备替身。只替代本轮测试不调用的记录功能。
_recordings = types.ModuleType("experiments.single_arm.lab.recordings")
def _refuse_recording(*_args, **_kwargs):
    raise AssertionError("recording boundary reached")
_recordings.RecordingInfo = _refuse_recording
_recordings.list_recordings = _refuse_recording
_recordings.resolve_recording = _refuse_recording
sys.modules[_recordings.__name__] = _recordings

from experiments.single_arm.lab import device, planned_return


class FakeFK:
    def fk(self, joints):
        return [float(joints[0]), 0.0, 0.7, 0.0, 0.0, 0.0]


def checked_synthetic_joints(joints, zero_radius):
    if len(joints) != 7 or not all(type(v) in (int, float) and math.isfinite(v)
                                    for v in joints):
        raise ValueError("synthetic joint input invalid")
    if math.dist(joints, [0.0] * 7) < zero_radius:
        raise ValueError("synthetic zero pose excluded")
    return list(joints)


class ReturnConstraintTests(unittest.TestCase):
    def setUp(self):
        self.robot = FakeFK()
        self.config = {"mount": "left", "channel": "synthetic",
                       "safe_start_joint_rad": [0.2, 0, 0, 0.4, 0, 0, 0],
                       "zero_exclusion_radius_rad": 0.25,
                       "flange_workspace_m": None}
        self.joints = patch.object(planned_return, "_validate_joints",
                                   side_effect=checked_synthetic_joints)
        self.workspace = patch.object(planned_return, "_validate_route_workspace",
                                      return_value=None)
        self.joints.start()
        self.workspace.start()
        self.addCleanup(self.joints.stop)
        self.addCleanup(self.workspace.stop)

    def state(self, x):
        joints = [x, 0, 0, 0.4, 0, 0, 0]
        return {"control_mode": 1, "arm_status": 0, "teach_status": 0,
                "error_code": 0, "joint_variation_0_25s_rad": 0.0,
                "drivers": [{"enabled": True, "undervoltage": False,
                             "driver_error": False} for _ in range(7)],
                "joint_rad": joints,
                "flange_pose_m_rad": list(self.robot.fk(joints)),
                "ready_for_motion": True}

    def test_no_default_x_floor_allows_two_numeric_candidates(self):
        for x in (0.10, -0.18):
            with self.subTest(x=x):
                candidate = planned_return.plan_from_snapshot(
                    self.robot, self.state(x), self.config)
                self.assertEqual(candidate["start_joint_rad"][0], x)
                self.assertIsNone(candidate["flange_x_constraint"])
                self.assertEqual(candidate["flange_x_check"], "not_evaluated")
                self.assertEqual(candidate["environment_assessment"], "not_evaluated")
                self.assertEqual(candidate["execution_status"], "candidate_only")
                self.assertEqual(candidate["route_joint_rad"][-1],
                                 self.config["safe_start_joint_rad"])

    def test_explicit_negative_constraint_passes_and_violation_names_value(self):
        allowed = planned_return.flange_x_constraint(-0.25, "synthetic fixture")
        candidate = planned_return.plan_from_snapshot(
            self.robot, self.state(-0.18), self.config, allowed)
        self.assertEqual(candidate["flange_x_check"], "sampled_pass")
        self.assertEqual(candidate["flange_x_constraint"], allowed)
        self.assertEqual(candidate["execution_status"], "candidate_only")

        denied = planned_return.flange_x_constraint(-0.10, "synthetic fixture")
        with self.assertRaisesRegex(ValueError, r"X=-0.180 m.*-0.100 m"):
            planned_return.plan_from_snapshot(
                self.robot, self.state(-0.18), self.config, denied)

    def test_sampled_route_violation_is_specific_to_explicit_condition(self):
        class DipFK:
            def fk(self, joints):
                return [0.2 - 0.15 * math.sin(math.pi * joints[0]),
                        0.0, 0.7, 0.0, 0.0, 0.0]

        robot = DipFK()
        start = self.state(0.0)
        start["flange_pose_m_rad"] = list(robot.fk(start["joint_rad"]))
        config = dict(self.config, safe_start_joint_rad=[1.0, 0, 0, 0.4, 0, 0, 0])
        condition = planned_return.flange_x_constraint(0.10, "synthetic dip")
        with self.assertRaisesRegex(ValueError, r"最小采样值 0.050 m.*0.100 m"):
            planned_return.plan_from_snapshot(robot, start, config, condition)

    def test_existing_specialist_assess_numeric_condition_is_preserved(self):
        route = [self.state(-0.18)["joint_rad"], self.state(-0.10)["joint_rad"]]
        ranges = planned_return._assess(self.robot, route, self.config, -0.19)
        self.assertEqual(len(ranges), 1)
        self.assertAlmostEqual(ranges[0]["x"][0], -0.18)
        with self.assertRaisesRegex(ValueError, r"-0.180 m.*-0.150 m"):
            planned_return._assess(self.robot, route, self.config, -0.15)

    def test_missing_scope_or_bad_numbers_do_not_gain_a_check(self):
        good = planned_return.flange_x_constraint(-0.3, "synthetic")
        for changed in (dict(good, frame="table"), dict(good, unit="mm"),
                        dict(good, source=""), dict(good, minimum_x_m=math.nan),
                        dict(good, minimum_x_m=True),
                        dict(good, minimum_x_m=10**1000),
                        {"minimum_x_m": -0.3}):
            with self.subTest(changed=changed):
                with self.assertRaises(ValueError):
                    planned_return.plan_from_snapshot(
                        self.robot, self.state(-0.18), self.config, changed)
        with self.assertRaisesRegex(ValueError, "明确数值"):
            planned_return.plan_from_snapshot(
                self.robot, self.state(-0.18), self.config, 0.15)
        state = self.state(-0.18)
        state["flange_pose_m_rad"][0] = math.nan
        with self.assertRaisesRegex(ValueError, "反馈缺失或坐标"):
            planned_return.plan_from_snapshot(self.robot, state, self.config)
        state["flange_pose_m_rad"][0] = 10**1000
        with self.assertRaisesRegex(ValueError, "反馈缺失或坐标"):
            planned_return.plan_from_snapshot(self.robot, state, self.config)
        state = self.state(-0.18)
        state["arm_status"] = 1
        with self.assertRaisesRegex(RuntimeError, "整机状态 1"):
            planned_return.plan_from_snapshot(self.robot, state, self.config)

    def test_narrow_low_pose_prefix_remains_candidate_without_general_floor(self):
        class NarrowFK:
            def fk(self, joints):
                ref = planned_return.SITE_LOW_REFERENCE_Q
                x = 0.038 + 0.55 * (ref[1] - joints[1])
                y = -0.01 + 0.07 * (joints[0] - ref[0]) + 0.06 * (joints[2] - ref[2])
                return [x, y, 0.718, 0.0, 0.0, 0.0]

        robot = NarrowFK()
        start = self.state(0.1)
        start["joint_rad"] = list(planned_return.SITE_LOW_REFERENCE_Q)
        start["flange_pose_m_rad"] = list(robot.fk(start["joint_rad"]))
        target = list(start["joint_rad"])
        target[2], target[0], target[1] = -0.30, 0.0, -0.30
        config = dict(self.config, safe_start_joint_rad=target)
        candidate = planned_return.plan_from_snapshot(
            robot, start, config, allow_site_low_pose=True)
        self.assertTrue(candidate["site_low_pose_trial"])
        self.assertEqual(candidate["site_prefix_check"], "sampled_trial_condition_only")
        self.assertEqual(candidate["flange_x_check"], "not_evaluated")
        self.assertEqual(candidate["execution_status"], "candidate_only")
        self.assertEqual(candidate["route_joint_rad"][-1], target)
        with self.assertRaisesRegex(ValueError, r"最小采样值.*0.100 m"):
            planned_return.plan_from_snapshot(
                robot, start, config,
                planned_return.flange_x_constraint(0.10, "synthetic low prefix"),
                allow_site_low_pose=True)

    def test_prepare_marks_new_version_and_never_uses_real_device(self):
        @contextmanager
        def fake_connected(config, factory):
            self.assertEqual(config["channel"], "synthetic")
            yield self.robot

        with tempfile.TemporaryDirectory() as directory:
            config_path = Path(directory) / "synthetic.json"
            config_path.write_text("{}", encoding="utf-8")
            with patch.object(planned_return, "_load_config", return_value=self.config), \
                    patch.object(device, "connected", side_effect=fake_connected), \
                    patch.object(device, "read_state", return_value=self.state(-0.18)):
                candidate = planned_return.prepare(
                    config_path, robot_factory=lambda *_: self.fail("factory reached"))
            self.assertEqual(candidate["schema_version"], 2)
            self.assertEqual(candidate["execution_status"], "candidate_only")
            self.assertEqual(candidate["flange_x_check"], "not_evaluated")

    def test_run_rejects_legacy_and_new_candidate_before_any_factory(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            for payload, message in (
                    ({"schema_version": 1, "kind": "live_planned_return_to_s1",
                      "min_flange_x_m": 0.15}, "历史回位计划"),
                    ({"schema_version": 2, "kind": "live_planned_return_to_s1",
                      "execution_status": "candidate_only",
                      "validated": True}, "候选尚缺"),
                    ({"schema_version": 999, "kind": "live_planned_return_to_s1"},
                     "版本或候选状态不明")):
                with self.subTest(payload=payload):
                    path.write_text(json.dumps(payload), encoding="utf-8")
                    with patch.object(device, "connected", side_effect=AssertionError(
                            "control boundary reached")):
                        with self.assertRaisesRegex(ValueError, message):
                            planned_return.run(
                                path, robot_factory=lambda *_: self.fail("factory reached"),
                                site_clearance_confirmed=True)

    def test_import_isolation_and_actual_api_cli_dispatch(self):
        root = Path(__file__).resolve().parents[1]
        code = r'''
import importlib, json, pathlib, sys, tempfile, types
from unittest.mock import Mock, patch

recordings = types.ModuleType("experiments.single_arm.lab.recordings")
def refuse_recording(*args, **kwargs): raise AssertionError("recording boundary reached")
recordings.RecordingInfo = refuse_recording
recordings.list_recordings = refuse_recording
recordings.resolve_recording = refuse_recording
sys.modules[recordings.__name__] = recordings
import experiments.single_arm.lab.planned_return as planner
import experiments.single_arm.lab.device as device
assert not ({"pyAgxArm", "can"} & {name.split(".")[0] for name in sys.modules})
assert "experiments.single_arm.teaching.teach_session" not in sys.modules
assert "experiments.single_arm.start_transfer.go_to_start" not in sys.modules

for suffix in ("cube", "joint_probe", "pose_candidate", "recordings", "replay", "starts"):
    name = "experiments.single_arm.lab." + suffix
    stub = types.ModuleType(name)
    if suffix == "cube": stub.FACES = ("front",)
    sys.modules[name] = stub
teach = types.ModuleType("experiments.single_arm.teaching.teach_session")
teach.DEFAULT_CONFIG = pathlib.Path("synthetic.json")
teach.check_start = lambda *_: (_ for _ in ()).throw(AssertionError("start reached"))
teach.load_config = lambda *_: {"channel": "synthetic"}
teach.run_session = lambda *_: (_ for _ in ()).throw(AssertionError("drag reached"))
sys.modules[teach.__name__] = teach
guard = types.ModuleType("experiments.single_arm.can_setup.drag_start_guard")
guard.require_drag_start_safe = lambda *_: (_ for _ in ()).throw(AssertionError("guard reached"))
sys.modules[guard.__name__] = guard

api_module = importlib.import_module("experiments.single_arm.lab.api")
cli = importlib.import_module("experiments.single_arm.lab.cli")
api = api_module.LabAPI(config_path="synthetic.json")
candidate = {"schema_version": 2, "kind": "live_planned_return_to_s1",
             "controller_targets": 2, "execution_status": "candidate_only"}
with patch.object(planner, "prepare", return_value=dict(candidate)) as prepare, \
        patch.object(planner, "save_plan", return_value=pathlib.Path("candidate.json")):
    out = api.plan_return()
    assert out["plan"]["execution_status"] == "candidate_only"
    assert prepare.call_args.args[1] is None

args = cli.parser_create().parse_args(["return", "plan"])
fake = Mock()
cli.dispatch(args, fake)
assert fake.plan_return.call_args.kwargs == {"flange_x_constraint": None}
args = cli.parser_create().parse_args(
    ["return", "plan", "--min-flange-x-m", "-0.3",
     "--flange-x-source", "synthetic fixture"])
cli.dispatch(args, fake)
passed = fake.plan_return.call_args.kwargs["flange_x_constraint"]
assert passed["minimum_x_m"] == -0.3 and passed["source"] == "synthetic fixture"
args = cli.parser_create().parse_args(
    ["return", "plan", "--min-flange-x-m", "0.1"])
try: cli.dispatch(args, fake)
except ValueError as exc: assert "同时提供" in str(exc)
else: raise AssertionError("source omission accepted")

quick = cli.parser_create().parse_args(["quick", "init", "--target", "S1"])
quick_api = Mock()
quick_api.plan_start.return_value = {"plan_path": "candidate.json",
                                    "plan": dict(candidate, already_at_target=False)}
try: cli.quick_dispatch(quick, quick_api, is_tty=True)
except ValueError as exc: assert "仅为" in str(exc)
else: raise AssertionError("candidate reached confirmation")
quick_api.run_start.assert_not_called()
named = Mock()
named.plan_start.return_value = {"plan_path": "named.json",
                                 "plan": {"kind": "named_joint_target_plan",
                                          "already_at_target": False}}
named.run_start.return_value = {"report_path": "synthetic_named.json"}
named_result = cli.quick_dispatch(quick, named, is_tty=True,
                                  input_fn=lambda _: "开始")
assert named_result["result"]["report_path"] == "synthetic_named.json"
named.run_start.assert_called_once_with("named.json")

with patch.object(device, "connected", side_effect=AssertionError("device reached")):
    try: api.teach_preflight(20)
    except ValueError as exc: assert "未就绪" in str(exc)
    else: raise AssertionError("teach preflight accepted")

with tempfile.TemporaryDirectory() as directory:
    path = pathlib.Path(directory) / "plan.json"
    path.write_text(json.dumps(dict(candidate, already_at_target=False)))
    try: api.run_start(path)
    except ValueError as exc: assert "候选尚缺" in str(exc)
    else: raise AssertionError("candidate reached run")

assert not ({"pyAgxArm", "can"} & {name.split(".")[0] for name in sys.modules})
print("actual API/CLI candidate gates isolated")
'''
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root)
        result = subprocess.run(
            [sys.executable, "-B", "-S", "-c", code], cwd=root,
            env=environment, capture_output=True, text=True, check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("actual API/CLI candidate gates isolated", result.stdout)
        self.assertEqual(result.stderr, "")


if __name__ == "__main__":
    unittest.main()
