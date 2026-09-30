"""F8：惰性 Python 使用面、依赖方向、迁移表与回归白名单。"""

import ast
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

import nero_runtime


ROOT = Path(__file__).resolve().parents[1]
EXPECTED_EXPORTS = {
    "RobotSnapshot", "SourceQuality", "capture_waypoint", "save_waypoint",
    "load_waypoint", "list_waypoints", "RoutePoint", "JointRoute",
    "RouteParameters", "MotionResult", "StopEvidence", "prepare_joint_route",
    "RecordingParameters", "load_recording_summary", "list_recordings",
    "ReplayPlan", "ReplayResult", "prepare_replay", "replay_route",
    "PointAsset", "RouteAsset", "EnvironmentProfile", "load_environment_profile",
    "save_environment_profile", "EnvironmentIdentity", "OperationPlan",
    "OperationResult", "TeachReturnPlan", "TeachV1Result", "prepare_start",
    "prepare_park", "prepare_return_ready", "prepare_teach_return",
}
RUNNER_FILES = (
    "test_public_surface.py", "test_runtime_cli.py", "test_environment.py",
    "test_workflows.py", "test_replay.py", "test_teaching.py", "test_drag.py",
    "test_recording.py", "test_motion.py", "test_waypoints.py",
    "test_readonly_entry.py", "test_runtime_snapshot.py",
    "test_runtime_acquisition.py",
)


class PublicSurfaceTests(unittest.TestCase):
    def test_isolated_package_import_has_no_device_or_file_side_effect(self):
        script = '''
import os, sys
writes = []
def audit(event, args):
    if event == "open":
        mode, flags = args[1], args[2]
        if (isinstance(mode, str) and any(char in mode for char in "wax+")) or (isinstance(flags, int) and flags & (os.O_WRONLY | os.O_RDWR | os.O_CREAT)):
            writes.append((event, str(args[0])))
    elif event in ("os.mkdir", "os.remove", "os.rename", "os.link", "os.symlink"):
        writes.append((event, str(args[0])))
sys.addaudithook(audit)
import nero_runtime
assert not writes, writes
assert "RobotSnapshot" not in vars(nero_runtime)
assert not any(name == "can" or name.startswith(("can.", "pyAgxArm", "experiments.")) for name in sys.modules)
print("isolated package import ok")
'''
        with tempfile.TemporaryDirectory() as directory:
            env = os.environ.copy()
            env["PYTHONPATH"] = str(ROOT / "src")
            proc = subprocess.run([sys.executable, "-B", "-S", "-c", script],
                                  cwd=directory, env=env, capture_output=True, text=True)
            self.assertEqual(proc.returncode, 0, proc.stdout + proc.stderr)
            self.assertEqual(list(Path(directory).iterdir()), [])
            self.assertIn("isolated package import ok", proc.stdout)

    def test_explicit_public_exports_and_advanced_execution_boundary(self):
        self.assertEqual(set(nero_runtime.__all__), EXPECTED_EXPORTS)
        self.assertEqual(len(nero_runtime.__all__), len(EXPECTED_EXPORTS))
        self.assertIs(nero_runtime.RobotSnapshot,
                      __import__("nero_runtime.snapshot", fromlist=["RobotSnapshot"]).RobotSnapshot)
        self.assertIs(nero_runtime.prepare_start,
                      __import__("nero_runtime.workflows", fromlist=["prepare_start"]).prepare_start)
        for name in ("execute_joint_route", "execute_replay", "execute_start",
                     "enter_drag", "run_guided_recording", "connected_nero"):
            self.assertNotIn(name, nero_runtime.__all__)
            with self.assertRaises(AttributeError):
                getattr(nero_runtime, name)
        self.assertFalse(any(name == "can" or name.startswith("pyAgxArm")
                             for name in sys.modules))

    def test_runtime_imports_and_defaults_do_not_depend_on_experiments(self):
        for path in sorted((ROOT / "src" / "nero_runtime").glob("*.py")):
            with self.subTest(path=path.name):
                source = path.read_text(encoding="utf-8")
                tree = ast.parse(source)
                for node in ast.walk(tree):
                    if isinstance(node, ast.Import):
                        self.assertFalse(any(item.name == "experiments" or
                                             item.name.startswith("experiments.")
                                             for item in node.names), path)
                    elif isinstance(node, ast.ImportFrom) and node.module:
                        self.assertFalse(node.module == "experiments" or
                                         node.module.startswith("experiments."), path)
                self.assertNotIn("experiments/single_arm/", source, path.name)
                self.assertNotIn("/home/leo/nero_dh116_ws/experiments", source, path.name)

    def test_migration_table_covers_live_legacy_entries(self):
        document = (ROOT / "docs" / "legacy_runtime_migration.md").read_text()
        for path in (
            "lab/api.py", "lab/cli.py", "lab/planned_return.py",
            "lab/optimized_home.py", "lab/supported_home.py",
            "teaching/teach_session.py", "start_transfer/go_to_start.py",
            "continuous_replay/", "pose_recording/", "can_setup/",
            "control_interfaces/",
        ):
            self.assertIn("experiments/single_arm/" + path, document)
        for state in ("ACTIVE_EXPERIMENT", "RUNTIME_CONSUMER_CANDIDATE",
                      "HARDWARE_EVIDENCE_ONLY", "LOCKED"):
            self.assertIn(state, document)
        self.assertIn("删除旧代码 0 个", document)

    def test_runner_is_explicit_whitelist_and_never_discovers(self):
        runner = (ROOT / "scripts" / "test_runtime_offline.sh").read_text()
        self.assertIn("set -eu", runner)
        self.assertIn("PYTHONPATH=", runner)
        self.assertEqual(tuple(re.findall(r"^run_file (test_[a-z_]+\.py)$", runner,
                                          flags=re.MULTILINE)), RUNNER_FILES)
        self.assertNotIn("unittest discover", runner)
        self.assertNotIn("pytest", runner)
        self.assertNotIn("experiments/single_arm", runner)

    def test_current_state_and_readme_keep_field_boundary(self):
        current = (ROOT / "CURRENT_STATE.md").read_text()
        readme = (ROOT / "README.md").read_text()
        self.assertIn("UNKNOWN_SIDE_EFFECT", current)
        self.assertIn("Field Validation", current)
        self.assertIn("真实", current)
        self.assertIn("合成 Profile", readme)
        self.assertIn("不是现实站点批准", readme)
        self.assertIn("尚未完成 Field Validation", readme)


if __name__ == "__main__":
    unittest.main()
