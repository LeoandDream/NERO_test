"""F1：真实采集/分派/存取函数在合成设备生命周期下的离线接线。"""

from contextlib import redirect_stderr, redirect_stdout
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

from nero_runtime.cli import main
from nero_runtime.device import connected_nero
from nero_runtime.waypoints import load_waypoint


class FakeRobot:
    def __init__(self, *, fail=None):
        self.calls = []
        self.fail = fail
        self.connected = False

    def _call(self, name, value):
        self.calls.append(name)
        if self.fail == name:
            raise OSError(f"synthetic {name} failure")
        if self.fail == "interrupt" and name == "get_joint_angles":
            raise KeyboardInterrupt
        return value

    def connect(self):
        self.calls.append("connect")
        if self.fail == "connect":
            raise OSError("synthetic connect failure")
        if self.fail in ("connect_runtime", "connect_runtime_cleanup"):
            raise RuntimeError("Failed to establish robot communication.")
        if self.fail == "connect_bug":
            raise RuntimeError("synthetic implementation bug")
        self.connected = True

    def disconnect(self):
        self.calls.append("disconnect")
        self.connected = False
        if self.fail == "disconnect" or self.fail == "connect_runtime_cleanup":
            raise OSError("synthetic disconnect failure")

    def get_arm_status(self):
        return self._call("get_arm_status", SimpleNamespace(
            timestamp=99.9,
            msg=SimpleNamespace(ctrl_mode=2, arm_status=6, teach_status=2,
                                motion_status=0, err_code=9)))

    def get_joint_angles(self):
        return self._call("get_joint_angles", SimpleNamespace(
            timestamp=99.8, msg=[0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]))

    def get_flange_pose(self):
        return self._call("get_flange_pose", SimpleNamespace(
            timestamp=99.7, msg=[-0.18, 0.01, 0.7, 0, 0, 0]))

    def get_driver_states(self, joint):
        return self._call(f"get_driver_states({joint})", SimpleNamespace(
            timestamp=99.6,
            msg=SimpleNamespace(vol=24.0,
                                foc_status=SimpleNamespace(
                                    driver_enable_status=False,
                                    voltage_too_low=False,
                                    driver_error_status=False))))

    def __getattr__(self, name):
        if name in {"enable", "disable", "reset", "hold", "move_j", "move_l",
                    "move_p", "set_motion_mode", "electronic_emergency_stop"}:
            raise AssertionError(f"control method reached: {name}")
        raise AttributeError(name)


class Clock:
    def __init__(self):
        self.values = iter((100.0, 100.05))

    def __call__(self):
        return next(self.values)


class ReadonlyEntryTests(unittest.TestCase):
    def source(self, robot):
        configurations = []

        def config_factory(**kwargs):
            configurations.append(kwargs)
            return kwargs

        def opener(channel):
            return connected_nero(channel, config_factory=config_factory,
                                  driver_factory=lambda config: robot)

        return opener, configurations

    def call(self, argv, *, opener=None, clock=None, monotonic=None, sleep=None):
        stdout, stderr = io.StringIO(), io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            code = main(argv, source_factory=opener,
                        clock_s=Clock() if clock is None else clock,
                        **({"monotonic_s": monotonic} if monotonic is not None else {}),
                        **({"sleep_s": sleep} if sleep is not None else {}),
                        source_label="synthetic SDK-shaped source")
        return code, stdout.getvalue(), stderr.getvalue()

    def test_status_and_capture_full_chain_without_control(self):
        status_robot = FakeRobot()
        opener, configs = self.source(status_robot)
        code, out, err = self.call(["status", "--robot-id", "synthetic-nero"],
                                   opener=opener)
        self.assertEqual((code, err), (0, ""))
        self.assertIn("J1=0.100000", out)
        self.assertIn("-0.180000", out)
        self.assertIn("6 / 9", out)
        self.assertIn("非运动许可", out)
        self.assertEqual(status_robot.calls[0], "connect")
        self.assertEqual(status_robot.calls[-1], "disconnect")
        self.assertFalse(status_robot.connected)
        self.assertEqual(configs[0]["firmeware_version"], "v121")
        self.assertFalse(configs[0]["auto_connect"])
        self.assertFalse(configs[0]["enable_check_can"])

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "named.json"
            capture_robot = FakeRobot()
            capture_opener, _ = self.source(capture_robot)
            code, out, err = self.call([
                "waypoint", "capture", "--name", "READY", "--output", str(path),
                "--robot-id", "synthetic-nero"], opener=capture_opener)
            self.assertEqual((code, err), (0, ""))
            self.assertIn("点位名：READY", out)
            self.assertIn(str(path), out)
            record = load_waypoint(path)
            self.assertEqual(record["snapshot"]["joint_rad"][1], -0.2)
            self.assertEqual(record["snapshot"]["joints_quality"]["timestamp_s"], 99.8)
            self.assertEqual(record["snapshot"]["arm_status"], 6)
            self.assertEqual(record["snapshot"]["error_code"], 9)
            self.assertEqual(record["captured_at_s"], 100.05)
            self.assertEqual(record["read_errors"], [])
            self.assertEqual(capture_robot.calls[-1], "disconnect")
            self.assertFalse(capture_robot.connected)

            def forbidden_opener(_channel):
                raise AssertionError("show/list attempted device source")

            code, shown, err = self.call(["waypoint", "show", str(path)],
                                          opener=forbidden_opener)
            self.assertEqual((code, err), (0, ""))
            self.assertIn("J7=0.700000", shown)
            self.assertIn("缓存", shown)
            code, listed, err = self.call(["waypoint", "list", directory],
                                           opener=forbidden_opener)
            self.assertEqual((code, err), (0, ""))
            self.assertIn("READY", listed)
            self.assertIn(str(path), listed)
            self.assertNotIn("pyAgxArm", sys.modules)
            self.assertNotIn("can", sys.modules)

    def test_validation_and_existing_output_stop_before_factory(self):
        def forbidden_opener(_channel):
            raise AssertionError("invalid request connected device")

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "point.json"
            path.write_text("original", encoding="utf-8")
            cases = (
                ["waypoint", "capture", "--name", "  ", "--output", str(path)],
                ["waypoint", "capture", "--name", "x", "--output", str(path)],
                ["status", "--max-age-seconds", "nan"],
                ["status", "--channel", "bad channel"],
            )
            for argv in cases:
                with self.subTest(argv=argv):
                    code, _, err = self.call(argv, opener=forbidden_opener)
                    self.assertEqual(code, 1)
                    self.assertIn("未完成", err)
            self.assertEqual(path.read_text(encoding="utf-8"), "original")

    def test_read_failure_interrupt_and_connect_failure_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "missing.json"
            failed = FakeRobot(fail="get_joint_angles")
            opener, _ = self.source(failed)
            code, _, err = self.call([
                "waypoint", "capture", "--name", "x", "--output", str(path),
                "--wait-seconds", "0"], opener=opener)
            self.assertEqual(code, 1)
            self.assertIn("seven finite joint angles", err)
            self.assertFalse(path.exists())
            self.assertEqual(failed.calls[-1], "disconnect")

            interrupted = FakeRobot(fail="interrupt")
            opener, _ = self.source(interrupted)
            code, _, err = self.call(["status"], opener=opener)
            self.assertEqual(code, 130)
            self.assertIn("已取消", err)
            self.assertEqual(interrupted.calls[-1], "disconnect")

            unconnected = FakeRobot(fail="connect")
            opener, _ = self.source(unconnected)
            code, _, err = self.call(["status"], opener=opener)
            self.assertEqual(code, 1)
            self.assertIn("synthetic connect failure", err)
            self.assertEqual(unconnected.calls, ["connect", "disconnect"])

    def test_status_keeps_missing_data_and_read_error_diagnostic(self):
        source = FakeRobot(fail="get_joint_angles")
        opener, _ = self.source(source)
        code, out, err = self.call(["status", "--wait-seconds", "0"], opener=opener)
        self.assertEqual((code, err), (0, ""))
        self.assertIn("七轴角：缺失或无效", out)
        self.assertIn("synthetic get_joint_angles failure", out)
        self.assertEqual(source.calls[-1], "disconnect")

    def test_status_waits_for_first_valid_feedback_without_health_gate(self):
        class LateRobot(FakeRobot):
            rounds = 0

            def get_arm_status(self):
                self.rounds += 1
                if self.rounds == 1:
                    return self._call("get_arm_status", None)
                return super().get_arm_status()

            def get_joint_angles(self):
                if self.rounds == 1:
                    return self._call("get_joint_angles", None)
                return super().get_joint_angles()

        robot = LateRobot()
        opener, _ = self.source(robot)
        now = [0.0]
        sleeps = []

        def sleep(seconds):
            sleeps.append(seconds)
            now[0] += seconds

        code, out, err = self.call(
            ["status", "--wait-seconds", "0.2"], opener=opener,
            clock=lambda: 100.0, monotonic=lambda: now[0], sleep=sleep)
        self.assertEqual((code, err), (0, ""))
        self.assertEqual(robot.rounds, 2)
        self.assertEqual(len(sleeps), 1)
        self.assertIn("J1=0.100000", out)
        self.assertIn("6 / 9", out)  # 故障状态也立即显示。
        self.assertIn("-0.180000", out)

    def test_status_finite_exit_for_no_data_and_partial_sources(self):
        class MissingRobot(FakeRobot):
            def get_arm_status(self):
                return self._call("get_arm_status", None)

            def get_joint_angles(self):
                return self._call("get_joint_angles", None)

        for robot, expected in ((MissingRobot(), "控制器、七轴"),
                                (FakeRobot(fail="get_joint_angles"), "七轴")):
            with self.subTest(expected=expected):
                opener, _ = self.source(robot)
                now = [0.0]
                def sleep(seconds):
                    now[0] += seconds
                code, out, err = self.call(
                    ["status", "--wait-seconds", "0.11"], opener=opener,
                    clock=lambda: 100.0, monotonic=lambda: now[0], sleep=sleep)
                self.assertEqual((code, err), (0, ""))
                self.assertIn("等候截止，必需反馈未齐：" + expected, out)
                self.assertIn("J1 驱动：", out)
                self.assertLessEqual(now[0], 0.11 + 1e-9)
                self.assertEqual(robot.calls[-1], "disconnect")

        optional_missing = FakeRobot(fail="get_flange_pose")
        opener, _ = self.source(optional_missing)
        def should_not_sleep(_seconds):
            raise AssertionError("optional flange was treated as required")
        code, out, err = self.call(
            ["status"], opener=opener, sleep=should_not_sleep)
        self.assertEqual((code, err), (0, ""))
        self.assertIn("法兰质量：missing", out)
        self.assertNotIn("等候截止", out)

    def test_capture_wait_and_cancel_during_wait(self):
        class LateJoints(FakeRobot):
            reads = 0
            def get_joint_angles(self):
                self.reads += 1
                if self.reads == 1:
                    return self._call("get_joint_angles", None)
                return super().get_joint_angles()

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "late.json"
            robot = LateJoints()
            opener, _ = self.source(robot)
            now = [0.0]
            def sleep(seconds):
                now[0] += seconds
            code, _, err = self.call(
                ["waypoint", "capture", "--name", "late", "--output", str(path)],
                opener=opener, clock=lambda: 100.0,
                monotonic=lambda: now[0], sleep=sleep)
            self.assertEqual((code, err), (0, ""))
            self.assertEqual(robot.reads, 2)
            self.assertTrue(path.exists())

            cancelled = LateJoints()
            opener, _ = self.source(cancelled)
            def interrupt(_seconds):
                raise KeyboardInterrupt
            other = Path(directory) / "cancelled.json"
            code, _, err = self.call(
                ["waypoint", "capture", "--name", "cancelled", "--output", str(other)],
                opener=opener, clock=lambda: 100.0,
                monotonic=lambda: 0.0, sleep=interrupt)
            self.assertEqual(code, 130)
            self.assertIn("已取消", err)
            self.assertFalse(other.exists())
            self.assertEqual(cancelled.calls[-1], "disconnect")

    def test_driver_flags_are_visible_and_unknown_stays_unknown(self):
        class AllEnabled(FakeRobot):
            def get_driver_states(self, joint):
                value = super().get_driver_states(joint)
                value.msg.foc_status.driver_enable_status = True
                return value

        base = AllEnabled()
        opener, _ = self.source(base)
        code, healthy, _ = self.call(["status"], opener=opener)
        self.assertEqual(code, 0)

        class J4Fault(AllEnabled):
            def get_driver_states(self, joint):
                if joint != 4:
                    return super().get_driver_states(joint)
                return self._call("get_driver_states(4)", SimpleNamespace(
                    timestamp=99.6, msg=SimpleNamespace(
                        vol=24.0, foc_status=SimpleNamespace(
                            driver_enable_status=False, voltage_too_low=True,
                            driver_error_status=True))))

        changed = J4Fault()
        opener, _ = self.source(changed)
        code, fault, _ = self.call(["status"], opener=opener)
        self.assertEqual(code, 0)
        self.assertNotEqual(healthy, fault)
        self.assertIn("J4 驱动：enabled=否；undervoltage=是；driver_error=是", fault)

        unknown = FakeRobot(fail="get_driver_states(4)")
        opener, _ = self.source(unknown)
        code, out, _ = self.call(["status"], opener=opener)
        self.assertEqual(code, 0)
        self.assertIn("J4 驱动：enabled=未知；undervoltage=未知；driver_error=未知", out)
        self.assertNotIn("ready_for_motion", out)

    def test_connection_close_and_primary_error_reporting(self):
        runtime = FakeRobot(fail="connect_runtime")
        opener, _ = self.source(runtime)
        code, _, err = self.call(["status"], opener=opener)
        self.assertEqual(code, 1)
        self.assertIn("连接失败：RuntimeError: Failed to establish robot communication.", err)
        self.assertEqual(runtime.calls, ["connect", "disconnect"])

        runtime_and_close = FakeRobot(fail="connect_runtime_cleanup")
        opener, _ = self.source(runtime_and_close)
        code, _, err = self.call(["status"], opener=opener)
        self.assertEqual(code, 1)
        self.assertIn("Failed to establish robot communication.", err)
        self.assertIn("关闭通信失败：OSError: synthetic disconnect failure", err)

        bug = FakeRobot(fail="connect_bug")
        opener, _ = self.source(bug)
        with self.assertRaisesRegex(RuntimeError, "synthetic implementation bug"):
            self.call(["status"], opener=opener)
        self.assertEqual(bug.calls, ["connect", "disconnect"])

        closed = FakeRobot(fail="disconnect")
        opener, _ = self.source(closed)
        code, _, err = self.call(["status"], opener=opener)
        self.assertEqual(code, 1)
        self.assertIn("关闭通信失败：OSError: synthetic disconnect failure", err)

        mixed = FakeRobot(fail="disconnect")
        opener, _ = self.source(mixed)
        def primary():
            raise OSError("synthetic primary failure")
        code, _, err = self.call(["status"], opener=opener, clock=primary)
        self.assertEqual(code, 1)
        self.assertIn("读取失败：OSError: synthetic primary failure", err)
        self.assertIn("关闭通信失败：OSError: synthetic disconnect failure", err)

        def invalid_clock():
            raise ValueError("synthetic primary value error")
        code, _, err = self.call(["status"], opener=opener, clock=invalid_clock)
        self.assertEqual(code, 1)
        self.assertIn("synthetic primary value error", err)
        self.assertIn("关闭通信失败：OSError: synthetic disconnect failure", err)

        cancelled = FakeRobot(fail="disconnect")
        opener, _ = self.source(cancelled)
        def interrupt():
            raise KeyboardInterrupt
        code, _, err = self.call(["status"], opener=opener, clock=interrupt)
        self.assertEqual(code, 130)
        self.assertIn("已取消", err)
        self.assertIn("关闭通信失败：OSError: synthetic disconnect failure", err)

    def test_import_and_help_do_not_load_sdk(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        result = subprocess.run(
            [sys.executable, "-B", "-S", "-c",
             "import sys; import nero_runtime.cli; "
             "assert 'pyAgxArm' not in sys.modules; "
             "assert 'can' not in sys.modules"],
            cwd=root, env=environment, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        result = subprocess.run(
            [sys.executable, "-B", "-S", "-m", "nero_runtime.cli", "--help"],
            cwd=root, env=environment, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("waypoint", result.stdout)


if __name__ == "__main__":
    unittest.main()
