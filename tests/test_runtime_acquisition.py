"""仅以合成 SDK 形状的内存反馈验证有限采集与现有纯解析器。"""

from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import unittest

from nero_runtime.acquisition import collect_snapshot


class FakeSource:
    """只返回内存对象；每次 getter 调用都有可核对记录。"""

    def __init__(self):
        self.calls = []
        self.fail_at = {}
        self.status = SimpleNamespace(
            timestamp=99.9,
            msg=SimpleNamespace(ctrl_mode=1, arm_status=0, teach_status=6,
                                motion_status=0, err_code=0),
        )
        self.joints = SimpleNamespace(
            timestamp=99.8, msg=[0.12, -0.04, 0.2, -0.3, 0.01, 0.1, 0.06],
        )
        self.flange = SimpleNamespace(
            timestamp=99.7, msg=[-0.182, 0.01, 0.73, 0.0, -0.1, 0.2],
        )
        self.drivers = {
            i: SimpleNamespace(
                timestamp=99.6,
                msg=SimpleNamespace(vol=24.0 + i, foc_status=SimpleNamespace(
                    driver_enable_status=True, voltage_too_low=False,
                    driver_error_status=False,
                )),
            ) for i in range(1, 8)
        }

    def _get(self, label, value):
        self.calls.append(label)
        if label in self.fail_at:
            raise self.fail_at[label]
        return value

    def get_arm_status(self):
        return self._get("get_arm_status", self.status)

    def get_joint_angles(self):
        return self._get("get_joint_angles", self.joints)

    def get_flange_pose(self):
        return self._get("get_flange_pose", self.flange)

    def get_driver_states(self, joint_index):
        return self._get(f"get_driver_states({joint_index})",
                         self.drivers.get(joint_index))

    def __getattr__(self, name):
        if name in {"connect", "disconnect", "enable", "disable", "reset",
                    "move_j", "move_l", "move_p", "set_motion_mode",
                    "set_speed_percent", "electronic_emergency_stop", "hold"}:
            raise AssertionError(f"forbidden control/lifecycle method: {name}")
        raise AttributeError(name)


class Clock:
    def __init__(self, *values):
        self.values = iter(values)
        self.calls = 0

    def __call__(self):
        self.calls += 1
        return next(self.values)


def collect(source=None, clock=None, **kwargs):
    source = FakeSource() if source is None else source
    clock = Clock(100.0, 100.05) if clock is None else clock
    result = collect_snapshot(
        source, robot_id=kwargs.pop("robot_id", "synthetic-arm"),
        clock_s=clock, time_basis=kwargs.pop("time_basis", "synthetic_seconds"),
        max_status_age_s=kwargs.pop("max_status_age_s", 0.25),
        max_joints_age_s=kwargs.pop("max_joints_age_s", 0.3),
        max_flange_age_s=kwargs.pop("max_flange_age_s", 0.4),
        max_driver_age_s=kwargs.pop("max_driver_age_s", 0.5),
        **kwargs,
    )
    return source, clock, result


class AcquisitionTests(unittest.TestCase):
    def test_complete_mapping_order_and_exact_read_counts(self):
        source, clock, result = collect()
        expected = ["get_arm_status", "get_joint_angles", "get_flange_pose"]
        expected += [f"get_driver_states({i})" for i in range(1, 8)]
        self.assertEqual(source.calls, expected)
        self.assertEqual(clock.calls, 2)
        self.assertEqual((result.started_at_s, result.ended_at_s),
                         (100.0, 100.05))
        snap = result.snapshot
        self.assertEqual(snap.observed_at_s, 100.05)
        self.assertEqual(snap.time_basis, "synthetic_seconds")
        self.assertEqual((snap.control_mode, snap.arm_status, snap.teach_status,
                          snap.motion_status, snap.error_code), (1, 0, 6, 0, 0))
        self.assertEqual(snap.joint_rad, tuple(source.joints.msg))
        self.assertEqual(snap.flange_pose_m_rad, tuple(source.flange.msg))
        self.assertEqual([item.joint for item in snap.drivers], list(range(1, 8)))
        self.assertEqual([item.voltage_v for item in snap.drivers],
                         [24.0 + i for i in range(1, 8)])
        self.assertTrue(all(item.quality.state == "valid" for item in snap.drivers))
        self.assertEqual(result.read_errors, ())
        json.dumps(asdict(result), allow_nan=False)

    def test_missing_status_and_driver_keep_specific_unknowns(self):
        source = FakeSource()
        source.status = None
        source.drivers[4] = None
        _, _, result = collect(source)
        snap = result.snapshot
        self.assertEqual(snap.status_quality.state, "missing")
        self.assertIsNone(snap.error_code)
        self.assertEqual(snap.drivers[3].quality.state, "missing")
        self.assertIsNone(snap.drivers[3].enabled)
        self.assertEqual(snap.joints_quality.state, "valid")
        self.assertEqual(len(source.calls), 10)

    def test_stale_future_and_malformed_values_are_not_refreshed(self):
        source = FakeSource()
        source.status.timestamp = 98.0
        source.joints.msg[2] = math.nan
        source.flange.timestamp = 100.2
        _, _, result = collect(source)
        snap = result.snapshot
        self.assertEqual(snap.status_quality.state, "stale")
        self.assertEqual(snap.status_quality.timestamp_s, 98.0)
        self.assertIsNone(snap.arm_status)
        self.assertEqual(snap.joints_quality.state, "invalid")
        self.assertIsNone(snap.joint_rad)
        self.assertEqual(snap.flange_quality.state, "invalid")
        self.assertIn("flange: timestamp_s is in the future", snap.issues)
        self.assertEqual(snap.drivers[0].quality.timestamp_s, 99.6)
        json.dumps(asdict(result), allow_nan=False)

    def test_fault_disabled_and_negative_flange_are_facts_not_policy(self):
        source = FakeSource()
        source.status.msg.arm_status = 1  # synthetic emergency-stop status
        source.status.msg.err_code = 4097
        source.drivers[2].msg.foc_status.driver_enable_status = False
        source.drivers[2].msg.foc_status.driver_error_status = True
        _, _, result = collect(source)
        snap = result.snapshot
        self.assertEqual(snap.status_quality.state, "valid")
        self.assertEqual((snap.arm_status, snap.error_code), (1, 4097))
        self.assertIs(snap.drivers[1].enabled, False)
        self.assertIs(snap.drivers[1].driver_error, True)
        self.assertEqual(snap.flange_pose_m_rad[0], -0.182)
        self.assertFalse(hasattr(snap, "ready_for_motion"))
        self.assertFalse(hasattr(result, "safe_to_move"))
        self.assertEqual(len(source.calls), 10)

    def test_expected_read_error_keeps_entry_type_and_missing_source(self):
        source = FakeSource()
        source.fail_at["get_joint_angles"] = TimeoutError("synthetic timeout")
        source.fail_at["get_driver_states(3)"] = OSError("synthetic I/O")
        _, _, result = collect(source)
        self.assertEqual(len(source.calls), 10)  # no retries; later reads continue
        self.assertEqual(result.snapshot.joints_quality.state, "missing")
        self.assertEqual(result.snapshot.drivers[2].quality.state, "missing")
        self.assertIsNone(result.snapshot.drivers[2].enabled)
        self.assertEqual(result.read_errors, (
            "get_joint_angles: TimeoutError: synthetic timeout",
            "get_driver_states(3): OSError: synthetic I/O",
        ))

    def test_unexpected_programming_error_is_not_disguised_as_feedback(self):
        source = FakeSource()
        source.fail_at["get_joint_angles"] = ValueError("synthetic implementation bug")
        with self.assertRaisesRegex(ValueError, "implementation bug"):
            collect(source)
        self.assertEqual(source.calls, ["get_arm_status", "get_joint_angles"])

    def test_interrupt_is_not_swallowed_or_followed_by_more_reads(self):
        source = FakeSource()
        source.fail_at["get_flange_pose"] = KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt):
            collect(source)
        self.assertEqual(source.calls,
                         ["get_arm_status", "get_joint_angles", "get_flange_pose"])

    def test_source_mutation_after_return_does_not_change_snapshot(self):
        source = FakeSource()
        _, _, result = collect(source)
        source.status.msg.err_code = 999
        source.joints.msg[0] = 999.0
        source.flange.msg[0] = 999.0
        source.drivers[1].msg.foc_status.driver_enable_status = False
        self.assertEqual(result.snapshot.error_code, 0)
        self.assertEqual(result.snapshot.joint_rad[0], 0.12)
        self.assertEqual(result.snapshot.flange_pose_m_rad[0], -0.182)
        self.assertIs(result.snapshot.drivers[0].enabled, True)

    def test_distinct_source_times_and_read_window_remain_distinct(self):
        _, _, result = collect(clock=Clock(100.0, 100.05))
        snap = result.snapshot
        self.assertEqual((result.started_at_s, result.ended_at_s),
                         (100.0, 100.05))
        self.assertEqual((snap.status_quality.timestamp_s,
                          snap.joints_quality.timestamp_s,
                          snap.flange_quality.timestamp_s,
                          snap.drivers[0].quality.timestamp_s),
                         (99.9, 99.8, 99.7, 99.6))
        self.assertFalse(hasattr(result, "synchronized"))

    def test_clock_error_before_read_and_backwards_clock(self):
        source = FakeSource()
        with self.assertRaises(ValueError):
            collect(source, Clock(math.nan, 100.0))
        self.assertEqual(source.calls, [])
        with self.assertRaisesRegex(ValueError, "backwards"):
            collect(source, Clock(100.0, 99.0))
        self.assertEqual(len(source.calls), 10)

    def test_broken_source_contract_propagates_without_claiming_success(self):
        class MissingGetter:
            def get_arm_status(self):
                return None

        with self.assertRaises(AttributeError):
            collect(MissingGetter())

    def test_isolated_import_needs_no_sdk_can_or_experiments(self):
        root = Path(__file__).resolve().parents[1]
        environment = os.environ.copy()
        environment["PYTHONPATH"] = str(root / "src")
        code = (
            "import sys; import nero_runtime; import nero_runtime.snapshot; "
            "import nero_runtime.acquisition; "
            "assert not ({'pyAgxArm', 'can', 'experiments'} & "
            "{name.split('.')[0] for name in sys.modules}); "
            "print('isolated import ok')"
        )
        run = subprocess.run(
            [sys.executable, "-B", "-S", "-c", code], cwd=root,
            env=environment, capture_output=True, text=True, check=False,
        )
        self.assertEqual(run.returncode, 0, run.stderr)
        self.assertEqual(run.stdout.strip(), "isolated import ok")
        self.assertEqual(run.stderr, "")


if __name__ == "__main__":
    unittest.main()
