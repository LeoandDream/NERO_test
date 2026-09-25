"""独立事后分析的证据门槛；使用合成日志，不代表实机通过。"""

import csv
import json
from pathlib import Path
import tempfile
import unittest

from experiments.single_arm.continuous_replay.analyze_run import analyze, JOINT_COMMAND_IDS
from experiments.single_arm.continuous_replay.continuous_session import create_plan
from experiments.single_arm.continuous_replay.trajectory import DEFAULT_SOURCE
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


ROOT = Path(__file__).resolve().parents[4]


class IndependentAnalysisTests(unittest.TestCase):
    def test_complete_synthetic_log_passes_and_missing_can_group_fails(self):
        plan = create_plan(ROOT / DEFAULT_SOURCE, load_config(ROOT / DEFAULT_CONFIG),
                           until_source_s=1.8, turnaround_hold_s=1.0)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_path = root / "plan.json"
            plan_path.write_text(json.dumps(plan), encoding="utf-8")
            csv_path, can_path, report_path = (root / name for name in ("samples.csv", "can.csv", "report.json"))
            with csv_path.open("w", newline="", encoding="utf-8") as stream:
                fields = ("index", "phase", "actual_t_s", "period_s", "tracking_max_rad",
                          "flange_error_m", "arm_status", "motion_status", "error_code",
                          "joint_target_rad", "joint_feedback_rad")
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for index, item in enumerate(plan["waypoints"]):
                    writer.writerow({
                        "index": index, "phase": item["phase"],
                        "actual_t_s": item["t_s"],
                        "period_s": "" if index == 0 else "0.05",
                        "tracking_max_rad": 0, "flange_error_m": 0,
                        "arm_status": 0,
                        "motion_status": index % 2 if item["phase"] == "hold" else 0,
                        "error_code": 0,
                        "joint_target_rad": json.dumps(item["joint_rad"]),
                        "joint_feedback_rad": json.dumps(item["joint_rad"]),
                    })
            with can_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=("can_id",))
                writer.writeheader()
                for _ in plan["waypoints"]:
                    for can_id in JOINT_COMMAND_IDS:
                        writer.writerow({"can_id": can_id})
            report_path.write_text(json.dumps({
                "plan": str(plan_path), "samples_csv": str(csv_path),
                "can_trace_csv": str(can_path), "completed": True,
                "actual_start_joint_rad": plan["waypoints"][0]["joint_rad"],
                "driver_snapshots": [
                    {"drivers": [{"joint": joint, "voltage_v": 24, "enabled": True}
                                 for joint in range(1, 8)]} for _ in range(2)],
                "max_return_joint_error_rad": 0,
            }), encoding="utf-8")
            passed = analyze(report_path, ROOT / DEFAULT_CONFIG)
            self.assertTrue(passed["acceptance_pass"])
            self.assertTrue(passed["turnaround_hold_verified"])
            self.assertEqual(passed["logged_targets"], len(plan["waypoints"]))
            # 即使关节反馈完美，缺少一组实际 CAN 控制帧也不得验收。
            with can_path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=("can_id",))
                writer.writeheader()
                for _ in plan["waypoints"]:
                    for can_id in JOINT_COMMAND_IDS[:-1]:
                        writer.writerow({"can_id": can_id})
            failed = analyze(report_path, ROOT / DEFAULT_CONFIG)
            self.assertFalse(failed["acceptance_pass"])


if __name__ == "__main__":
    unittest.main()
