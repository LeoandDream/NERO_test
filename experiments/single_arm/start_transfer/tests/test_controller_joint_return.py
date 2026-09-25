"""单目标回位的文件门禁测试；不会连接 CAN。"""

import json
from pathlib import Path
import tempfile
import unittest

from experiments.single_arm.start_transfer.controller_joint_return import validate_candidate
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


ROOT = Path("experiments/single_arm/start_transfer")
PLAN = ROOT / "data/plans/nero_two_target_offline_20260925T071016Z.json"
AUDIT = ROOT / "data/diagnostics/nero_mesh_audit_20260925T071353Z.json"


class CandidateChecks(unittest.TestCase):
    def setUp(self):
        self.config = load_config(DEFAULT_CONFIG)

    def test_current_offline_artifacts_pass_file_checks(self):
        start, middle, target = validate_candidate(PLAN, AUDIT, self.config)
        self.assertEqual(len(start), 7)
        self.assertAlmostEqual(middle[1], 0.2)
        self.assertEqual(target, self.config["safe_start_joint_rad"])

    def test_unrelated_mesh_audit_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "audit.json"
            audit = json.loads(AUDIT.read_text(encoding="utf-8"))
            audit["source_plan"] = "unrelated-plan.json"
            path.write_text(json.dumps(audit), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "另一份候选"):
                validate_candidate(PLAN, path, self.config)

    def test_line_only_audit_is_rejected_for_controller_motion(self):
        old_audit = ROOT / "data/diagnostics/nero_mesh_audit_20260925T071047Z.json"
        with self.assertRaisesRegex(ValueError, "关节不同步范围"):
            validate_candidate(PLAN, old_audit, self.config)

    def test_modified_intermediate_target_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            plan = json.loads(PLAN.read_text(encoding="utf-8"))
            plan["candidate_intermediate_joint_rad"][3] += 0.01
            path.write_text(json.dumps(plan), encoding="utf-8")
            audit_path = Path(directory) / "audit.json"
            audit = json.loads(AUDIT.read_text(encoding="utf-8"))
            audit["source_plan"] = str(path)
            audit_path.write_text(json.dumps(audit), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "中间目标"):
                validate_candidate(path, audit_path, self.config)


if __name__ == "__main__":
    unittest.main()
