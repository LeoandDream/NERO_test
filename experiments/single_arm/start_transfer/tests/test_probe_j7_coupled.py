"""微小 J4/J7 诊断路线的离线门槛测试。"""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from experiments.single_arm.start_transfer.probe_j7_coupled import plan_probe  # noqa: E402
from experiments.single_arm.teaching.teach_session import load_config  # noqa: E402


class CoupledProbeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        cls.recorded = [
            0.2239431963233924, 0.07812093731926618, 0.06729989595690135,
            -0.08246680715673207, 0.010838494654884786, 0.11309733552923257,
            0.4490383099531011,
        ]

    def test_only_j4_and_j7_change_by_one_hundredth_radian(self):
        start, target = plan_probe(self.recorded, self.config)
        self.assertEqual(start, self.recorded)
        self.assertAlmostEqual(target[3] - start[3], -0.01)
        self.assertAlmostEqual(target[6] - start[6], -0.01)
        self.assertTrue(all(target[i] == start[i] for i in (0, 1, 2, 4, 5)))

    def test_rejects_unexpected_start_or_failed_joint_limits(self):
        shifted = list(self.recorded)
        shifted[3] = -0.3
        with self.assertRaisesRegex(ValueError, "起点范围"):
            plan_probe(shifted, self.config)


if __name__ == "__main__":
    unittest.main()
