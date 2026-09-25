"""回放路线的离线检查；不连接机械臂。"""

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from experiments.single_arm.replay.replay_session import build_cycle  # noqa: E402
from experiments.single_arm.teaching.return_session import load_recording  # noqa: E402
from experiments.single_arm.teaching.teach_session import load_config  # noqa: E402


class ReplayPlanningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / "experiments/single_arm/config/nero_teach.json")
        _, cls.metadata, cls.original, cls.recorded = load_recording(
            ROOT / "experiments/single_arm/teaching/data/recordings/nero_session_20260923T145952Z.csv",
            cls.config, allow_completed=True,
        )

    def test_completed_recording_can_form_exact_out_and_back_route(self):
        self.assertTrue(self.metadata["return_completed"])
        route, forward_steps = build_cycle(
            self.recorded, self.original, self.original, self.config
        )
        self.assertEqual(forward_steps, 22)
        self.assertEqual(len(route) - 1, 44)
        self.assertEqual(route[0], route[-1])
        self.assertEqual(route[forward_steps], self.recorded[-1])
        self.assertTrue(all(
            math.dist(a, b) <= 0.05 + 1e-9 for a, b in zip(route, route[1:])
        ))

    def test_unknown_start_or_incomplete_recording_is_rejected(self):
        shifted = list(self.original)
        shifted[0] += 0.03
        with self.assertRaisesRegex(ValueError, "未贴近"):
            build_cycle(self.recorded, shifted, self.original, self.config)
        with self.assertRaisesRegex(ValueError, "停稳轨迹"):
            load_recording(
                ROOT / "experiments/single_arm/teaching/data/recordings/nero_session_20260923T151241Z.csv",
                self.config, allow_completed=True,
            )


if __name__ == "__main__":
    unittest.main()
