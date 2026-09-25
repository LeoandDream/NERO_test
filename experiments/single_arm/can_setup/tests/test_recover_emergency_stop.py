"""复位前的关键状态门禁测试，不连接真实机械臂。"""

import unittest

from experiments.single_arm.can_setup.recover_emergency_stop import check_before_reset


class RecoveryGuardTests(unittest.TestCase):
    def setUp(self):
        self.stopped = {
            "control_mode": 1,
            "arm_status": 1,
            "teach_status": 0,
            "joint_error_code": 0,
            "joint_rad": [0.1, 0.2, -0.3, -0.4, 0.0, 0.1, -0.1],
        }

    def test_only_stable_emergency_stop_can_proceed(self):
        later = {**self.stopped, "joint_rad": [0.1005, 0.2, -0.3, -0.4, 0, 0.1, -0.1]}
        self.assertAlmostEqual(check_before_reset(self.stopped, later), 0.0005)

    def test_status_change_or_motion_rejects_reset(self):
        normal = {**self.stopped, "arm_status": 0}
        moving = {**self.stopped, "joint_rad": [0.11, 0.2, -0.3, -0.4, 0, 0.1, -0.1]}
        with self.assertRaisesRegex(RuntimeError, "拒绝复位"):
            check_before_reset(self.stopped, normal)
        with self.assertRaisesRegex(RuntimeError, "仍在移动"):
            check_before_reset(self.stopped, moving)


if __name__ == "__main__":
    unittest.main()
