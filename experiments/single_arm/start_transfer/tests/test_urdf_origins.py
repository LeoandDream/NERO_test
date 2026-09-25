"""验证固定版厂商 URDF 与本机 SDK 的 Nero 法兰坐标定义一致。"""

import json
import math
import unittest

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.start_transfer.geometry.urdf_origins import (
    link_origins, parse_joint_origins,
)
from experiments.single_arm.start_transfer.plan_two_target_s1 import DEFAULT_REPORT
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG


class UrdfOriginTests(unittest.TestCase):
    def test_link7_matches_sdk_flange_at_three_independent_poses(self):
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel="can0"
        ))
        model = parse_joint_origins()
        config = json.loads(DEFAULT_CONFIG.read_text(encoding="utf-8"))
        report = json.loads(DEFAULT_REPORT.read_text(encoding="utf-8"))
        cases = [
            [0.0] * 7,
            config["safe_start_joint_rad"],
            report["feedback_samples"][-1]["joint_rad"],
        ]
        for joints in cases:
            with self.subTest(joints=joints):
                origin = link_origins(joints, model)["link7"]
                self.assertLess(math.dist(origin, robot.fk(joints)[:3]), 1e-6)


if __name__ == "__main__":
    unittest.main()
