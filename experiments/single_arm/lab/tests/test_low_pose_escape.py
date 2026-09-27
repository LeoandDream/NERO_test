"""本次低位离桌只允许已核对姿态，不能把任意姿态套进单轴目标。"""

import unittest

from experiments.single_arm.lab.low_pose_escape import plan_from_state
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class LowPoseEscapeTest(unittest.TestCase):
    def setUp(self):
        self.q=[.031, .223, .044, -.187, -.001, .206, -1.318]
        self.state={"ready_for_motion":True,"reasons":[],
                    "joint_rad":list(self.q),
                    "flange_pose_m_rad":[-.077,-.023,.715,0,0,0]}
        self.config=load_config(DEFAULT_CONFIG)

    def test_single_axis_route_reaches_positive_x(self):
        class Robot:
            def fk(self, q):
                return [-.077+(.223-q[1])*.58,-.023,.715,0,0,0]

        route=plan_from_state(Robot(),self.state,self.config)
        self.assertEqual(route["target_joint_rad"][1],-.2)
        self.assertEqual(route["target_joint_rad"][:1]+route["target_joint_rad"][2:],
                         self.q[:1]+self.q[2:])
        self.assertGreater(route["predicted_target_xyz_m"][0],.15)

    def test_other_low_pose_rejected(self):
        class Robot:
            def fk(self, q): return [-.077,-.023,.715,0,0,0]

        self.state["joint_rad"][3]=-.35
        with self.assertRaisesRegex(ValueError,"不是本次"):
            plan_from_state(Robot(),self.state,self.config)


if __name__ == "__main__":
    unittest.main()
