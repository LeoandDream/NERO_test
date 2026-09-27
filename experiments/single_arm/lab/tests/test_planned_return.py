"""新回位路线只依赖实时姿态和命名目标；这些测试不连接设备。"""

import unittest
from unittest.mock import patch
import json
import math
from pathlib import Path
import tempfile

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.lab.planned_return import plan_from_snapshot
from experiments.single_arm.lab.api import LabAPI
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class PlannedReturnTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(DEFAULT_CONFIG)
        cls.robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel="can0"))

    def _state(self, joints):
        return {"control_mode": 2, "arm_status": 0, "teach_status": 0,
                "error_code": 0, "joint_variation_0_25s_rad": 0.0,
                "drivers": [{"enabled": True, "undervoltage": False,
                             "driver_error": False} for _ in range(7)],
                "joint_rad": joints,
                "flange_pose_m_rad": list(self.robot.fk(joints))}

    def test_drag_endpoint_to_full_s1_without_recording_path(self):
        start = [0.516338, -0.269357, -0.752691, -0.564614,
                 0.760492, 0.205041, -0.828194]
        result = plan_from_snapshot(self.robot, self._state(start),
                                    self.config, 0.15)
        self.assertEqual(result["route_joint_rad"][0], start)
        self.assertEqual(result["route_joint_rad"][-1],
                         self.config["safe_start_joint_rad"])
        self.assertLessEqual(result["controller_targets"], 4)
        self.assertGreaterEqual(min(box["x"][0] for box in
                                    result["joint_box_flange_ranges_m"]), 0.15)

    def test_large_drag_endpoint_splits_only_long_controller_targets(self):
        # 实际 30 秒边界拖动末端含大幅 J3/J5 变化；不能把整段一次发给控制器。
        start = [0.09824458359476082, -0.6993708845666478,
                 -1.2041899174134878, -0.38997636806561303,
                 1.4498973695092494, -0.1956339558560444,
                 -0.5233893360880595]
        result = plan_from_snapshot(self.robot, self._state(start),self.config,.15)
        self.assertEqual(result["route_joint_rad"][-1],
                         self.config["safe_start_joint_rad"])
        self.assertLessEqual(result["controller_targets"], 6)
        self.assertTrue(all(math.dist(a,b) <= 1.0+1e-9
                            for a,b in zip(result["route_joint_rad"],
                                            result["route_joint_rad"][1:])))
        self.assertGreaterEqual(min(box["x"][0] for box in
                                    result["joint_box_flange_ranges_m"]), .15)

    def test_abnormal_state_has_no_route(self):
        state = self._state(self.config["safe_start_joint_rad"])
        state["arm_status"] = 1
        with self.assertRaisesRegex(RuntimeError, "整机状态 1"):
            plan_from_snapshot(self.robot, state, self.config)

    def test_unstable_return_state_reports_measured_variation(self):
        state = self._state(self.config["safe_start_joint_rad"])
        state["joint_variation_0_25s_rad"] = 0.003
        with self.assertRaisesRegex(RuntimeError, "0.003000 rad 超过 0.002 rad"):
            plan_from_snapshot(self.robot, state, self.config)

    def test_s1_needs_no_motion_target(self):
        result = plan_from_snapshot(
            self.robot, self._state(self.config["safe_start_joint_rad"]),
            self.config)
        self.assertEqual(result["controller_targets"], 0)
        self.assertEqual(result["strategy"], "already_at_s1")

    def test_floor_rejects_current_low_position(self):
        start = [0.516338, -0.269357, -0.752691, -0.564614,
                 0.760492, 0.205041, -0.828194]
        with self.assertRaises(ValueError):
            plan_from_snapshot(self.robot, self._state(start), self.config,
                               0.35)

    def test_h0_low_position_explains_dedicated_start(self):
        # 通用 S1 路线不能从桌边低位直接规划，也不能暗示放宽门槛。
        h0 = [0.23, 0.31084313978019007, -0.028082347664588763,
              -0.05707226654021458, 1.6599302916942469,
              0.13151055913777274, 0.592539281052075]
        with self.assertRaisesRegex(ValueError, "H0 专用启动入口"):
            plan_from_snapshot(self.robot, self._state(h0), self.config)

    def test_public_s1_init_uses_live_planner(self):
        api = LabAPI()
        expected = {"plan_path": "fresh.json", "plan": {"already_at_target": True}}
        with patch.object(api, "status", return_value=self._state(
                self.config["safe_start_joint_rad"])), patch.object(
            api, "plan_return", return_value=expected) as planned, patch(
            "experiments.single_arm.lab.api.starts.prepare_start"
        ) as archive:
            self.assertEqual(api.plan_start("S1"), expected)
        planned.assert_called_once()
        archive.assert_not_called()

    def test_public_s1_run_dispatches_live_plan(self):
        api = LabAPI()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "plan.json"
            path.write_text(json.dumps({"kind": "live_planned_return_to_s1"}))
            with patch.object(api, "run_return", return_value={"report_path": "run.json"}) as execute:
                self.assertEqual(api.run_start(path), {"report_path": "run.json"})
            execute.assert_called_once_with(path)


if __name__ == "__main__":
    unittest.main()
