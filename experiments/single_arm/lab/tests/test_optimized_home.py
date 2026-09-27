"""Offline checks for the shorter searched S1/home joint route."""

import unittest
from unittest.mock import patch

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.lab.optimized_home import (
    DEFAULT_STRATEGY, HOME_Q, MAX_LOW_X_Y_M, REDUCED_J3_MIN_FLANGE_Z_M,
    _low_x_worst_y, _read_preflight_state, plan, plan_resume,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


class OptimizedHomeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = robot_instance("can0")
        cls.config = load_config(DEFAULT_CONFIG)

    def _state(self, q):
        return {"ready_for_motion": True, "reasons": [], "joint_rad": q,
                "flange_pose_m_rad": self.robot.fk(q)}

    def test_home_search_removes_excessive_joint_excursion(self):
        planned = plan(self.robot,
                       self._state(self.config["safe_start_joint_rad"]),
                       self.config, "home")
        route = planned["route_joint_rad"]
        self.assertLessEqual(len(route)-1, 7)
        self.assertLess(planned["search"]["joint_path_length_rad"], 4.7)
        self.assertLessEqual(_low_x_worst_y(self.robot, route[:-1]),
                             MAX_LOW_X_Y_M)
        self.assertLess(self.robot.fk(route[1])[1], -0.08)
        self.assertEqual(route[-1], HOME_Q)
        self.assertEqual(route[-2][1:], route[-1][1:])
        self.assertEqual(planned["strategy"], DEFAULT_STRATEGY)
        self.assertGreaterEqual(planned["search"]["j3_bend_rad"], -0.70)

    def test_start_releases_edge_with_only_j1(self):
        noisy = list(HOME_Q)
        noisy[4] += 0.0001
        planned = plan(self.robot, self._state(noisy), self.config, "start")
        route = planned["route_joint_rad"]
        self.assertEqual(route[0][1:], route[1][1:])
        self.assertLess(self.robot.fk(route[1])[1],
                        self.robot.fk(route[0])[1]-0.015)
        self.assertEqual(route[-1], self.config["safe_start_joint_rad"])

    def test_reduced_j3_candidate_preserves_low_corridor_and_height(self):
        old = plan(self.robot, self._state(HOME_Q), self.config, "start",
                   strategy="validated")
        new = plan(self.robot, self._state(HOME_Q), self.config, "start")
        route = new["route_joint_rad"]
        self.assertLess(abs(new["search"]["j3_bend_rad"]),
                        abs(old["search"]["j3_bend_rad"]))
        self.assertLess(new["search"]["cost"], old["search"]["cost"])
        self.assertLessEqual(_low_x_worst_y(self.robot, route[1:]),
                             MAX_LOW_X_Y_M)
        self.assertGreaterEqual(min(box["z"][0] for box in
                                    new["joint_box_flange_ranges_m"]),
                                REDUCED_J3_MIN_FLANGE_Z_M)
        self.assertEqual(route[0][1:], route[1][1:])
        self.assertEqual(route[-1], self.config["safe_start_joint_rad"])

    def test_paused_home_can_resume_only_from_last_reached_target(self):
        original = plan(self.robot,
                        self._state(self.config["safe_start_joint_rad"]),
                        self.config, "home")
        route = original["route_joint_rad"]
        source = {
            "kind": "optimized_sparse_s1_h0_trial", "direction": "home",
            "strategy": DEFAULT_STRATEGY, "completed": False,
            "emergency_stop_requested": False, "error_type": "RoutePaused",
            "error": "第 1 步未执行；机械臂仍停在上一小步附近",
            "route_joint_rad": route,
            "completed_targets": [
                {"index": i, "joint_rad": route[i]} for i in (1, 2)],
        }
        resumed = plan_resume(self.robot, self._state(route[2]),
                              self.config, source)
        self.assertEqual(resumed["resumed_after_target"], 2)
        self.assertEqual(resumed["route_joint_rad"][1:], route[3:])
        self.assertEqual(resumed["route_joint_rad"][-1], HOME_Q)
        paused_again = {**source,
                        "full_route_joint_rad": route,
                        "route_joint_rad": resumed["route_joint_rad"],
                        "resumed_after_target": 2,
                        "completed_targets": [{"index": 1,
                                               "joint_rad": route[3]}]}
        second_resume = plan_resume(self.robot, self._state(route[3]),
                                    self.config, paused_again)
        self.assertEqual(second_resume["resumed_after_target"], 3)
        self.assertEqual(second_resume["route_joint_rad"][1:], route[4:])
        displaced = list(route[2])
        displaced[0] += 0.01
        with self.assertRaisesRegex(ValueError, "实时姿态不在"):
            plan_resume(self.robot, self._state(displaced), self.config, source)
        source["route_joint_rad"][3][0] += 0.01
        with self.assertRaisesRegex(ValueError, "目标序列"):
            plan_resume(self.robot, self._state(route[2]), self.config, source)

    def test_transient_stability_reading_is_rechecked_before_motion(self):
        unstable = {"ready_for_motion": False,
                    "reasons": ["七轴反馈尚未停稳"],
                    "joint_rad": list(HOME_Q),
                    "joint_variation_0_25s_rad": 0.003}
        stable = {**unstable, "ready_for_motion": True, "reasons": [],
                  "joint_variation_0_25s_rad": 0.0}
        report = {}
        with patch("experiments.single_arm.lab.optimized_home.read_state",
                   side_effect=[unstable, stable]) as read, patch(
                   "experiments.single_arm.lab.optimized_home.time.sleep"):
            result = _read_preflight_state(self.robot, report, label="initial")
        self.assertIs(result, stable)
        self.assertEqual(read.call_count, 2)
        self.assertEqual([item["variation_rad"] for item in
                          report["preflight_state_attempts"]], [0.003, 0.0])

    def test_stability_retry_rejects_real_pose_change(self):
        unstable = {"ready_for_motion": False,
                    "reasons": ["七轴反馈尚未停稳"],
                    "joint_rad": list(HOME_Q),
                    "joint_variation_0_25s_rad": 0.003}
        moved = {**unstable, "ready_for_motion": True, "reasons": [],
                 "joint_variation_0_25s_rad": 0.0,
                 "joint_rad": [HOME_Q[0] + 0.004, *HOME_Q[1:]]}
        with patch("experiments.single_arm.lab.optimized_home.read_state",
                   side_effect=[unstable, moved]), patch(
                   "experiments.single_arm.lab.optimized_home.time.sleep"):
            with self.assertRaisesRegex(RuntimeError, "姿态变化超过"):
                _read_preflight_state(self.robot, {}, label="initial")

    def test_stability_retry_does_not_mask_other_faults(self):
        fault = {"ready_for_motion": False, "reasons": ["七轴中存在失能、欠压或驱动故障"],
                 "joint_rad": list(HOME_Q), "joint_variation_0_25s_rad": 0.0}
        with patch("experiments.single_arm.lab.optimized_home.read_state",
                   return_value=fault) as read:
            self.assertIs(_read_preflight_state(self.robot, {}, label="initial"), fault)
        read.assert_called_once()


if __name__ == "__main__":
    unittest.main()
