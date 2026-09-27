"""One measured move_j from the contacting H0 pose to a demonstrated outboard pose.

This is an incident recovery experiment, not approval of the H0 parking route.
The target is sample 359 of the confirmed 2026-09-26 hand guided record.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from experiments.single_arm.lab.device import connected, read_state
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints,
)


HOME_FILE = Path("experiments/single_arm/lab/config/supported_home.json")
RECORDING = Path("experiments/single_arm/lab/data/home_demos/recordings/"
                 "nero_poses_lab_20260926T093258489363Z.csv")
RECORDING_SHA256 = "2d633392ea49f64f9ccdd4b69b7155363de718eefeb3401deb5a80e0cf61cc82"
SAMPLE_INDEX = 359
RUN_DIR = Path("experiments/single_arm/lab/data/edge_releases")


def demonstrated_target(config):
    if hashlib.sha256(RECORDING.read_bytes()).hexdigest() != RECORDING_SHA256:
        raise ValueError("外侧目标的原始示教 CSV 校验失败")
    with RECORDING.open(newline="", encoding="utf-8") as stream:
        row = next((item for item in csv.DictReader(stream)
                    if int(item["sample_index"]) == SAMPLE_INDEX), None)
    if row is None or abs(float(row["elapsed_s"]) - 18) > 0.02:
        raise ValueError("外侧目标样本缺失")
    target = validate_joints([float(row[f"joint_{i}_rad"])
                              for i in range(1, 8)],
                             config["zero_exclusion_radius_rad"])
    return target, [float(row[f"{axis}_m"]) for axis in "xyz"]


def plan_from_snapshot(robot, state, config, home):
    if (state["control_mode"] != 1 or state["arm_status"] != 0
            or state["error_code"] != 0 or not state["ready_for_motion"]):
        raise RuntimeError("当前控制器状态不允许外移")
    if home.get("name") != "H0" or not home.get("route_blocked"):
        raise RuntimeError("只允许从已停用路线的 H0 接触位做本次外移")
    current = validate_joints(state["joint_rad"], config["zero_exclusion_radius_rad"])
    if max(abs(a-b) for a, b in zip(current, home["joint_rad"])) > 0.005:
        raise RuntimeError("实时七轴不在原 H0 接触位附近")
    target, demonstrated_xyz = demonstrated_target(config)
    if math.dist(robot.fk(target)[:3], demonstrated_xyz) > 0.005:
        raise RuntimeError("示教样本法兰位置与当前正运动学不符")
    boxes = _assess(robot, [current, target], config, -0.18)
    delta_x = float(robot.fk(target)[0]) - state["flange_pose_m_rad"][0]
    if not -0.05 <= delta_x <= -0.04:
        raise RuntimeError("外侧目标不在预期 X 负向 40～50 mm 范围")
    return {"source_name": "H0_contact", "target_name": "demonstrated_outboard",
            "source_sample_index": SAMPLE_INDEX, "source_csv": str(RECORDING),
            "source_csv_sha256": RECORDING_SHA256,
            "start_joint_rad": current, "target_joint_rad": target,
            "target_flange_xyz_m": demonstrated_xyz,
            "predicted_x_change_m": delta_x,
            "joint_box_flange_ranges_m": boxes,
            "speed_percent": 2, "automatic_disable": False}


def session(*, run=False, config_path=DEFAULT_CONFIG, robot_factory=None,
            countdown_s=3):
    home = json.loads(HOME_FILE.read_text(encoding="utf-8"))
    if home.get("edge_release_blocked"):
        raise RuntimeError("旧外移方向已停用：" + home.get(
            "edge_release_block_reason", "须重新确认现场方向"))
    config = load_config(config_path)
    with connected(config, robot_factory) as robot:
        before = read_state(robot)
        plan = plan_from_snapshot(robot, before, config, home)
        print("外侧解压预检：", json.dumps(plan, ensure_ascii=False), flush=True)
        if not run:
            print("只读完成；未发送运动指令。", flush=True)
            return {"plan": plan, "ran": False}
        report = {"schema_version": 1, "kind": "h0_edge_release",
                  "started_at_utc": datetime.now(timezone.utc).isoformat(),
                  "completed": False, "preflight_state": before, "plan": plan,
                  "automatic_disable": False, "emergency_stop_requested": False}
        path = _write_new(RUN_DIR, "nero_edge_release_", report)
        interrupted = False

        def stop(_signum, _frame):
            nonlocal interrupted
            interrupted = True

        handlers = (signal.signal(signal.SIGINT, stop),
                    signal.signal(signal.SIGTERM, stop))
        try:
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("外移前取消")
                print(f"{remaining} 秒后发送一次 2% move_j 外移；按 Ctrl+C 取消。",
                      flush=True)
                time.sleep(1)
            if interrupted:
                raise InterruptedError("外移前取消")
            fresh = read_state(robot)
            checked = plan_from_snapshot(robot, fresh, config, home)
            if max(abs(a-b) for a, b in zip(checked["start_joint_rad"],
                                             plan["start_joint_rad"])) > 0.003:
                raise RuntimeError("倒计时后起点改变；未发送运动指令")
            execute_route(
                robot, config, [checked["start_joint_rad"], plan["target_joint_rad"]],
                lambda: interrupted, speed_percent=2,
                target_tolerance_rad=0.005, waypoint_timeout_s=20,
                waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                path_mode="joint_box", joint_box_margin_rad=0.03,
                on_emergency_stop=lambda reason: report.update(
                    emergency_stop_requested=True, emergency_stop_reason=reason))
            after = read_state(robot)
            error = max(abs(a-b) for a, b in zip(after["joint_rad"],
                                                 plan["target_joint_rad"]))
            report.update(final_state=after, final_max_joint_error_rad=error)
            if not after["ready_for_motion"] or error > 0.005:
                raise RuntimeError("外移后状态或到位误差不合格")
            report["completed"] = True
        except Exception as exc:
            report.update(error=str(exc), error_type=type(exc).__name__)
            raise RuntimeError(f"外侧解压未完成：{exc}；报告 {path}") from exc
        finally:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
            report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
            print("外侧解压报告：", path, flush=True)
        return {"report_path": str(path), "ran": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="预检通过后外移一次")
    args = parser.parse_args()
    try:
        session(run=args.run)
    except (ValueError, RuntimeError, InterruptedError) as exc:
        print("实验未完成：", exc, flush=True)
        raise SystemExit(1) from exc


if __name__ == "__main__":
    main()
