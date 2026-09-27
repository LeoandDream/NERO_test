"""Monitored startup from the newly supported table-edge candidate to S1.

The first move_j target rotates J1 away from the table edge before any other
joint moves. The following targets are planned from measured safe-region poses;
the hand-guided recording is used as sparse geometry, never played backward.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.high_y_home import _check_outer_corridor, _source_points
from experiments.single_arm.lab.planned_return import _assess, _subdivide_long_segments, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


SUPPORTED_Q = [0.21568778896145926, 0.3107733266101103,
               -0.028029987787028934, -0.05707226654021458,
               1.6598081186466076, 0.13187707828069153,
               0.5924869211745151]
REPORT_DIR = Path("experiments/single_arm/lab/data/h0_candidate_starts")


def _check_supported_state(robot, state, config):
    if config["mount"] != "left" or state["control_mode"] != 1:
        raise ValueError("仅适用于当前左侧安装的 CAN 控制状态")
    if state["arm_status"] != 6 or state["error_code"] != 0:
        raise ValueError("当前并非已验收的失能桌边支撑状态")
    if any(d["enabled"] or d["undervoltage"] or d["driver_error"]
           for d in state["drivers"]):
        raise ValueError("七轴驱动状态不符合支撑位启动条件")
    if state["joint_variation_0_25s_rad"] > 0.002:
        raise ValueError("桌边支撑姿态尚未停稳")
    if max(abs(a-b) for a, b in zip(state["joint_rad"], SUPPORTED_Q)) > 0.01:
        raise ValueError("实际支撑姿态已偏离本次验收点")
    if math.dist(robot.fk(state["joint_rad"])[:3],
                 state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("法兰反馈与模型不符")
    return state


def plan(robot, state, config):
    start = list(state["joint_rad"])
    samples = _source_points(robot, config)
    first = list(start)
    first[0] = 0.5
    high = list(samples["high"])
    high[0] = 0.4
    high_outward = list(config["safe_start_joint_rad"])
    high_outward[0] = -0.2
    route = _subdivide_long_segments([
        start, first, samples["outer"], samples["outer_turn"],
        samples["outer_mid"], high, high_outward,
        list(config["safe_start_joint_rad"]),
    ])
    boxes = _assess(robot, route, config, -0.19)
    start_pose, first_pose = robot.fk(start), robot.fk(route[1])
    if (first_pose[1] > start_pose[1] - 0.03 or
            first_pose[0] < start_pose[0] + 0.005 or
            boxes[0]["y"][1] > start_pose[1] + 0.002):
        raise ValueError("第一目标未沿 Y 负向、X 正向离桌")
    _check_outer_corridor(robot, route[1:])
    return route, boxes


def run(*, execute=False, countdown_s=5,
        config_path=DEFAULT_CONFIG, robot_factory=None):
    config = load_config(config_path)
    report = {"kind": "h0_candidate_to_s1_move_j_trial",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "execution_requested": execute, "enable_commands_sent": 0,
              "completed": False, "completed_targets": [],
              "emergency_stop_requested": False}
    path = _write_new(REPORT_DIR, "nero_h0_candidate_start_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            before = _check_supported_state(robot, read_state(robot), config)
            route, boxes = plan(robot, before, config)
            report.update(before_state=before, route_joint_rad=route,
                          joint_box_flange_ranges_m=boxes, preflight_passed=True)
            print(f"支撑位→S1：{len(route)-1} 个 move_j 目标；第一目标仅 J1 离桌。",
                  flush=True)
            print("第一目标/终点法兰 XYZ：", robot.fk(route[1])[:3],
                  robot.fk(route[-1])[:3], flush=True)
            if not execute:
                print("只读预检完成；未发送使能或运动指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送使能或运动指令")
                print(f"{remaining} 秒后开始启动；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live = _check_supported_state(robot, read_state(robot), config)
            if max(abs(a-b) for a, b in zip(before["joint_rad"],
                                             live["joint_rad"])) > 0.002:
                raise RuntimeError("倒计时期间支撑姿态变化；未发送使能")
            report["enable_commands_sent"] = 1
            report["enable_return"] = bool(robot.enable())
            time.sleep(0.3)
            enabled = require_ready(read_state(robot))
            report["after_enable_state"] = enabled
            if max(abs(a-b) for a, b in zip(enabled["joint_rad"],
                                             live["joint_rad"])) > 0.02:
                raise RuntimeError("使能后姿态变化超过 0.02 rad；未发送运动目标")
            route, boxes = plan(robot, enabled, config)
            report.update(route_joint_rad=route,
                          joint_box_flange_ranges_m=boxes)
            for index in range(1, len(route)):
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                speed = 2 if index == 1 else 5
                execute_route(
                    robot, config, route[index-1:index+1],
                    lambda: interrupted, speed_percent=speed,
                    target_tolerance_rad=0.005, waypoint_timeout_s=20,
                    waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                    path_mode="joint_box", joint_box_margin_rad=0.03,
                    on_waypoint=lambda _i, target: report["completed_targets"].append(
                        {"index": index, "joint_rad": target, "speed_percent": speed}),
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason),
                )
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                encoding="utf-8")
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a,b in zip(final["joint_rad"], route[-1]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("S1 终点关节误差超限")
            report["completed"] = True
            print(f"支撑位→S1 完成；{len(route)-1}/{len(route)-1} 目标到位；"
                  f"最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"支撑位启动未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("启动试验报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        run(execute=args.run)
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
