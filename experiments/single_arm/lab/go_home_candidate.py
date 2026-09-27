"""Monitored S1-to-supported-home trial with a high negative-Y detour.

The last target is the newly calibrated light-contact candidate, not the old
over-pressed H0. This command leaves the joints enabled for an onsite contact
check. Disabling is a separate, guarded action.
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


CONTACT_Q = [0.23, 0.31084313978019007, -0.028082347664588763,
             -0.05707226654021458, 1.6599302916942469,
             0.13151055913777274, 0.592539281052075]
REPORT_DIR = Path("experiments/single_arm/lab/data/h0_candidate_parks")


def plan(robot, state, config):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("仅适用于当前左侧安装")
    start = list(state["joint_rad"])
    if max(abs(a-b) for a, b in zip(
            start, config["safe_start_joint_rad"])) > 0.01:
        raise ValueError("当前位置不是 S1；未发送运动指令")
    if math.dist(robot.fk(start)[:3],
                 state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    samples = _source_points(robot, config)
    high_outward = list(config["safe_start_joint_rad"])
    high_outward[0] = -0.2
    high = list(samples["high"])
    high[0] = 0.4
    outer_mid = list(samples["outer_mid"])
    outer_mid[0] = 0.7
    far = list(CONTACT_Q)
    far[0] = 0.5
    route = _subdivide_long_segments([
        start, high_outward, high, outer_mid, samples["outer_turn"],
        samples["outer"], far, list(CONTACT_Q),
    ])
    boxes = _assess(robot, route, config, -0.19)
    _check_outer_corridor(robot, route[:-1])
    if (max(abs(a-b) for a,b in zip(route[-2][1:], route[-1][1:])) > 1e-9 or
            robot.fk(route[-2])[1] > -0.06 or
            robot.fk(route[-1])[1] < -0.035 or
            boxes[-1]["y"][0] < -0.08):
        raise ValueError("最后目标不是从外侧仅用 J1 单轴贴边")
    return route, boxes


def run(*, execute=False, countdown_s=5):
    config = load_config(DEFAULT_CONFIG)
    report = {"kind": "s1_to_h0_candidate_move_j_trial",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "execution_requested": execute, "completed": False,
              "automatic_disable": False, "completed_targets": [],
              "emergency_stop_requested": False}
    path = _write_new(REPORT_DIR, "nero_h0_candidate_park_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            before = read_state(robot)
            route, boxes = plan(robot, before, config)
            report.update(before_state=before, route_joint_rad=route,
                          joint_box_flange_ranges_m=boxes, preflight_passed=True)
            print(f"S1→轻触支撑位：{len(route)-1} 个 move_j 目标；"
                  "高位先沿 Y 负向绕行，最后仅 J1 贴边。", flush=True)
            print("外侧点/终点法兰 XYZ：", robot.fk(route[-2])[:3],
                  robot.fk(route[-1])[:3], flush=True)
            if not execute:
                print("只读预检完成；未发送运动或失能指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后开始回家；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live_route, live_boxes = plan(robot, read_state(robot), config)
            if max(abs(a-b) for a,b in zip(route[0], live_route[0])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            route, boxes = live_route, live_boxes
            report.update(route_joint_rad=route,
                          joint_box_flange_ranges_m=boxes)
            for index in range(1, len(route)):
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                speed = 2 if index == len(route)-1 else 5
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
                raise RuntimeError("轻触支撑位关节误差超限")
            report["completed"] = True
            print(f"S1→轻触支撑位完成；{len(route)-1}/{len(route)-1} 目标到位；"
                  f"最大关节误差 {error:.6f} rad；保持使能。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"回家试验未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("回家试验报告：", path, flush=True)
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
