"""One-time move_j return from the verified outward table-edge point to S1.

The preceding edge release must have reached its target. The arm first moves
farther along the known table-clearance direction before aligning the wrist.
This does not authorize the reverse S1-to-H0 approach or disable the arm.
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
from experiments.single_arm.lab.edge_y_release_current import SOURCE_Q, TARGET_J1_RAD
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


REPORT_DIR = Path("experiments/single_arm/lab/data/returns_after_edge_y_release")
FLOOR_X = (-0.16, -0.11, 0.10, 0.12, 0.32, 0.32, 0.32)


def plan(robot, state, config):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("仅适用于当前左侧安装")
    start = list(state["joint_rad"])
    expected = list(SOURCE_Q)
    expected[0] = TARGET_J1_RAD
    if max(abs(a-b) for a, b in zip(start, expected)) > 0.005:
        raise ValueError("当前姿态不是本次 J1 沿 Y 负向外移的终点")
    if math.dist(robot.fk(start)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    s1 = config["safe_start_joint_rad"]
    route = [start]
    current = list(start)
    for joint, value in ((1, 0.2), (1, -0.2), (0, s1[0]), (1, s1[1])):
        current = list(current)
        current[joint] = value
        route.append(current)
    wrist_start = list(current)
    for fraction in (1/3, 2/3, 1):
        current = list(wrist_start)
        for joint in (2, 3, 4, 5, 6):
            current[joint] += fraction * (s1[joint] - current[joint])
        route.append(current)
    if max(abs(a-b) for a, b in zip(route[-1], s1)) > 1e-9:
        raise ValueError("末端不是 S1")
    boxes = _assess(robot, route, config, min(FLOOR_X))
    if any(box["x"][0] < floor for box, floor in zip(boxes, FLOOR_X)):
        raise ValueError("候选关节盒没有满足逐段离桌门槛")
    # The operator confirmed negative base Y is away from the person's table
    # edge. In the low-X region, keep the flange on that outward side.
    for a, b in zip(route, route[1:]):
        for step in range(101):
            q = [x+(y-x)*step/100 for x, y in zip(a, b)]
            x, y = robot.fk(q)[:2]
            if x < -0.05 and y > -0.020:
                raise ValueError("低位段可能向桌边内侧偏移")
    return route, boxes


def run(*, config_path=DEFAULT_CONFIG, countdown_s=5):
    config = load_config(Path(config_path))
    report = {"kind": "one_time_move_j_return_after_edge_y_release",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "completed_targets": []}
    path = _write_new(REPORT_DIR, "nero_return_s1_after_edge_y_release_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            route, boxes = plan(robot, read_state(robot), config)
            report.update(route_joint_rad=route, joint_box_flange_ranges_m=boxes)
            print(f"回 S1：{len(route)-1} 个 move_j 目标；终点法兰 XYZ：",
                  robot.fk(route[-1])[:3], flush=True)
            print("逐段法兰预测范围：", boxes, flush=True)
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后执行回 S1；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live_route, _ = plan(robot, read_state(robot), config)
            if max(abs(a-b) for a, b in zip(route[0], live_route[0])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            route[0] = live_route[0]
            for start_index, stop_index, speed in ((0, 1, 2), (1, 7, 5)):
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                execute_route(
                    robot, config, route[start_index:stop_index+1],
                    lambda: interrupted, speed_percent=speed,
                    target_tolerance_rad=0.005, waypoint_timeout_s=20,
                    waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                    path_mode="joint_box", joint_box_margin_rad=0.03,
                    on_waypoint=lambda index, target: report["completed_targets"].append({
                        "index": start_index + index, "joint_rad": target,
                        "speed_percent": speed}),
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason))
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                encoding="utf-8")
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], route[-1]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("S1 到位误差超限")
            report["completed"] = True
            print(f"已回 S1；7/7 目标到位，最大关节误差 {error:.6f} rad。",
                  flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"回 S1 未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("回 S1 报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    if not args.run:
        parser.error("须显式加 --run；执行命令会在同次调用中预检")
    try:
        run()
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
