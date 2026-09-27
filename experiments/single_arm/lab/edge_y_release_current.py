"""One-time move_j exit from the measured 2026-09-26 table-edge pose.

It moves only J1, toward the operator-confirmed negative base-Y direction.
It stops at the outer point and does not enter teaching or disable the arm.
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
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


SOURCE_Q = [0.22404791607851207, 0.3026749988808566,
            0.01699950691442477, 0.06557201999742697,
            1.8085276242090442, 0.13199925132833115,
            0.5920680421540364]
TARGET_J1_RAD = 0.5
REPORT_DIR = Path("experiments/single_arm/lab/data/edge_y_releases_current")


def plan(robot, state, config):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("仅适用于当前左侧安装")
    start = list(state["joint_rad"])
    if max(abs(a-b) for a, b in zip(start, SOURCE_Q)) > 0.003:
        raise ValueError("实时姿态已偏离本次桌边起点；拒绝复用")
    flange = state["flange_pose_m_rad"]
    if math.dist(robot.fk(start)[:3], flange[:3]) > 0.005:
        raise ValueError("法兰反馈与模型不符")
    target = list(start)
    target[0] = TARGET_J1_RAD
    box = _assess(robot, [start, target], config, -0.18)[0]
    predicted = robot.fk(target)
    if (box["x"][0] < -0.18 or box["y"][1] > flange[1] + 0.002
            or predicted[1] > flange[1] - 0.035
            or predicted[0] < flange[0] + 0.01):
        raise ValueError("单轴目标未持续沿已确认的 Y 负向退出桌边")
    return start, target, box


def run(*, config_path=DEFAULT_CONFIG, countdown_s=5):
    config = load_config(Path(config_path))
    report = {"kind": "one_time_edge_y_release_move_j",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "automatic_return": False}
    path = _write_new(REPORT_DIR, "nero_edge_y_release_current_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state = read_state(robot)
            start, target, box = plan(robot, state, config)
            report.update(start_state=state, target_joint_rad=target,
                          joint_box_flange_range_m=box)
            print("单轴 J1 外移；法兰起点/预测目标 XYZ：",
                  state["flange_pose_m_rad"][:3], robot.fk(target)[:3], flush=True)
            print("关节盒法兰范围：", box, flush=True)
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后以 2% 执行单轴外移；按 Ctrl+C 取消。",
                      flush=True)
                time.sleep(1)
            live_start, live_target, _ = plan(robot, read_state(robot), config)
            if max(abs(a-b) for a, b in zip(start, live_start)) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            execute_route(
                robot, config, [live_start, live_target], lambda: interrupted,
                speed_percent=2, target_tolerance_rad=0.005,
                waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                pause_on_stationary_timeout=True, path_mode="joint_box",
                joint_box_margin_rad=0.03,
                on_emergency_stop=lambda reason: report.update(
                    emergency_stop_requested=True, emergency_stop_reason=reason))
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], target))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("外移目标误差超限")
            report["completed"] = True
            print(f"Y 负向外移到位；最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"桌边外移未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
        print("外移报告：", path, flush=True)
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
