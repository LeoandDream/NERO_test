"""One-time low-speed J1 approach from the measured clear table-edge pose.

This only moves toward the edge a short distance. It never disables the arm;
the operator must assess the physical clearance and load at the endpoint.
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


SOURCE_Q = [0.4000120112645804, 0.3108082331951502,
            -0.028047441079548877, -0.05707226654021458,
            1.6599302916942469, 0.1315280124302927, 0.592539281052075]
TARGET_J1 = 0.34
STEP2_SOURCE_Q = [0.3400250448735353, 0.31079077990263027,
                  -0.028082347664588763, -0.05707226654021458,
                  1.6599302916942469, 0.13147565255273286,
                  0.5925741876371148]
STEP2_TARGET_J1 = 0.28
STEP3_SOURCE_Q = [0.2799682653124104, 0.3108082331951502,
                  -0.028082347664588763, -0.05707226654021458,
                  1.6599302916942469, 0.13147565255273286,
                  0.5925741876371148]
STEP3_TARGET_J1 = 0.247
STEP4_SOURCE_Q = [0.24701644903475747, 0.3108082331951502,
                  -0.028082347664588763, -0.05707226654021458,
                  1.6599302916942469, 0.13151055913777274,
                  0.5925741876371148]
STEP4_TARGET_J1 = 0.230
REPORT_DIR = Path("experiments/single_arm/lab/data/h0_j1_approaches")


def plan(robot, state, config, *, source_q=SOURCE_Q, target_j1=TARGET_J1):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("仅适用于当前左侧安装")
    start = list(state["joint_rad"])
    if max(abs(a-b) for a, b in zip(start, source_q)) > 0.003:
        raise ValueError("当前姿态已偏离这次退让终点；未发送运动指令")
    flange = state["flange_pose_m_rad"]
    if math.dist(robot.fk(start)[:3], flange[:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    target = list(start)
    target[0] = target_j1
    box = _assess(robot, [start, target], config, -0.14)[0]
    predicted = robot.fk(target)
    y_approach = predicted[1] - flange[1]
    if not 0.002 <= y_approach <= 0.011:
        raise ValueError("J1 目标并非预定的 2～11 mm Y 正向短距离靠边")
    if predicted[0] < flange[0] - 0.005 or box["x"][0] < -0.14:
        raise ValueError("J1 目标的 X 向变化超过本次现场范围")
    return start, target, box


def run(*, execute=False, countdown_s=5, step=1):
    if step not in (1, 2, 3, 4):
        raise ValueError("只允许已经现场复核的靠边步骤")
    if execute and step in (1, 2, 3):
        raise RuntimeError("该靠边目标已完成；禁止重复执行")
    source_q, target_j1 = {
        1: (SOURCE_Q, TARGET_J1),
        2: (STEP2_SOURCE_Q, STEP2_TARGET_J1),
        3: (STEP3_SOURCE_Q, STEP3_TARGET_J1),
        4: (STEP4_SOURCE_Q, STEP4_TARGET_J1),
    }[step]
    config = load_config(DEFAULT_CONFIG)
    report = {"kind": "one_time_h0_j1_short_approach", "step": step,
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "execution_requested": execute, "emergency_stop_requested": False}
    path = _write_new(REPORT_DIR, "nero_h0_j1_approach_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state = read_state(robot)
            start, target, box = plan(robot, state, config,
                                      source_q=source_q, target_j1=target_j1)
            report.update(start_state=state, target_joint_rad=target,
                          joint_box_flange_range_m=box, preflight_passed=True)
            print("仅 J1 向桌边短距离靠近；法兰起点/预测目标 XYZ：",
                  state["flange_pose_m_rad"][:3], robot.fk(target)[:3], flush=True)
            print("关节盒法兰范围：", box, flush=True)
            if not execute:
                print("只读预检完成；未发送运动指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后以 2% 执行短距离靠边；按 Ctrl+C 取消。",
                      flush=True)
                time.sleep(1)
            live_start, live_target, _ = plan(
                robot, read_state(robot), config,
                source_q=source_q, target_j1=target_j1)
            if max(abs(a-b) for a, b in zip(start, live_start)) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            execute_route(
                robot, config, [live_start, live_target], lambda: interrupted,
                speed_percent=2, target_tolerance_rad=0.005,
                waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                pause_on_stationary_timeout=True, path_mode="joint_box",
                joint_box_margin_rad=0.03,
                on_emergency_stop=lambda reason: report.update(
                    emergency_stop_requested=True, emergency_stop_reason=reason),
            )
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], target))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("短距离靠边关节误差超限")
            report["completed"] = True
            print(f"短距离靠边到位；最大关节误差 {error:.6f} rad；保持使能。",
                  flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"H0 单轴短距离靠边未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("H0 单轴靠边报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--step", type=int, choices=(1, 2, 3, 4), default=1)
    args = parser.parse_args(argv)
    try:
        run(execute=args.run, step=args.step)
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
