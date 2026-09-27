"""One measured 2% move_j to relieve excessive contact at the old H0.

This one-time command starts only at the old H0, retreats about 5 mm in base
negative Y, and remains enabled. It does not approve a new shutdown pose.
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
from experiments.single_arm.lab.high_y_home import HOME_PATH, _source_points
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


REPORT_DIR = Path("experiments/single_arm/lab/data/h0_contact_relief")
# The first trial moved negative Y by 5 mm but also negative X by 15 mm; the
# operator still reported excessive contact or an abnormal condition. Retire
# this target, including any repeat from its original starting pose.
EXECUTION_HOLD = True


def plan(robot, state, config):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("仅适用于现场左侧安装")
    home = json.loads(HOME_PATH.read_text(encoding="utf-8"))
    if home.get("name") != "H0" or home.get("route_blocked") is not True:
        raise ValueError("原 H0 及路线锁定状态已变化")
    current = list(state["joint_rad"])
    h0 = home["joint_rad"]
    if max(abs(a-b) for a, b in zip(current, h0)) > 0.003:
        raise ValueError("实时姿态不是刚才过度贴边的旧 H0")
    if math.dist(robot.fk(current)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    precontact = _source_points(robot, config)["precontact"]
    target = [(a+b)/2 for a, b in zip(h0, precontact)]
    box = _assess(robot, [current, target], config, -0.16)[0]
    start_xyz, target_xyz = robot.fk(current), robot.fk(target)
    if (not -0.006 < target_xyz[1] - start_xyz[1] < -0.004
            or box["y"][1] > start_xyz[1] + 0.001
            or math.dist(current, target) > 0.07):
        raise ValueError("单次退让没有保持预定 Y 负向约 5 mm")
    return current, target, box


def run(*, execute=False, countdown_s=5):
    if execute and EXECUTION_HOLD:
        raise RuntimeError("首次退让后现场仍报告受压或异常；本目标已锁定")
    config = load_config(DEFAULT_CONFIG)
    report = {"kind": "one_time_h0_contact_relief_move_j",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "execution_requested": execute,
              "emergency_stop_requested": False}
    path = _write_new(REPORT_DIR, "nero_h0_contact_relief_", report)
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
                          joint_box_flange_range_m=box, preflight_passed=True)
            print("旧 H0 接触退让；法兰起点/目标 XYZ：",
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
                print(f"{remaining} 秒后以 2% 沿 Y 负向退让；按 Ctrl+C 取消。",
                      flush=True)
                time.sleep(1)
            live, live_target, _ = plan(robot, read_state(robot), config)
            if max(abs(a-b) for a, b in zip(start, live)) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            execute_route(
                robot, config, [live, live_target], lambda: interrupted,
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
                raise RuntimeError("单次退让关节误差超限")
            report["completed"] = True
            print(f"退让到位；最大关节误差 {error:.6f} rad；保持使能。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"H0 接触退让未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("H0 接触退让报告：", path, flush=True)
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
