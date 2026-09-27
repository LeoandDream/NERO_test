"""One-time move_j return to S1 after the confirmed negative-Y probe.

This route is only for the measured 2026-09-26 pose. It does not unlock the
blocked H0 approach, start drag teaching, or disable the arm.
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
from experiments.single_arm.lab.supported_home import _route_for
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


SOURCE_Q = [
    0.22952824992977428, 0.30560715202420713, -0.04799655442984406,
    -0.1439547567044923, 1.6672432212601032, 0.12428489603451622,
    0.5856103239216573,
]
HOME_PATH = Path("experiments/single_arm/lab/config/supported_home.json")
REPORT_DIR = Path("experiments/single_arm/lab/data/returns_after_y_probe")
MIN_X_BY_LEG = (-0.12, -0.11, -0.11, 0.34, 0.38, 0.37)
ROUTE_BLOCKED = True  # Operator later rejected its X-first clearance design.


def build_route(robot, state, config):
    require_ready(state)
    start = list(state["joint_rad"])
    if max(abs(a - b) for a, b in zip(start, SOURCE_Q)) > 0.003:
        raise ValueError("实时起点已偏离 5 mm Y 向试探终点")
    if math.dist(robot.fk(start)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    home = json.loads(HOME_PATH.read_text(encoding="utf-8"))
    if not home.get("route_blocked") or config["mount"] != "left":
        raise ValueError("此一次性回 S1 入口要求原 H0 路线仍锁定且安装方向为 left")
    h0 = home["joint_rad"]
    s1 = config["safe_start_joint_rad"]
    old_return = _route_for("H0", start, s1, h0)
    release = list(start)
    release[1] = h0[1] - 0.04
    route = [start, release, *old_return[1:]]
    if len(route) != 7 or max(abs(a - b) for a, b in zip(route[-1], s1)) > 1e-9:
        raise ValueError("回 S1 关节目标异常")
    boxes = _assess(robot, route, config, min(MIN_X_BY_LEG))
    if any(box["x"][0] < bound for box, bound in zip(boxes, MIN_X_BY_LEG)):
        raise ValueError("候选路线没有保持各段离桌边界")
    return route, boxes


def run(*, config_path=DEFAULT_CONFIG, execute=False, countdown_s=5):
    if ROUTE_BLOCKED:
        raise RuntimeError("现场确认本路线先走 X、没有先沿 Y 负向离桌；此一次性路线已停用")
    config = load_config(Path(config_path))
    report = {
        "kind": "one_time_move_j_return_to_s1_after_y_probe",
        "started_at_utc": datetime.now(timezone.utc).isoformat(),
        "completed": False,
        "source_y_probe": "experiments/single_arm/lab/data/h0_y_probes/nero_h0_y_probe_20260926T102804488586Z.json",
        "old_h0_route_blocked": True,
        "automatic_disable": False,
        "completed_targets": [],
    }
    report_path = _write_new(REPORT_DIR, "nero_return_s1_after_y_probe_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state = read_state(robot)
            route, boxes = build_route(robot, state, config)
            report.update(start_state=state, route_joint_rad=route,
                          joint_box_flange_ranges_m=boxes)
            print("回 S1：6 个 move_j 目标；当前/目标法兰 XYZ：",
                  state["flange_pose_m_rad"][:3], robot.fk(route[-1])[:3], flush=True)
            print("各段预测法兰范围：", boxes, flush=True)
            if not execute:
                print("只读预检完成；未发送运动指令。", flush=True)
                return report_path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后回 S1；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live = read_state(robot)
            live_route, _ = build_route(robot, live, config)
            if max(abs(a - b) for a, b in zip(route[0], live_route[0])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            route[0] = live_route[0]
            for points, speed, offset in ((route[:3], 2, 0), (route[2:], 5, 2)):
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                execute_route(
                    robot, config, points, lambda: interrupted,
                    speed_percent=speed, target_tolerance_rad=0.005,
                    waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                    pause_on_stationary_timeout=True, path_mode="joint_box",
                    joint_box_margin_rad=0.03,
                    on_waypoint=lambda index, target: report["completed_targets"].append({
                        "index": offset + index, "target_joint_rad": target,
                        "speed_percent": speed}),
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason))
                report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                       encoding="utf-8")
            final = require_ready(read_state(robot))
            error = max(abs(a - b) for a, b in zip(final["joint_rad"], route[-1]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("S1 到位误差超限")
            report["completed"] = True
            print(f"已回 S1；6/6 目标到位，最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"回 S1 未完成：{exc}；报告 {report_path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                               encoding="utf-8")
        print("回 S1 报告：", report_path, flush=True)
    return report_path


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
