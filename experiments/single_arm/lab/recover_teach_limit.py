"""One-time, monitored recovery from the 2026-09-26 J4 teaching limit stop.

The failed recording is never replayed. This command first switches an idle
teaching controller to CAN joint mode without publishing a joint target, then
moves J4 only toward the SDK interior. It does not return to S1 automatically.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, JOINT_LIMITS, load_config, validate_flange_workspace,
    validate_joints,
)


SOURCE = Path("experiments/single_arm/teaching/data/recordings/"
              "nero_session_20260926T183143020488Z.csv")
REPORT_DIR = Path("experiments/single_arm/lab/data/teach_limit_recoveries")
J4_TARGET_RAD = -0.98
MAX_SOURCE_DEVIATION_RAD = 0.05


def _source_joint():
    metadata = json.loads(SOURCE.with_suffix(".json").read_text(encoding="utf-8"))
    if (metadata.get("recording_id") != SOURCE.stem or
            metadata.get("return_completed") is not False or
            metadata.get("stop_reason") != "error" or
            not str(metadata.get("error", "")).startswith("关节 4 当前") or
            metadata.get("recording_sha256") != hashlib.sha256(SOURCE.read_bytes()).hexdigest()):
        raise ValueError("只允许恢复本次 J4 限位中止记录")
    with SOURCE.open(newline="", encoding="utf-8") as stream:
        last = None
        for last in csv.DictReader(stream):
            pass
    if last is None:
        raise ValueError("中止示教记录没有有效姿态")
    return [float(last[f"joint_{i}_rad"]) for i in range(1, 8)]


def plan(robot, state, config, source_joint):
    """Accept only this stopped teach endpoint and an inward J4-only step."""
    if (config["mount"] != "left" or state["control_mode"] != 2 or
            state["arm_status"] != 0 or state["teach_status"] not in (0, 2, 6) or
            state["error_code"] != 0 or
            state["joint_variation_0_25s_rad"] > 0.002 or
            len(state["drivers"]) != 7 or
            any(not d["enabled"] or d["undervoltage"] or d["driver_error"]
                for d in state["drivers"])):
        raise RuntimeError("须在停稳、七轴使能的空闲示教模式恢复")
    q = list(state["joint_rad"])
    if (len(q) != 7 or
            max(abs(a-b) for a,b in zip(q, source_joint)) >
            MAX_SOURCE_DEVIATION_RAD):
        raise ValueError("实时姿态已偏离本次中止示教终点")
    lower = JOINT_LIMITS[3][0]
    if not lower - 0.01 <= q[3] < lower + 0.01:
        raise ValueError("J4 不在本次限位边缘恢复窗口")
    target = list(q)
    target[3] = J4_TARGET_RAD
    if not 0.01 <= target[3]-q[3] <= 0.05:
        raise ValueError("J4 恢复距离超出 0.01～0.05 rad")
    validate_joints(target, config["zero_exclusion_radius_rad"])
    if math.dist(robot.fk(q)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰位姿与模型不一致")
    poses = []
    for index in range(101):
        sample = list(q)
        sample[3] += (target[3]-q[3])*index/100
        validate_flange_workspace(robot, sample, config)
        poses.append([float(v) for v in robot.fk(sample)[:3]])
    if min(p[0] for p in poses) < 0.25:
        raise ValueError("法兰 X 低于本次离桌恢复门槛")
    return {"start_joint_rad": q, "target_joint_rad": target,
            "flange_xyz_range_m": {axis: [min(p[i] for p in poses),
                                           max(p[i] for p in poses)]
                                   for i, axis in enumerate("xyz")},
            "speed_percent": 2, "move_j_targets": 1}


def run(*, execute=False, config_path=DEFAULT_CONFIG, countdown_s=5,
        robot_factory=None):
    config = load_config(Path(config_path))
    source_joint = _source_joint()
    report = {"kind": "one_time_j4_teach_limit_recovery",
              "source_recording": str(SOURCE),
              "created_at_utc": datetime.now(timezone.utc).isoformat(),
              "execution_requested": execute, "completed": False,
              "mode_switch_attempted": False, "move_j_attempted": False,
              "emergency_stop_requested": False}
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    path = REPORT_DIR / ("nero_teach_limit_recovery_" +
                         datetime.now(timezone.utc).strftime(
                             "%Y%m%dT%H%M%S%fZ") + ".json")
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            initial = read_state(robot)
            planned = plan(robot, initial, config, source_joint)
            report.update(start_state=initial, plan=planned,
                          preflight_passed=True)
            print("本次 J4 限位恢复只读预检：", planned, flush=True)
            if not execute:
                print("只读完成；未切换模式或发送运动指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未切换模式")
                print(f"{remaining} 秒后先切回 CAN，再仅调整 J4；按 Ctrl+C 取消。",
                      flush=True)
                time.sleep(1)
            before = read_state(robot)
            plan(robot, before, config, source_joint)
            if max(abs(a-b) for a,b in zip(
                    initial["joint_rad"], before["joint_rad"])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未切换模式")
            report["mode_switch_attempted"] = True
            execute_route(robot, config, [before["joint_rad"]],
                          lambda: interrupted, speed_percent=2,
                          path_mode="joint_box", pause_on_stationary_timeout=True,
                          on_emergency_stop=lambda reason: report.update(
                              emergency_stop_requested=True,
                              emergency_stop_reason=reason))
            switched = require_ready(read_state(robot))
            report["after_mode_switch_state"] = switched
            if max(abs(a-b) for a,b in zip(
                    before["joint_rad"], switched["joint_rad"])) > 0.003:
                raise RuntimeError("切回 CAN 后姿态变化超过 0.003 rad；未发布目标")
            # Recompute the one-axis target from the exact post-switch pose.
            candidate = dict(switched, control_mode=2)
            live_plan = plan(robot, candidate, config, source_joint)
            report["executed_plan"] = live_plan
            if interrupted:
                raise InterruptedError("模式切换后取消；未发布目标")
            report["move_j_attempted"] = True
            execute_route(
                robot, config,
                [switched["joint_rad"], live_plan["target_joint_rad"]],
                lambda: interrupted, speed_percent=2,
                target_tolerance_rad=0.005, waypoint_timeout_s=15,
                waypoint_max_timeout_s=60, pause_on_stationary_timeout=True,
                path_mode="joint_box", joint_box_margin_rad=0.01,
                on_emergency_stop=lambda reason: report.update(
                    emergency_stop_requested=True,
                    emergency_stop_reason=reason))
            final = require_ready(read_state(robot))
            validate_joints(final["joint_rad"],
                            config["zero_exclusion_radius_rad"])
            error = max(abs(a-b) for a,b in zip(
                final["joint_rad"], live_plan["target_joint_rad"]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("J4 恢复目标未到位")
            report["completed"] = True
            print(f"J4 已回到限位余量内；最大关节误差 {error:.6f} rad。"
                  "保持使能，未自动回 S1。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"J4 限位恢复未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("限位恢复报告：", path, flush=True)
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
