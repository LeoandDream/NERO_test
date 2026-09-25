#!/usr/bin/env python3
"""从实时稳定姿态逐段探测法兰直线位置；每次 --run 只发一条 move_l。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

import can

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.continuous_replay.continuous_session import (
    driver_snapshot, read_realtime, robot_instance, utc_now, write_json_exclusive,
)
from experiments.single_arm.continuous_replay.run_stream import _controlled_stop, _read_snapshot
from experiments.single_arm.control_interfaces.cartesian_linear_session import (
    read_anchor_recording, preview_line,
)
from experiments.single_arm.replay.can_trace import ReplayCanTrace
from experiments.single_arm.start_transfer.go_to_start import stable_joint_feedback
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, check_drivers, load_config, point_segment_distance,
)


ROOT = Path("experiments/single_arm/reachability/data")
PLAN_DIR = ROOT / "plans"
RUN_DIR = ROOT / "runs"
TRACE_IDS = (0x151, 0x152, 0x153, 0x154, 0x2A1, 0x2A5,
             0x2A6, 0x2A7, 0x2A9, 0x471)


def create_plan(robot, config, config_path, axis, delta, anchor_path):
    if axis not in "xyz" or not 0.002 <= abs(delta) <= 0.010:
        raise ValueError("单段轴向位移须在 2～10 mm；每段从实时姿态重新生成")
    status, start, flange = read_realtime(robot)
    check_drivers(robot)
    stable = stable_joint_feedback(robot)
    if max(abs(a-b) for a, b in zip(start, stable)) > 0.002:
        raise ValueError("起点未停稳")
    anchor_joint = None
    if anchor_path is not None:
        anchor_joint, anchor_pose = read_anchor_recording(anchor_path, robot, config)
        if (max(abs(a-b) for a, b in zip(start, anchor_joint)) > 0.005
                or math.dist(flange[:3], anchor_pose[:3]) > 0.005):
            raise ValueError("实时起点与记录锚点不一致")
    target = list(flange)
    target["xyz".index(axis)] += delta
    route = preview_line(robot, start, flange, target, config, anchor_joint)
    return {
        "schema_version": 1,
        "created_at_utc": utc_now(),
        "config_sha256": hashlib.sha256(Path(config_path).read_bytes()).hexdigest(),
        "axis": axis, "delta_m": delta,
        "anchor_recording": str(anchor_path) if anchor_path else None,
        "start_joint_rad": start, "start_flange_pose": flange,
        "target_flange_pose": target,
        "offline_nearby_ik_max_joint_change_rad": max(
            max(abs(a-b) for a, b in zip(point, start)) for point in route),
        "offline_route_joint_rad": route,
        "speed_percent": 5,
        "note": "只预检单条直线，实际控制器 IK 和连杆路线仍需现场观察；不自动返回。",
    }


def check_plan(robot, plan, config, config_path):
    if plan.get("schema_version") != 1 or hashlib.sha256(Path(config_path).read_bytes()).hexdigest() != plan.get("config_sha256"):
        raise ValueError("计划格式或配置已改变")
    if plan["axis"] not in "xyz" or not 0.002 <= abs(plan["delta_m"]) <= 0.010:
        raise ValueError("位移超出单段限制")
    start = plan["start_flange_pose"]
    target = plan["target_flange_pose"]
    expected = list(start)
    expected["xyz".index(plan["axis"])] += plan["delta_m"]
    if len(target) != 6 or any(abs(a-b) > 1e-9 for a, b in zip(expected, target)):
        raise ValueError("目标与轴向位移不符")
    status, current, flange = read_realtime(robot)
    check_drivers(robot)
    if (max(abs(a-b) for a, b in zip(current, plan["start_joint_rad"])) > 0.005
            or math.dist(flange[:3], start[:3]) > 0.003):
        raise ValueError("实时起点偏离本段计划；须重新规划")
    anchor_joint = None
    if plan.get("anchor_recording"):
        anchor_joint, _ = read_anchor_recording(Path(plan["anchor_recording"]), robot, config)
    route = preview_line(robot, current, flange, target, config, anchor_joint)
    # 实时反馈的毫米级噪声可能让 ceil(距离/2mm) 改变采样数；比较终点解。
    if max(abs(a-b) for a,b in zip(route[-1], plan["offline_route_joint_rad"][-1])) > 0.02:
        raise ValueError("实时近邻 IK 终点偏离离线计划")
    return current, flange


def execute(robot, plan, plan_path, config, config_path):
    stem = "nero_probe_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = RUN_DIR / f"{stem}.json"
    trace_path = RUN_DIR / f"{stem}.can.csv"
    report = {"schema_version": 1, "plan": str(plan_path),
              "started_at_utc": utc_now(), "control_command": "move_l",
              "command_count": 0, "completed": False,
              "feedback_samples": [], "driver_snapshots": [],
              "can_trace_csv": str(trace_path)}
    write_json_exclusive(output, report)
    previous = None
    stopped = False
    sent = False
    trace = None

    def on_signal(_number, _frame):
        nonlocal stopped
        stopped = True

    previous = (signal.signal(signal.SIGINT, on_signal),
                signal.signal(signal.SIGTERM, on_signal))
    try:
        start_joint, start_pose = check_plan(robot, plan, config, config_path)
        trace = ReplayCanTrace(config["channel"], trace_path,
                               frame_ids=TRACE_IDS, flush_every=20)
        trace.start()
        report["driver_snapshots"].append(driver_snapshot(robot))
        for remaining in range(5, 0, -1):
            if stopped:
                raise RuntimeError("倒计时取消")
            print(f"{remaining} 秒后探测 {plan['axis']} {plan['delta_m']:+.3f} m；按 Ctrl+C 停止。", flush=True)
            time.sleep(1)
        check_plan(robot, plan, config, config_path)
        if stopped:
            raise RuntimeError("发送前取消")
        robot.set_speed_percent(5)
        report["command_attempted_at_utc"] = utc_now()
        sent = True  # SDK 若在一组 CAN 帧中途抛错，仍须按已发送处理。
        robot.move_l(plan["target_flange_pose"])
        report["command_count"] = 1
        report["command_returned_at_utc"] = utc_now()
        started = time.monotonic()
        last_driver = started
        steady = 0
        while time.monotonic() - started < 30:
            if stopped:
                raise RuntimeError("现场停止指令")
            trace.check()
            status, joints, pose, joint_stamp, pose_stamp = read_realtime(
                robot, with_timestamps=True)
            elapsed = time.monotonic() - started
            if elapsed - (last_driver-started) >= 1.0:
                report["driver_snapshots"].append(driver_snapshot(robot))
                last_driver = time.monotonic()
            line_error = point_segment_distance(pose[:3], start_pose[:3],
                                                plan["target_flange_pose"][:3])
            joint_change = max(abs(a-b) for a, b in zip(joints, start_joint))
            rotation_change = max(abs(a-b) for a,b in zip(pose[3:], start_pose[3:]))
            position_error = math.dist(pose[:3], plan["target_flange_pose"][:3])
            report["feedback_samples"].append({
                "elapsed_s": elapsed, "flange_pose": pose, "joint_rad": joints,
                "joint_feedback_unix_s": joint_stamp, "flange_feedback_unix_s": pose_stamp,
                "arm_status": int(status.msg.arm_status),
                "motion_status": int(status.msg.motion_status),
                "line_error_m": line_error, "joint_change_rad": joint_change,
                "rotation_change_rad": rotation_change,
                "position_error_m": position_error,
            })
            if line_error > 0.005 or joint_change > 0.25 or rotation_change > 0.03:
                raise RuntimeError("法兰直线或关节偏离本段门槛")
            if position_error <= 0.0005 and int(status.msg.motion_status) == 0:
                steady += 1
                if steady >= 5:
                    report["completed"] = True
                    report["final_joint_rad"] = joints
                    report["final_flange_pose"] = pose
                    report["final_position_error_m"] = position_error
                    report["driver_snapshots"].append(driver_snapshot(robot))
                    break
            else:
                steady = 0
            time.sleep(0.05)
        else:
            raise RuntimeError("单段超过 30 秒，未确认到位")
        print(f"探测到位；法兰位置误差 {report['final_position_error_m']:.6f} m。", flush=True)
    except Exception as exc:
        report["error"] = str(exc)
        report["failure_snapshot"] = _read_snapshot(robot)
        if sent:
            _controlled_stop(robot, report)
        raise
    finally:
        if trace is not None:
            try:
                trace.stop()
            except Exception as exc:
                report["trace_stop_error"] = str(exc)
            report["can_trace_frames"] = trace.count
        report["finished_at_utc"] = utc_now()
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        signal.signal(signal.SIGINT, previous[0])
        signal.signal(signal.SIGTERM, previous[1])
        print("探测报告：", output, flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--axis", choices=("x", "y", "z"))
    parser.add_argument("--delta-m", type=float)
    parser.add_argument("--anchor-recording", type=Path)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if args.run and args.plan is None:
        parser.error("--run 必须指定已保存的 --plan")
    if args.plan is None and (args.axis is None or args.delta_m is None):
        parser.error("生成计划时必须给 --axis 和 --delta-m")
    robot = None
    try:
        config = load_config(args.config)
        robot = robot_instance(config["channel"])
        robot.connect()
        if wait_status(robot, lambda item: fresh(item, 0.2), timeout=3) is None:
            raise RuntimeError("CAN 整机状态反馈缺失")
        deadline = time.monotonic() + 3.0
        while True:
            try:
                read_realtime(robot)
                break
            except RuntimeError as exc:
                if ("反馈超过" not in str(exc) and "反馈组成帧" not in str(exc)) or time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        if args.plan is None:
            plan = create_plan(robot, config, args.config, args.axis, args.delta_m,
                               args.anchor_recording)
            path = PLAN_DIR / ("nero_probe_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
            write_json_exclusive(path, plan)
        else:
            path = args.plan
            plan = json.loads(path.read_text(encoding="utf-8"))
            check_plan(robot, plan, config, args.config)
        print("探测计划：", path)
        print("实际起点法兰：", plan["start_flange_pose"])
        print("目标法兰：", plan["target_flange_pose"])
        print("近邻 IK 最大关节变化：", plan["offline_nearby_ik_max_joint_change_rad"])
        if not args.run:
            print("只读预检完成；未发送运动指令。")
            return 0
        execute(robot, plan, path, config, args.config)
        return 0
    except (OSError, ValueError, RuntimeError, can.CanError) as exc:
        print(f"探测未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
