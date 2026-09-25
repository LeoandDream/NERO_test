#!/usr/bin/env python3
"""从 2026-09-25 实测低位小幅抬离桌面；默认只读，--run 才运动。

仅改变 J2 -0.05 rad，其他六轴保持实时反馈值。起点必须与指定的
失能后稳定记录一致；这是一次性脱离接触试验，不是通用回 S1 路线。
"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.start_transfer.go_to_start import (
    execute_route, stable_joint_feedback,
)
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, validate_joints,
)


ANCHOR = Path("experiments/single_arm/pose_recording/data/recordings/"
              "nero_poses_20260925T143752Z.csv")
REPORT_DIR = Path("experiments/single_arm/start_transfer/data/runs")
J2_DELTA_RAD = -0.05
START_TOLERANCE_RAD = 0.005


def anchor_joints(path):
    """读取该次稳定低位记录；跨样本漂移则不能作为起点。"""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 10 or any(row["arm_status"] != "JOINT_BRAKE_NOT_RELEASED"
                              for row in rows):
        raise ValueError("低位记录缺失或状态不符合本次试验")
    samples = [[float(row[f"joint_{axis}_rad"]) for axis in range(1, 8)]
               for row in rows]
    reference = samples[-1]
    if max(abs(a - b) for sample in samples for a, b in zip(sample, reference)) > 0.002:
        raise ValueError("低位记录的关节姿态不稳定")
    return reference


def plan_lift(robot, current, anchor, config):
    """用实时姿态形成单轴目标，并逐点核对 FK 向离桌方向移动。"""
    if max(abs(a - b) for a, b in zip(current, anchor)) > START_TOLERANCE_RAD:
        raise RuntimeError("当前位置与低位记录不同；拒绝使用旧抬升路线")
    start = validate_joints(current, config["zero_exclusion_radius_rad"])
    target = list(start)
    target[1] += J2_DELTA_RAD
    validate_joints(target, config["zero_exclusion_radius_rad"])
    positions = []
    for step in range(51):
        point = list(start)
        point[1] += J2_DELTA_RAD * step / 50
        validate_joints(point, config["zero_exclusion_radius_rad"])
        pose = robot.fk(point)
        xyz = [float(value) for value in pose[:3]]
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("正运动学输出无效")
        if positions and xyz[0] < positions[-1][0] - 0.0001:
            raise ValueError("预测路径没有持续沿 X 正向离桌")
        positions.append(xyz)
    if not 0.02 <= positions[-1][0] - positions[0][0] <= 0.04:
        raise ValueError("预测法兰抬升量与本次几何复核不符")
    if math.dist(positions[0], positions[-1]) > 0.04:
        raise ValueError("预测法兰总位移超出本次试验上限")
    return target, positions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true", help="倒计时后只发送一次 move_j")
    args = parser.parse_args()
    robot = None
    report = None
    report_path = None
    try:
        config = load_config(args.config)
        if config["mount"] != "left":
            raise ValueError("本次抬升仅针对已确认的左侧安装")
        anchor = anchor_joints(ANCHOR)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
            channel=config["channel"], local_loopback=True,
        ))
        robot.connect()
        if wait_status(robot, fresh, timeout=3) is None:
            raise RuntimeError("控制器状态反馈缺失")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        current = stable_joint_feedback(robot)
        target, positions = plan_lift(robot, current, anchor, config)
        print("实时起点关节角 (rad)：", current, flush=True)
        print("单次 move_j 目标 (rad)：", target, flush=True)
        print("预测法兰沿基座 X 正向离桌 (m)：",
              round(positions[-1][0] - positions[0][0], 4), flush=True)
        print("预测法兰起点/终点 XYZ (m)：", positions[0], positions[-1], flush=True)
        print("速度 5%；只改变 J2 -0.05 rad；不自动回 S1。", flush=True)
        if not args.run:
            print("只读预检完成；未发送运动指令。", flush=True)
            return 0

        interrupted = False

        def request_stop(_signum, _frame):
            nonlocal interrupted
            interrupted = True

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        for remaining in range(5, 0, -1):
            if interrupted:
                raise InterruptedError("倒计时取消，未发送运动指令")
            print(f"{remaining} 秒后发送一次抬升目标；按 Ctrl+C 取消。", flush=True)
            time.sleep(1)
        if interrupted:
            raise InterruptedError("倒计时取消，未发送运动指令")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        latest = stable_joint_feedback(robot)
        target, positions = plan_lift(robot, latest, anchor, config)
        report = {
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "anchor_recording": str(ANCHOR),
            "start_joint_rad": latest, "target_joint_rad": target,
            "speed_percent": 5, "command_count": 0, "completed": False,
            "predicted_flange_start_xyz_m": positions[0],
            "predicted_flange_target_xyz_m": positions[-1],
        }
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        report_path = REPORT_DIR / ("nero_table_lift_" +
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
        try:
            execute_route(
                robot, config, [latest, target], lambda: interrupted,
                speed_percent=5, target_tolerance_rad=0.004,
                waypoint_timeout_s=10, waypoint_max_timeout_s=30,
                pause_on_stationary_timeout=True,
                path_mode="joint_box", joint_box_margin_rad=0.01,
                on_command_published=lambda _index, _target: report.update(command_count=1),
            )
            final = stable_joint_feedback(robot)
            error = max(abs(a - b) for a, b in zip(final, target))
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            report["final_joint_rad"] = final
            report["final_max_joint_error_rad"] = error
            report["completed"] = error <= 0.004
            if not report["completed"]:
                raise RuntimeError("抬升后关节误差超过 0.004 rad")
            print(f"抬升目标到位；最大关节误差 {error:.6f} rad。", flush=True)
        except Exception as exc:
            report["error"] = str(exc)
            raise
        finally:
            report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                   encoding="utf-8")
            print("抬升报告：", report_path, flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError,
            can.CanError) as exc:
        print(f"低位抬升未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
