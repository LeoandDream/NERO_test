#!/usr/bin/env python3
"""从已验证的桌面抬升姿态分两次完整 move_j 目标返回 S1。"""

import argparse
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
    load_config, validate_joints,
)


LIFT_REPORT = Path("experiments/single_arm/start_transfer/data/runs/"
                   "nero_table_lift_20260925T144728Z.json")
REPORT_DIR = Path("experiments/single_arm/start_transfer/data/runs")
START_TOLERANCE_RAD = 0.005
MIDDLE_J2_RAD = -0.2


def targets(config):
    """固定本次已验证的抬升报告，避免误用其他低位或重复抬升。"""
    source = json.loads(LIFT_REPORT.read_text(encoding="utf-8"))
    if source.get("completed") is not True or source.get("command_count") != 1:
        raise ValueError("桌面抬升报告未显示成功")
    lifted = validate_joints(source["final_joint_rad"], config["zero_exclusion_radius_rad"])
    s1 = config["safe_start_joint_rad"]
    middle = list(lifted)
    middle[1] = MIDDLE_J2_RAD
    validate_joints(middle, config["zero_exclusion_radius_rad"])
    if not 0.09 <= lifted[1] <= 0.105:
        raise ValueError("抬升报告中的 J2 不符合本次两目标路线")
    return lifted, middle, s1


def check_geometry(robot, first, second, config, stage):
    """离线重算假定关节插值；保持法兰朝离桌方向移动。"""
    xyz = []
    for index in range(101):
        fraction = index / 100
        point = [a + fraction * (b - a) for a, b in zip(first, second)]
        validate_joints(point, config["zero_exclusion_radius_rad"])
        position = [float(value) for value in robot.fk(point)[:3]]
        if not all(math.isfinite(value) for value in position):
            raise ValueError("正运动学输出无效")
        xyz.append(position)
    minimum_x = min(point[0] for point in xyz)
    if (stage == 1 and minimum_x < 0.0) or (stage == 2 and minimum_x < 0.15):
        raise ValueError("本段预测法兰太靠近桌面")
    if any(b[0] < a[0] - 0.001 for a, b in zip(xyz, xyz[1:])):
        raise ValueError("本段预测法兰 X 没有持续远离桌面")
    return xyz, minimum_x


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", type=int, choices=(1, 2), required=True,
                        help="每次仅执行一段；第一段 J2 至 -0.2，第二段对齐 S1")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true", help="倒计时后发布本段一次 move_j")
    args = parser.parse_args()
    robot = None
    report = None
    report_path = None
    try:
        config = load_config(args.config)
        if config["mount"] != "left":
            raise ValueError("本次路线仅针对已确认的左侧安装")
        lifted, middle, s1 = targets(config)
        previous, destination = ((lifted, middle) if args.stage == 1 else (middle, s1))
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
        if max(abs(a - b) for a, b in zip(current, previous)) > START_TOLERANCE_RAD:
            raise RuntimeError("实时姿态与本段起点不符；不要套用旧路线")
        predicted, min_x = check_geometry(robot, current, destination, config, args.stage)
        print(f"只读预检：第 {args.stage}/2 段", flush=True)
        print("实时起点关节角 (rad)：", current, flush=True)
        print("单次 move_j 目标 (rad)：", destination, flush=True)
        print("预测法兰起点/终点 XYZ (m)：", predicted[0], predicted[-1], flush=True)
        print(f"预测最小基座 X：{min_x:.4f} m；关节距离："
              f"{math.dist(current, destination):.3f} rad；速度：5%。", flush=True)
        print("假定关节插值与实际控制器路线可能不同；现场需检查桌面、连杆及线缆。",
              flush=True)
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
            print(f"{remaining} 秒后发送本段单个目标；按 Ctrl+C 取消。", flush=True)
            time.sleep(1)
        if interrupted:
            raise InterruptedError("倒计时取消，未发送运动指令")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        latest = stable_joint_feedback(robot)
        if max(abs(a - b) for a, b in zip(latest, previous)) > START_TOLERANCE_RAD:
            raise RuntimeError("倒计时期间起点改变；未发送运动指令")
        _, min_x = check_geometry(robot, latest, destination, config, args.stage)
        report = {
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "lift_report": str(LIFT_REPORT), "stage": args.stage,
            "start_joint_rad": latest, "target_joint_rad": destination,
            "predicted_min_flange_x_m": min_x,
            "speed_percent": 5, "command_count": 0, "completed": False,
        }
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        report_path = REPORT_DIR / ("nero_after_table_return_stage" + str(args.stage) +
            "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
        try:
            def monitor_feedback(joints):
                actual_x = float(robot.fk(joints)[0])
                # 基座 X 正向已由现场确认为离桌方向。该反馈阈值只检测
                # 已观测到的退回，不能代替对桌面实体与连杆的碰撞检测。
                if actual_x < (0.0 if args.stage == 1 else 0.14):
                    raise RuntimeError("实测法兰退回桌面方向，停止本段")

            execute_route(
                robot, config, [latest, destination], lambda: interrupted,
                speed_percent=5, target_tolerance_rad=0.005,
                waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                pause_on_stationary_timeout=True,
                path_mode="joint_box", joint_box_margin_rad=0.03,
                on_feedback=monitor_feedback,
                on_command_published=lambda _index, _target: report.update(command_count=1),
            )
            final = stable_joint_feedback(robot)
            error = max(abs(a - b) for a, b in zip(final, destination))
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            report["final_joint_rad"] = final
            report["final_max_joint_error_rad"] = error
            report["completed"] = error <= 0.005
            if not report["completed"]:
                raise RuntimeError("本段结束后关节误差超过 0.005 rad")
            print(f"第 {args.stage} 段到位；最大关节误差 {error:.6f} rad。", flush=True)
        except Exception as exc:
            report["error"] = str(exc)
            raise
        finally:
            report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                   encoding="utf-8")
            print("本段报告：", report_path, flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError,
            can.CanError) as exc:
        print(f"桌面抬升后回 S1 未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
