#!/usr/bin/env python3
"""从已复核的起点每次发布一个完整 move_j 目标；默认只读预检。

仅用于当前试验的两目标候选：第 1 段先弯 J4 并调整 J7，第 2 段到 S1。
离线网格审计不能保证控制器真实路径或现场环境安全，因此每段须单独运行。
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.start_transfer.geometry.audit_self_collision import MESH_SHA256
from experiments.single_arm.start_transfer.go_to_start import (
    check_preplan_drivers, execute_route, stable_joint_feedback,
)
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, validate_joints,
)


REPORT_DIR = Path("experiments/single_arm/start_transfer/data/runs")
START_TOLERANCE_RAD = 0.005
TARGET_TOLERANCE_RAD = 0.005


def validate_candidate(plan_path, audit_path, config, stage=1):
    """只接受本次固定几何方法和同源的裸臂网格审计。"""
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    audit = json.loads(audit_path.read_text(encoding="utf-8"))
    if plan.get("executable") is not False or audit.get("executable") is not False:
        raise ValueError("只接受明确标记为离线候选的文件")
    if Path(audit.get("source_plan", "")).resolve() != plan_path.resolve():
        raise ValueError("网格审计对应另一份候选计划")
    if audit.get("mesh_sha256") != MESH_SHA256:
        raise ValueError("网格审计版本与本机固定模型不符")
    if (audit.get("environment_checked") is not False
            or audit.get("controller_actual_path_checked") is not False):
        raise ValueError("审计文件的现场和控制器路径字段不符合预期")
    checked = audit.get("two_target", {})
    if (checked.get("sample_count", 0) < 202
            or checked.get("nonadjacent_pair_count") != 21
            or checked.get("intersecting_pairs") != {}):
        raise ValueError("两目标裸臂采样检查不完整或发现网格相交")
    distances = checked.get("minimum_pair_distance_m", {})
    if len(distances) != 21 or min(distances.values()) < 0.010:
        raise ValueError("裸臂采样间距不足 10 mm；拒绝本次候选")
    box = audit.get("joint_box", {})
    varied = [2, 4, 7] if stage == 1 else [1, 2, 3, 5, 6]
    if (audit.get("joint_box_stage") != stage
            or box.get("varied_joint_numbers") != varied
            or box.get("sample_count", 0) < (1331 if stage == 1 else 3125)
            or box.get("nonadjacent_pair_count") != 21
            or box.get("intersecting_pairs") != {}):
        raise ValueError("本段关节不同步范围尚未完整做裸臂网格抽查")
    box_distances = box.get("minimum_pair_distance_m", {})
    if len(box_distances) != 21 or min(box_distances.values()) < 0.010:
        raise ValueError("本段关节不同步范围的裸臂间距不足 10 mm")

    zero_radius = config["zero_exclusion_radius_rad"]
    start = validate_joints(plan["start_joint_rad_from_recording"], zero_radius)
    middle = validate_joints(plan["candidate_intermediate_joint_rad"], zero_radius)
    target = validate_joints(plan["historical_s1_joint_rad"], zero_radius)
    if max(abs(a - b) for a, b in zip(target, config["safe_start_joint_rad"])) > 1e-9:
        raise ValueError("S1 配置已改变，须重新规划")
    expected_middle = list(start)
    expected_middle[1] = 0.2
    expected_middle[3] = target[3]
    expected_middle[6] = target[6]
    if max(abs(a - b) for a, b in zip(middle, expected_middle)) > 1e-9:
        raise ValueError("中间目标与本次限定的两目标策略不符")
    # 这不是运动安全证明，只检查文件所描述的假定路线和关节限位。
    for first, second in ((start, middle), (middle, target)):
        if math.dist(first, second) > 0.8:
            raise ValueError("单个关节目标距离超过 0.8 rad")
        for index in range(101):
            fraction = index / 100
            validate_joints([a + fraction * (b - a) for a, b in zip(first, second)],
                            zero_radius)
    return start, middle, target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True, help="两目标离线比较 JSON")
    parser.add_argument("--audit", type=Path, required=True, help="同源厂商网格审计 JSON")
    parser.add_argument("--stage", type=int, choices=(1, 2), default=1,
                        help="每次仅发布这一段的一个完整关节目标")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--speed-percent", type=int, choices=(5, 10), default=5)
    parser.add_argument("--run", action="store_true", help="显式发送本段唯一一个 move_j 目标")
    args = parser.parse_args()
    robot = None
    report = None
    try:
        config = load_config(args.config)
        start, middle, target = validate_candidate(args.plan, args.audit, config,
                                                   stage=args.stage)
        previous, destination = (start, middle) if args.stage == 1 else (middle, target)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
            channel=config["channel"], local_loopback=True,
        ))
        robot.connect()
        if wait_status(robot, fresh, timeout=3) is None:
            raise RuntimeError("未收到机械臂状态反馈")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_preplan_drivers(robot)
        check_drivers(robot)
        current = stable_joint_feedback(robot)
        if max(abs(a - b) for a, b in zip(current, previous)) > START_TOLERANCE_RAD:
            raise RuntimeError("实时起点与本段离线候选不符；须重新采样和审计")
        print(f"只读预检：第 {args.stage}/2 段", flush=True)
        print("实时起点 (rad)：", current, flush=True)
        print("单次 move_j 目标 (rad)：", destination, flush=True)
        print("目标关节空间距离 (rad)：", round(math.dist(current, destination), 4), flush=True)
        print(f"速度：{args.speed_percent}%；发送后最多等待 120 秒。", flush=True)
        print("网格审计抽查了本段关节不同步范围，但仅含裸臂离散姿态；须现场核对全部连杆、支撑和线缆。",
              flush=True)
        if not args.run:
            print("只读预检完成；未发送运动指令。", flush=True)
            return 0

        # 每个进程只执行一段。下一段必须由现场再次检查并另起进程。
        interrupted = False

        def request_stop(_signal, _frame):
            nonlocal interrupted
            interrupted = True

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        for remaining in range(5, 0, -1):
            if interrupted:
                raise InterruptedError("倒计时取消；没有发送运动指令")
            print(f"{remaining} 秒后发送第 {args.stage} 段单个目标；按 Ctrl+C 取消。",
                  flush=True)
            time.sleep(1)
        if interrupted:
            raise InterruptedError("倒计时取消；没有发送运动指令")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        if max(abs(a-b) for a,b in zip(joint_point(robot), previous)) > START_TOLERANCE_RAD:
            raise RuntimeError("倒计时期间关节位置改变；未发送运动指令")

        report = {
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "source_plan": str(args.plan), "source_audit": str(args.audit),
            "stage": args.stage, "start_joint_rad": current,
            "target_joint_rad": destination, "speed_percent": args.speed_percent,
            "command_count": 0, "completed": False,
            "feedback_samples": [], "status_samples": [],
        }
        report_path = REPORT_DIR / (
            "nero_controller_joint_return_" +
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
        )
        started = time.monotonic()
        try:
            execute_route(
                robot, config, [current, destination], lambda: interrupted,
                speed_percent=args.speed_percent,
                target_tolerance_rad=TARGET_TOLERANCE_RAD,
                waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                pause_on_stationary_timeout=True,
                path_mode="joint_box", joint_box_margin_rad=0.03,
                on_command=lambda _index, _target: report.update(command_attempted=True),
                on_command_published=lambda _index, _target: report.update(command_count=1),
                on_feedback=lambda joints: report["feedback_samples"].append(
                    {"elapsed_s": round(time.monotonic() - started, 3),
                     "joint_rad": list(joints)}
                ),
                on_status=lambda status: report["status_samples"].append(
                    {"elapsed_s": round(time.monotonic() - started, 3),
                     "arm_status": int(status.msg.arm_status),
                     "motion_status": int(status.msg.motion_status)}
                ),
            )
            final = stable_joint_feedback(robot)
            error = max(abs(a-b) for a,b in zip(final, destination))
            report["final_joint_rad"] = final
            report["final_max_joint_error_rad"] = error
            if error > TARGET_TOLERANCE_RAD:
                raise RuntimeError("本段结束后实测关节误差超过 0.005 rad")
            report["completed"] = True
            print(f"第 {args.stage} 段完成；最大关节误差 {error:.6f} rad。", flush=True)
        except Exception as exc:
            report["error"] = str(exc)
            report["failed"] = True
            print(f"第 {args.stage} 段未完成：{exc}", file=sys.stderr, flush=True)
            return 1
        finally:
            REPORT_DIR.mkdir(parents=True, exist_ok=True)
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                   encoding="utf-8")
            print("阶段报告：", report_path, flush=True)
        return 0
    except (ValueError, RuntimeError, InterruptedError, OSError) as exc:
        print(f"单目标回位预检未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    sys.exit(main())
