#!/usr/bin/env python3
"""以当前姿态为起点，J4 正向 0.05 rad 后沿同一小段返回。"""

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
    execute_route, flange_envelope, stable_joint_feedback,
)
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, validate_joints, validate_route_workspace,
)


J4_DELTA_RAD = 0.05
START_TOLERANCE_RAD = 0.005
RETURN_TOLERANCE_RAD = 0.005
REPORT_DIR = Path("experiments/single_arm/local_probe/data/runs")


def plan_probe(start, config):
    """仅在当前 J4 已为正时继续增大 J4，避免本对照穿越 J4 零位。"""
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    if start[3] < 0.02:
        raise ValueError("当前 J4 未在 +0.02 rad 以上；拒绝此正向对照路线")
    outward = list(start)
    outward[3] += J4_DELTA_RAD
    outward = validate_joints(outward, config["zero_exclusion_radius_rad"])
    if math.dist(start, outward) > 0.05 + 1e-9:
        raise ValueError("局部路线超过 0.05 rad 小步上限")
    return [start, outward, list(start)]


def write_report(path, report):
    """每次结束或中止时保留可读的 JSON 状态，不覆盖其他实验记录。"""
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    """默认只读规划；--run 时倒计时后复核起点并执行一次往返。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--speed-percent", type=int, choices=(5, 10), default=5)
    parser.add_argument("--run", action="store_true", help="显式执行 J4 小行程往返")
    args = parser.parse_args()

    try:
        config = load_config(args.config)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, fresh, timeout=3.0) is None:
                raise RuntimeError("控制器状态反馈缺失")
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            current = stable_joint_feedback(robot)
            route = plan_probe(current, config)
            validate_route_workspace(robot, route, config)
            envelope = flange_envelope(robot, route, config)
            print("实际起点关节角 (rad)：", route[0])
            print("J4 外行程目标 (rad)：", route[1][3])
            print("路线：J4 正向 0.05 rad，再回到本次实际起点；2 个小步。")
            print("法兰中心预测范围 (m)：", envelope)
            print(f"速度：{args.speed_percent}%；每步 5 秒到位上限。")
            if not args.run:
                print("只读规划完成；未发送运动指令。")
                return 0

            aborted = False

            def on_signal(_signum, _frame):
                nonlocal aborted
                aborted = True

            previous_int = signal.signal(signal.SIGINT, on_signal)
            previous_term = signal.signal(signal.SIGTERM, on_signal)
            try:
                for remaining in range(5, 0, -1):
                    if aborted:
                        raise InterruptedError("已取消实验")
                    print(f"{remaining} 秒后开始；按 Ctrl+C 取消。", flush=True)
                    time.sleep(1)
                # 用户观察规划结果后，必须用最新状态重新确认原位和完整路线。
                healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                check_drivers(robot)
                latest = stable_joint_feedback(robot)
                if max(abs(a - b) for a, b in zip(latest, route[0])) > START_TOLERANCE_RAD:
                    raise RuntimeError("倒计时期间起点改变；未发送运动指令")
                route = plan_probe(latest, config)
                validate_route_workspace(robot, route, config)
                envelope = flange_envelope(robot, route, config)

                REPORT_DIR.mkdir(parents=True, exist_ok=True)
                report_path = REPORT_DIR / (
                    "nero_local_probe_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
                )
                report = {
                    "started_at_utc": datetime.now(timezone.utc).isoformat(),
                    "speed_percent": args.speed_percent,
                    "j4_delta_rad": J4_DELTA_RAD,
                    "actual_start_joint_rad": route[0],
                    "outward_joint_rad": route[1],
                    "flange_xyz_envelope_m": envelope,
                    "last_reached_step": 0,
                    "completed": False,
                }
                with report_path.open("x", encoding="utf-8") as stream:
                    json.dump(report, stream, ensure_ascii=False, indent=2)
                    stream.write("\n")
                try:
                    deadline = time.monotonic() + 30.0
                    started = time.monotonic()

                    def reached(index, _target):
                        # 记录每步耗时，供 5% 与 10% 对照，而不是仅凭观感调速。
                        report["last_reached_step"] = index
                        report.setdefault("step_elapsed_s", []).append(round(time.monotonic() - started, 3))

                    execute_route(
                        robot, config, route,
                        lambda: aborted or time.monotonic() > deadline,
                        on_waypoint=reached,
                        target_tolerance_rad=RETURN_TOLERANCE_RAD,
                        speed_percent=args.speed_percent,
                    )
                    final = joint_point(robot)
                    error = max(abs(a - b) for a, b in zip(final, route[0]))
                    if error > RETURN_TOLERANCE_RAD:
                        raise RuntimeError(f"小行程已发送完毕，但回到起点误差 {error:.4f} rad")
                    healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                    check_drivers(robot)
                    report["final_joint_rad"] = final
                    report["max_return_joint_error_rad"] = error
                    report["completed"] = True
                    print(f"小行程往返完成；回位最大单关节误差 {error:.6f} rad。", flush=True)
                except Exception as exc:
                    report["error"] = str(exc)
                    # 已发送运动后的异常由 execute_route 执行阻尼急停；
                    # 此处只读记录故障后姿态，绝不自动重试或自动回位。
                    try:
                        report["joint_after_error_rad"] = joint_point(robot)
                    except Exception as feedback_exc:
                        report["joint_after_error_error"] = str(feedback_exc)
                    raise
                finally:
                    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                    write_report(report_path, report)
                    print(f"实验报告：{report_path}", flush=True)
            finally:
                signal.signal(signal.SIGINT, previous_int)
                signal.signal(signal.SIGTERM, previous_term)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError, can.CanError) as exc:
        print(f"局部测试未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
