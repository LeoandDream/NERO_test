#!/usr/bin/env python3
"""从急停恢复姿态只读规划 J4/J7 微小联动；--run 才执行单步。"""

import argparse
from datetime import datetime, timezone
import json
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


J4_DELTA_RAD = -0.01
J7_DELTA_RAD = -0.01
REPORT_DIR = Path("experiments/single_arm/start_transfer/data/diagnostics")


def plan_probe(start, config):
    """只接受本次恢复后的近零 J2、负角 J4、正角 J7 起点。"""
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    if not (-0.2 <= start[1] <= 0.2 and -0.2 <= start[3] <= -0.03
            and 0.35 <= start[6] <= 0.55):
        raise ValueError("实时 J2/J4/J7 不在本次微小联动测试的起点范围")
    target = list(start)
    target[3] += J4_DELTA_RAD
    target[6] += J7_DELTA_RAD
    target = validate_joints(target, config["zero_exclusion_radius_rad"])
    return [start, target]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true", help="显式执行 0.01 rad 级单步测试")
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
            print("实际起点 (rad)：", route[0])
            print("单步目标 (rad)：", route[1])
            print("变化：J4 -0.01 rad，J7 -0.01 rad；其余关节目标不变。")
            print("法兰中心预测范围 (m)：", envelope)
            print("速度：5%；最多等待 5 秒；成功后停在新姿态，不自动返回。")
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
                        raise InterruptedError("已取消微小联动测试")
                    print(f"{remaining} 秒后执行单步；按 Ctrl+C 取消。", flush=True)
                    time.sleep(1)
                if aborted:
                    raise InterruptedError("已取消微小联动测试")
                healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                check_drivers(robot)
                latest = stable_joint_feedback(robot)
                if max(abs(a - b) for a, b in zip(latest, route[0])) > 0.005:
                    raise RuntimeError("倒计时期间起点变化；未发送运动指令")
                route = plan_probe(latest, config)
                validate_route_workspace(robot, route, config)
                flange_envelope(robot, route, config)

                REPORT_DIR.mkdir(parents=True, exist_ok=True)
                path = REPORT_DIR / (
                    "nero_j7_coupled_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
                )
                report = {
                    "started_at_utc": datetime.now(timezone.utc).isoformat(),
                    "actual_start_joint_rad": route[0],
                    "target_joint_rad": route[1],
                    "last_reached_step": 0,
                    "completed": False,
                }
                with path.open("x", encoding="utf-8") as stream:
                    json.dump(report, stream, ensure_ascii=False, indent=2)
                    stream.write("\n")
                try:
                    execute_route(
                        robot, config, route, lambda: aborted,
                        on_waypoint=lambda index, _target: report.update(last_reached_step=index),
                        target_tolerance_rad=0.003,
                        speed_percent=5,
                    )
                    actual = joint_point(robot)
                    error = max(abs(a - b) for a, b in zip(actual, route[1]))
                    healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                    check_drivers(robot)
                    report["actual_joint_rad"] = actual
                    report["max_joint_error_rad"] = error
                    report["completed"] = error <= 0.003
                    if not report["completed"]:
                        raise RuntimeError("单步执行后关节误差超过 0.003 rad")
                    print(f"单步完成；最大关节误差 {error:.6f} rad。", flush=True)
                except Exception as exc:
                    report["error"] = str(exc)
                    try:
                        report["joint_after_error_rad"] = joint_point(robot)
                    except Exception as feedback_exc:
                        report["joint_after_error_error"] = str(feedback_exc)
                    raise
                finally:
                    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                    encoding="utf-8")
                    print("诊断记录：", path, flush=True)
            finally:
                signal.signal(signal.SIGINT, previous_int)
                signal.signal(signal.SIGTERM, previous_term)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError,
            can.CanError) as exc:
        print(f"J4/J7 微小联动测试未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
