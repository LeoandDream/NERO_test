#!/usr/bin/env python3
"""只读检查 Nero 电子急停；显式 --run 时仅发送一次 reset，不自动使能或运动。"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status


REPORT_DIR = Path("experiments/single_arm/can_setup/data/recoveries")
EMERGENCY_STOP = 1
STABILITY_SECONDS = 5.0
STABILITY_POLL_SECONDS = 0.5


def snapshot(robot):
    """只读取新鲜的控制器、关节和法兰反馈，拒绝缓存或非有限数值。"""
    status = wait_status(robot, fresh, timeout=3.0)
    if status is None:
        raise RuntimeError("控制器状态反馈缺失或过期")
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        joints = robot.get_joint_angles()
        flange = robot.get_flange_pose()
        if fresh(joints) and fresh(flange):
            q = [float(value) for value in joints.msg]
            pose = [float(value) for value in flange.msg]
            if len(q) != 7 or len(pose) != 6 or not all(
                math.isfinite(value) for value in [*q, *pose]
            ):
                raise RuntimeError("关节或法兰反馈格式错误")
            return {
                "captured_at_utc": datetime.now(timezone.utc).isoformat(),
                "control_mode": int(status.msg.ctrl_mode),
                "arm_status": int(status.msg.arm_status),
                "teach_status": int(status.msg.teach_status),
                "joint_error_code": int(status.msg.err_code),
                "joint_rad": q,
                "flange_pose": pose,
            }
        time.sleep(0.05)
    raise RuntimeError("关节或法兰反馈缺失或过期")


def check_before_reset(first, second):
    """复位前只接受静止且无附加错误的 CAN 急停状态。"""
    if (first["control_mode"], first["arm_status"], first["teach_status"],
            first["joint_error_code"]) != (1, EMERGENCY_STOP, 0, 0):
        raise RuntimeError("不是无附加故障的 CAN 急停状态；拒绝复位")
    if (second["control_mode"], second["arm_status"], second["teach_status"],
            second["joint_error_code"]) != (1, EMERGENCY_STOP, 0, 0):
        raise RuntimeError("复核时状态已变化；拒绝复位")
    drift = max(abs(a - b) for a, b in zip(first["joint_rad"], second["joint_rad"]))
    if drift > 0.002:
        raise RuntimeError(f"机械臂仍在移动：关节变化 {drift:.4f} rad；拒绝复位")
    return drift


def observe_stability(robot, seconds=STABILITY_SECONDS):
    """在数秒内反复读取姿态，避免将急停阻尼下落误判为静止。"""
    first = snapshot(robot)
    last = first
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        time.sleep(min(STABILITY_POLL_SECONDS, max(0.0, deadline - time.monotonic())))
        last = snapshot(robot)
        check_before_reset(first, last)
    return first, last


def main():
    """先核对状态与静止姿态；--run 倒计时后单次复位并记录新状态。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0", help="SocketCAN 通道，默认 can0")
    parser.add_argument("--run", action="store_true", help="发送一次 reset；可能立即失电下落")
    args = parser.parse_args()

    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=args.channel, local_loopback=True
    ))
    robot.connect()
    try:
        first, second = observe_stability(robot)
        drift = check_before_reset(first, second)
        print("当前状态：EMERGENCY_STOP；错误码 0；示教未启用。")
        print("当前关节角 (rad)：", second["joint_rad"])
        print("当前法兰位姿：", second["flange_pose"])
        print(f"{STABILITY_SECONDS:g} 秒最大关节变化：{drift:.6f} rad")
        if not args.run:
            print("只读检查完成；未发送复位、使能或运动指令。")
            return 0

        print("注意：reset 会使抬起的关节立即失电下落；须由现场可靠支撑并避开夹点。")
        for remaining in range(5, 0, -1):
            print(f"{remaining} 秒后发送一次 reset；按 Ctrl+C 取消。", flush=True)
            time.sleep(1)
        # 倒计时期间若机械臂移动或状态变化，绝不能沿用之前的检查结果。
        _, before = observe_stability(robot)
        check_before_reset(second, before)
        output = REPORT_DIR / (
            "nero_emergency_recovery_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        report = {"before": before, "reset_attempted": False, "reset_sent": False, "after": None}
        with output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        print(f"恢复记录：{output}", flush=True)

        # 先记录尝试，再调用 SDK。若发送时抛异常，记录明确表示结果未知，
        # 操作者应只读检查状态，不能因为异常就再次发送复位。
        report["reset_attempted"] = True
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        # reset 是此脚本唯一的控制指令。即使反馈确认失败，也不自动重发。
        robot.reset()
        report["reset_sent"] = True
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print("已发送一次 reset；不自动使能或运动。", flush=True)
        time.sleep(1.0)
        try:
            report["after"] = snapshot(robot)
            print("复位后机械臂状态码：", report["after"]["arm_status"])
            print("复位后关节角 (rad)：", report["after"]["joint_rad"])
        except RuntimeError as exc:
            report["after_error"] = str(exc)
            print(f"复位后的只读反馈未确认：{exc}；请勿重复发送 reset。", file=sys.stderr)
        finally:
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if report["after"] is None:
            return 2
        if report["after"]["arm_status"] == EMERGENCY_STOP:
            print("复位后仍报告急停；请勿重复发送 reset，先核对现场状态。", file=sys.stderr)
            return 2
        return 0
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print(f"恢复检查未完成：{exc}", file=sys.stderr)
        return 1
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
