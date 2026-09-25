#!/usr/bin/env python3
"""只读检查 J7 的关节、驱动和电机反馈；不发送运动或复位指令。"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh


REPORT_DIR = Path("experiments/single_arm/start_transfer/data/diagnostics")


def read_fresh(robot, getter, name, timeout=3.0):
    """等待指定 CAN 反馈更新；过期缓存不能用作当前状态。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        message = getter()
        if fresh(message):
            return message
        time.sleep(0.05)
    raise RuntimeError(f"{name} 反馈缺失或过期")


def sample(robot):
    """采集同一时间附近的状态、七轴角度及 J7 电机和驱动反馈。"""
    status = read_fresh(robot, robot.get_arm_status, "整机状态")
    joints = read_fresh(robot, robot.get_joint_angles, "关节角")
    driver = read_fresh(robot, lambda: robot.get_driver_states(7), "J7 驱动")
    motor = read_fresh(robot, lambda: robot.get_motor_states(7), "J7 电机")
    q = [float(value) for value in joints.msg]
    if len(q) != 7 or not all(math.isfinite(value) for value in q):
        raise RuntimeError("关节反馈格式错误")
    foc = driver.msg.foc_status
    data = {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": {
            "control_mode": int(status.msg.ctrl_mode),
            "arm_status": int(status.msg.arm_status),
            "teach_status": int(status.msg.teach_status),
            "motion_status": str(status.msg.motion_status),
            "joint_error_code": int(status.msg.err_code),
        },
        "joint_rad": q,
        "j7_driver": {
            "voltage_v": float(driver.msg.vol),
            "drive_temperature_c": float(driver.msg.foc_temp),
            "motor_temperature_c": float(driver.msg.motor_temp),
            "bus_current_a": float(driver.msg.bus_current),
            "enable": bool(foc.driver_enable_status),
            "undervoltage": bool(foc.voltage_too_low),
            "motor_overheating": bool(foc.motor_overheating),
            "drive_overcurrent": bool(foc.driver_overcurrent),
            "drive_overheating": bool(foc.driver_overheating),
            "collision": bool(foc.collision_status),
            "drive_error": bool(foc.driver_error_status),
            "stall": bool(foc.stall_status),
        },
        "j7_motor": {
            "position_rad": float(motor.msg.position),
            "velocity_rad_s": float(motor.msg.velocity),
            "current_a": float(motor.msg.current),
            "torque_nm": float(motor.msg.torque),
        },
        "feedback_unix_s": {
            "status": float(status.timestamp),
            "joints": float(joints.timestamp),
            "j7_driver": float(driver.timestamp),
            "j7_motor": float(motor.timestamp),
        },
    }
    values = [*data["j7_motor"].values(), *(
        data["j7_driver"][key] for key in (
            "voltage_v", "drive_temperature_c", "motor_temperature_c", "bus_current_a"
        )
    )]
    if not all(math.isfinite(value) for value in values):
        raise RuntimeError("J7 电机或驱动反馈含非有限数值")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()
    if not 2 <= args.samples <= 20 or not 0.1 <= args.interval <= 2.0:
        parser.error("--samples 需为 2～20；--interval 需为 0.1～2 秒")

    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=args.channel
    ))
    try:
        robot.connect()
        samples = []
        for index in range(args.samples):
            samples.append(sample(robot))
            if index + 1 < args.samples:
                time.sleep(args.interval)
    except (OSError, ValueError, RuntimeError, can.CanError) as exc:
        print(f"J7 只读诊断未完成：{exc}", file=sys.stderr)
        return 1
    finally:
        robot.disconnect()

    drift = max(
        max(abs(a - b) for a, b in zip(samples[0]["joint_rad"], item["joint_rad"]))
        for item in samples
    )
    report = {
        "purpose": "read_only_j7_diagnostics",
        "samples": samples,
        "max_joint_change_from_first_rad": drift,
    }
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    path = REPORT_DIR / (
        "nero_j7_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
    )
    with path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("只读 J7 诊断：", path)
    print("状态：", samples[-1]["status"])
    print("当前关节角 (rad)：", samples[-1]["joint_rad"])
    print("J7 驱动：", samples[-1]["j7_driver"])
    print("J7 电机：", samples[-1]["j7_motor"])
    print(f"采样期间最大关节变化：{drift:.6f} rad")
    print("未发送使能、复位或运动指令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
