#!/usr/bin/env python3
"""只读记录整机反馈及急停、使能、运动模式 CAN 帧，诊断突然失能。"""

import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh


OUTPUT_DIR = Path("experiments/single_arm/can_setup/data/monitoring")
MAX_DURATION_S = 43200.0  # 最长 12 小时，用于覆盖上位机进程退出后的状态变化。


def control_event(arbitration_id, data):
    """解释 SDK 已定义的急停、模式和使能帧；其余字节保留原始值。"""
    if arbitration_id == 0x150 and len(data) >= 1:
        return {1: "electronic_emergency_stop", 2: "reset"}.get(data[0], "other_motion_control")
    if arbitration_id == 0x471 and len(data) >= 2:
        return {1: "disable", 2: "enable"}.get(data[1], "other_motor_control")
    if arbitration_id == 0x151 and len(data) >= 2:
        if data[1] == 255:
            return "mode_unchanged_or_speed_config"
        return {0: "move_p_mode", 1: "move_j_mode", 2: "move_l_mode",
                3: "move_c_mode"}.get(data[1], "other_mode_control")
    return "unknown"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--duration", type=float, default=300.0,
                        help="记录秒数，默认 300；Ctrl+C 可提前结束")
    parser.add_argument("--rate", type=float, default=2.0, help="每秒采样次数，默认 2")
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not math.isfinite(args.rate)
            or not 1 <= args.duration <= MAX_DURATION_S or not 0.2 <= args.rate <= 10):
        parser.error("duration 必须 1～43200 秒，rate 必须 0.2～10 Hz")
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=args.channel
    ))
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    session = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    path = OUTPUT_DIR / f"nero_state_{session}.csv"
    frame_path = OUTPUT_DIR / f"nero_control_frames_{session}.csv"
    fields = ["time_utc", "elapsed_s", "arm_status", "motion_status", "control_mode",
              "teach_status", "error_code"]
    fields += [f"joint_{i}_rad" for i in range(1, 8)]
    fields += [f"joint_{i}_enabled" for i in range(1, 8)]
    fields += [f"joint_{i}_voltage_v" for i in range(1, 8)]
    frame_count = 0
    # 第二个 SocketCAN 套接字只接收帧，不发送命令。先开始抓帧，避免
    # connect() 到首条状态样本之间出现诊断盲区。
    filters = [
        {"can_id": frame_id, "can_mask": 0x7FF, "extended": False}
        for frame_id in (0x150, 0x151, 0x471)
    ]
    with can.Bus(interface="socketcan", channel=args.channel,
                 receive_own_messages=False, can_filters=filters) as bus:
        with frame_path.open("x", newline="", encoding="utf-8") as frame_stream:
            frame_writer = csv.DictWriter(frame_stream, fieldnames=(
                "time_utc", "can_id", "data_hex", "event"
            ))
            frame_writer.writeheader()
            frame_stream.flush()

            def record_control_frame(message):
                nonlocal frame_count
                if message.is_error_frame or message.is_remote_frame or message.is_extended_id:
                    return
                if message.arbitration_id not in (0x150, 0x151, 0x471):
                    return
                frame_writer.writerow({
                    "time_utc": datetime.fromtimestamp(message.timestamp, timezone.utc).isoformat(),
                    "can_id": f"0x{message.arbitration_id:03X}",
                    "data_hex": bytes(message.data).hex().upper(),
                    "event": control_event(message.arbitration_id, message.data),
                })
                frame_stream.flush()
                frame_count += 1

            notifier = can.Notifier(bus, [record_control_frame], timeout=0.2)
            try:
                robot.connect()
                print(f"只读 CAN 控制帧记录：{frame_path}", flush=True)
                count = record_status(robot, path, fields, args, notifier)
                if notifier.exception is not None:
                    raise RuntimeError("CAN 控制帧监听已停止；诊断记录不完整") from notifier.exception
            finally:
                try:
                    robot.disconnect()
                finally:
                    notifier.stop()
    print(f"记录结束：{count} 条状态，{frame_count} 条控制帧。", flush=True)
    print(f"状态文件：{path}", flush=True)
    print(f"控制帧文件：{frame_path}", flush=True)


def record_status(robot, path, fields, args, notifier):
    """每行即时落盘；中断时保留已采到的状态和 CAN 帧。"""
    count = 0
    try:
        with path.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            stream.flush()
            print(f"只读状态记录：{path}；按 Ctrl+C 提前结束。", flush=True)
            begin = time.monotonic()
            next_sample = begin
            while time.monotonic() - begin < args.duration:
                if notifier.exception is not None:
                    raise RuntimeError("CAN 控制帧监听已停止；诊断记录不完整") from notifier.exception
                now = time.monotonic()
                if now < next_sample:
                    time.sleep(next_sample - now)
                next_sample += 1.0 / args.rate
                status = robot.get_arm_status()
                joints = robot.get_joint_angles()
                row = {"time_utc": datetime.now(timezone.utc).isoformat(),
                       "elapsed_s": round(time.monotonic() - begin, 3)}
                if fresh(status):
                    row.update({
                        "arm_status": str(status.msg.arm_status),
                        "motion_status": str(status.msg.motion_status),
                        "control_mode": str(status.msg.ctrl_mode),
                        "teach_status": str(status.msg.teach_status),
                        "error_code": f"0x{status.msg.err_code:04X}",
                    })
                if fresh(joints) and len(joints.msg) == 7:
                    row.update({f"joint_{i}_rad": value for i, value in enumerate(joints.msg, 1)})
                for index in range(1, 8):
                    driver = robot.get_driver_states(index)
                    if fresh(driver):
                        row[f"joint_{index}_enabled"] = bool(driver.msg.foc_status.driver_enable_status)
                        row[f"joint_{index}_voltage_v"] = float(driver.msg.vol)
                writer.writerow(row)
                stream.flush()  # 会话突然结束时，已采到的行仍尽量留在文件中。
                count += 1
    except KeyboardInterrupt:
        print("收到 Ctrl+C，结束只读记录。", flush=True)
    return count


if __name__ == "__main__":
    main()
