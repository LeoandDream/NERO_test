#!/usr/bin/env python3
"""Record read-only Nero joint and flange feedback to a timestamped CSV."""

import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import signal
import sys
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config


JOINT_COLUMNS = [f"joint_{index}_rad" for index in range(1, 8)]
POSE_COLUMNS = ["x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"]
STATUS_COLUMNS = ["control_mode", "arm_status", "teach_status", "motion_status"]
# 列名带单位，便于后续筛选、FK 校验和控制脚本读取原始记录。
FIELDNAMES = [
    "sample_index",
    "captured_at_utc",
    "elapsed_s",
    "joint_feedback_unix_s",
    "pose_feedback_unix_s",
    "status_feedback_unix_s",
    *STATUS_COLUMNS,
    *JOINT_COLUMNS,
    *POSE_COLUMNS,
]


def default_output() -> Path:
    """用 UTC 时间戳生成新的采集文件名，避免覆盖历史记录。"""
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return Path("experiments/single_arm/pose_recording/data/recordings") / f"nero_poses_{stamp}.csv"


def main() -> int:
    """周期读取新鲜的关节、法兰和状态反馈并写入 CSV；超时或信号时关闭文件。"""
    parser = argparse.ArgumentParser(description="连续记录 Nero 的关节角与法兰位姿")
    parser.add_argument("--channel", default="can0", help="CAN 接口，默认 can0")
    parser.add_argument("--profile", default=NeroFW.V121,
                        choices=[NeroFW.DEFAULT, NeroFW.V111, NeroFW.V112,
                                 NeroFW.V120, NeroFW.V121],
                        help="固件档位，默认 v121（当前机械臂）")
    parser.add_argument("--rate", type=float, default=10.0, help="采样频率 Hz，默认 10")
    parser.add_argument("--duration", type=float, default=1800.0,
                        help="最长录制秒数，默认 1800（30 分钟）")
    parser.add_argument("--max-age", type=float, default=1.0,
                        help="允许的 CAN 反馈最大延迟秒数，默认 1")
    parser.add_argument("--output", type=Path, default=None, help="CSV 输出路径")
    args = parser.parse_args()

    if args.rate <= 0 or args.duration <= 0 or args.max_age <= 0:
        parser.error("--rate、--duration 和 --max-age 必须大于 0")

    output = args.output or default_output()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        parser.error(f"输出文件已存在：{output}")

    config = create_agx_arm_config(
        robot=ArmModel.NERO,
        firmeware_version=args.profile,
        channel=args.channel,
    )
    robot = AgxArmFactory.create_arm(config)
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    count = 0
    start = time.monotonic()
    next_tick = start
    next_status = start + 5.0

    try:
        robot.connect()
        with output.open("x", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=FIELDNAMES)
            writer.writeheader()
            stream.flush()
            print(f"开始记录到 {output}；按 Ctrl+C 停止。", flush=True)

            while not stop_requested and time.monotonic() - start < args.duration:
                now = time.time()
                joints = robot.get_joint_angles()
                pose = robot.get_flange_pose()
                status = robot.get_arm_status()

                # 关节和法兰反馈都有效且未过期才落盘；状态列允许暂时为空。
                if joints is not None and pose is not None:
                    joint_values = list(joints.msg)
                    pose_values = list(pose.msg)
                    joint_time = float(joints.timestamp)
                    pose_time = float(pose.timestamp)

                    if (len(joint_values) == 7 and len(pose_values) == 6
                            and 0 <= now - joint_time <= args.max_age
                            and 0 <= now - pose_time <= args.max_age):
                        row = {
                            "sample_index": count,
                            "captured_at_utc": datetime.fromtimestamp(
                                now, timezone.utc).isoformat(timespec="milliseconds"),
                            "elapsed_s": round(time.monotonic() - start, 3),
                            "joint_feedback_unix_s": joint_time,
                            "pose_feedback_unix_s": pose_time,
                            "status_feedback_unix_s": "",
                        }
                        if (status is not None
                                and 0 <= now - float(status.timestamp) <= args.max_age):
                            row["status_feedback_unix_s"] = float(status.timestamp)
                            for column, attribute in zip(
                                STATUS_COLUMNS,
                                ("ctrl_mode", "arm_status", "teach_status", "motion_status"),
                            ):
                                value = getattr(status.msg, attribute, None)
                                row[column] = getattr(value, "name", str(value)) if value is not None else ""
                        row.update(zip(JOINT_COLUMNS, joint_values))
                        row.update(zip(POSE_COLUMNS, pose_values))
                        writer.writerow(row)
                        stream.flush()
                        count += 1

                if time.monotonic() >= next_status:
                    status = f"已记录 {count} 条" if count else "等待有效的关节角和位姿反馈"
                    print(status, flush=True)
                    next_status = time.monotonic() + 5.0

                # 用单调时钟推进固定采样节拍，避免 CSV 写入耗时持续累积漂移。
                next_tick += 1.0 / args.rate
                time.sleep(max(0.0, next_tick - time.monotonic()))
    except Exception as exc:
        print(f"记录失败：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        robot.disconnect()

    print(f"记录结束：{count} 条，文件：{output}", flush=True)
    return 0 if count else 2


if __name__ == "__main__":
    raise SystemExit(main())
