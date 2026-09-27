#!/usr/bin/env python3
"""现场手动拖动到完整会话的最后一个记录点；不会自动回位。"""

import argparse
import math
from pathlib import Path
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import MOUNT_CODES, fresh, send_frame, wait_status
from experiments.single_arm.can_setup.drag_start_guard import require_drag_start_safe
from experiments.single_arm.teaching.return_session import load_recording
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, check_drivers, healthy_preteach_status, joint_point,
    load_config, stop_drag, validate_joints,
)


def main():
    """检查记录末端姿态并进入人工拖动恢复流程，不发送自动回位路线。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true", help="进入拖动并在接近记录末端后结束；默认只读")
    parser.add_argument("--max-seconds", type=float, default=90.0)
    args = parser.parse_args()
    if not 1 <= args.max_seconds <= 180:
        parser.error("--max-seconds 必须在 1 到 180 秒之间")

    try:
        if args.run:
            require_drag_start_safe()
        config = load_config(args.config)
        _, _, _, recorded = load_recording(args.recording, config)
        target = recorded[-1]
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, lambda item: fresh(item), timeout=3.0) is None:
                raise RuntimeError("机械臂状态反馈缺失")
            healthy_preteach_status(robot)
            check_drivers(robot)
            current = joint_point(robot)
            print("当前关节角：", [round(value, 5) for value in current])
            print("记录末端：", [round(value, 5) for value in target])
            print(f"末端关节空间距离：{math.dist(current, target):.4f} rad")
            if not args.run:
                print("只读检查完成。--run 会进入拖动模式，由现场人员手动调整。")
                return 0

            if math.dist(current, target) > 0.10:
                raise RuntimeError("当前位置距记录末端超过 0.10 rad，拒绝进入拖动恢复")
            if current[3] < math.radians(-59.5):
                raise RuntimeError("J4 已过于接近用户指南下限，拒绝进入拖动")
            start_joint = current
            reached = False
            with can.Bus(channel=config["channel"], interface="socketcan", local_loopback=True) as bus:
                baseline = robot.get_arm_status()
                if not fresh(baseline):
                    raise RuntimeError("控制模式反馈缺失")
                baseline_timestamp = float(baseline.timestamp)
                send_frame(bus, 0x151, [1, 0xFF, 50, 0, 0, MOUNT_CODES[config["mount"]], 0, 0])
                ready = wait_status(
                    robot,
                    lambda item: fresh(item) and float(item.timestamp) > baseline_timestamp
                    and int(item.msg.ctrl_mode) == 1 and int(item.msg.arm_status) == 0
                    and item.msg.err_code == 0,
                    timeout=3.0,
                )
                if ready is None:
                    latest = robot.get_arm_status()
                    detail = (
                        f"最新状态={latest.msg.ctrl_mode}/{latest.msg.arm_status}/"
                        f"{latest.msg.teach_status}/错误码0x{latest.msg.err_code:04X}；"
                        f"状态时间戳={latest.timestamp:.6f}，基准={baseline_timestamp:.6f}"
                        if fresh(latest) else "最新状态反馈缺失"
                    )
                    raise RuntimeError(f"未确认 CAN 控制状态（{detail}）；未进入拖动")

                started = True
                try:
                    send_frame(bus, 0x150, [0, 0, 1, 0, 0, 0, 0, 0])
                    entered = wait_status(
                        robot, lambda item: fresh(item) and int(item.msg.teach_status) == 1,
                        timeout=3.0,
                    )
                    if entered is None:
                        raise RuntimeError("未确认进入拖动示教")
                    deadline = time.monotonic() + args.max_seconds
                    stable_since = None
                    stable_anchor = None
                    next_report = 0.0
                    print("已进入拖动；请扶稳机械臂，将 J4 缓慢往角度增大方向移动。", flush=True)
                    while time.monotonic() < deadline:
                        status = robot.get_arm_status()
                        if (not fresh(status) or status.msg.err_code
                                or int(status.msg.teach_status) != 1):
                            raise RuntimeError("拖动状态异常或反馈中断")
                        current = joint_point(robot)
                        if current[3] < math.radians(-59.5):
                            raise RuntimeError("J4 继续靠近用户指南下限，已结束拖动")
                        distance = math.dist(current, target)
                        if current[3] > target[3] + 0.05:
                            raise RuntimeError("J4 已超过目标角度 0.05 rad，结束拖动")
                        if distance > 0.15 or math.dist(current, start_joint) > 0.15:
                            raise RuntimeError("关节姿态偏离目标恢复范围，结束拖动")
                        now = time.monotonic()
                        if now >= next_report:
                            print(f"J4={current[3]:.4f} rad；与记录末端距离={distance:.4f} rad", flush=True)
                            next_report = now + 2.0
                        if distance <= 0.03:
                            try:
                                validate_joints(current, config["zero_exclusion_radius_rad"])
                            except ValueError:
                                stable_since = None
                                stable_anchor = None
                            else:
                                if (stable_anchor is None or
                                        max(abs(a - b) for a, b in zip(current, stable_anchor)) > 0.002):
                                    stable_anchor = current
                                    stable_since = now
                                elif now - stable_since >= 1.0:
                                    reached = True
                                    print("已在记录末端附近保持 1 秒，结束拖动。", flush=True)
                                    break
                        else:
                            stable_since = None
                            stable_anchor = None
                        time.sleep(0.1)
                finally:
                    if started:
                        stop_drag(bus, robot)

            time.sleep(0.5)
            current = validate_joints(joint_point(robot), config["zero_exclusion_radius_rad"])
            if not reached:
                raise RuntimeError("未在时限内稳定接近记录末端；已退出拖动，未回位")
            print("停稳后仍与记录末端吻合；本脚本未发送回位运动指令。", flush=True)
            print("回 S1 请从实时姿态运行 lab.cli return plan/run。", flush=True)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, can.CanError) as exc:
        print(f"手动恢复未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        print("手动恢复已中断；已尝试结束拖动。", file=sys.stderr, flush=True)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
