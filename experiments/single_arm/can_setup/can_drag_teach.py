#!/usr/bin/env python3
"""通过 Nero CAN 协议单次进入或退出示教；限时回位请用 teach_session.py。"""

import argparse
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.drag_start_guard import require_drag_start_safe


# 安装码参与重力补偿；必须与物理安装方向一致。
MOUNT_CODES = {"horizontal": 0x01, "left": 0x02, "right": 0x03}


def wait_status(robot, predicate, timeout=3.0):
    """在限定时间内轮询最新控制器状态，直到满足指定条件。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = robot.get_arm_status()
        if status is not None and predicate(status):
            return status
        time.sleep(0.05)
    return None


def fresh(message, max_age=1.0):
    """拒绝空消息和超过最大年龄的 CAN 缓存反馈。"""
    return message is not None and 0 <= time.time() - float(message.timestamp) <= max_age


def send_frame(bus, can_id, data, *, drag_authorization=None):
    """构造并发送一个标准 8 字节 Nero CAN 控制帧。"""
    if can_id == 0x150 and len(data) >= 3 and data[2] == 1:
        require_drag_start_safe(drag_authorization)
    if (can_id == 0x151 and len(data) >= 6 and data[0] == 1
            and data[1] == 0xFF and data[5] in MOUNT_CODES.values()):
        require_drag_start_safe(drag_authorization)
    bus.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=False), timeout=0.5)


def main():
    """先读状态，再按显式参数开始或结束拖动；默认不改变模式。"""
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group()
    action.add_argument("--start", action="store_true", help="开始拖动示教；机械臂可能改变姿态")
    action.add_argument("--stop", action="store_true", help="结束拖动示教")
    parser.add_argument("--mount", choices=MOUNT_CODES, help="机械臂实际安装方向，--start 时必填")
    parser.add_argument("--channel", default="can0", help="SocketCAN 通道，默认 can0")
    args = parser.parse_args()
    if args.start and args.mount is None:
        parser.error("--start 必须同时指定 --mount")
    if args.mount and not args.start:
        parser.error("--mount 仅与 --start 一起使用")
    if args.start:
        try:
            require_drag_start_safe()
        except RuntimeError as exc:
            print(exc)
            return 2

    config = create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=args.channel, local_loopback=True
    )
    robot = AgxArmFactory.create_arm(config)
    robot.connect()
    try:
        status = wait_status(robot, lambda item: fresh(item))
        if status is None:
            print("没有收到新鲜的机械臂状态反馈。")
            if not args.stop:
                print("未发送命令。")
                return 2
        else:
            print("控制模式：", status.msg.ctrl_mode)
            print("机械臂状态：", status.msg.arm_status)
            print("示教状态：", status.msg.teach_status)
            print(f"关节错误码：0x{status.msg.err_code:04X}")

        # 裸命令只查询状态；开始和结束都要求命令行显式给出动作参数。
        if not args.start and not args.stop:
            print("只读检查完成；未发送 CAN 命令。")
            return 0

        with can.Bus(channel=args.channel, interface="socketcan", local_loopback=True) as bus:
            if args.stop:
                # 0x150 Byte 2 = 0x02: 结束拖动示教记录。
                send_frame(bus, 0x150, [0, 0, 2, 0, 0, 0, 0, 0])
                stopped = wait_status(
                    robot,
                    lambda item: fresh(item) and int(item.msg.teach_status) in (0, 2, 6),
                )
                print("结束指令已发送；示教状态：", stopped.msg.teach_status if stopped else "未确认")
                return 0 if stopped else 2

            idle_teach = (int(status.msg.teach_status) in (0, 2)
                          or (int(status.msg.ctrl_mode) == 1
                              and int(status.msg.teach_status) == 6))
            if (int(status.msg.ctrl_mode) not in (1, 2) or int(status.msg.arm_status) != 0
                    or not idle_teach or status.msg.err_code):
                print("机械臂未处于无故障的 CAN/空闲示教状态；未进入拖动示教。")
                return 2
            drivers = [robot.get_driver_states(index) for index in range(1, 8)]
            if any(not fresh(driver) for driver in drivers):
                print("关节驱动反馈不完整；未进入拖动示教。")
                return 2
            if any(
                not driver.msg.foc_status.driver_enable_status
                or driver.msg.foc_status.voltage_too_low
                or driver.msg.foc_status.driver_error_status
                for driver in drivers
            ):
                print("有未使能、欠压或故障的关节；未进入拖动示教。")
                return 2

            # 0x151 Byte 5 = 安装位置；Byte 1 = 0xFF 表示保持原运动模式，
            # 与 pyAgxArm set_leader_mode() 中的模式帧一致。Byte 6 = 0 不改 CAN 推送。
            mount_code = MOUNT_CODES[args.mount]
            # 用更新后的状态帧确认安装码设置，避免误把切换前缓存当作成功。
            baseline_timestamp = float(status.timestamp)
            send_frame(bus, 0x151, [1, 0xFF, 50, 0, 0, mount_code, 0, 0])
            updated = wait_status(
                robot,
                lambda item: fresh(item) and float(item.timestamp) > baseline_timestamp,
            )
            if updated is None or updated.msg.err_code or int(updated.msg.arm_status) != 0:
                print("设置安装方向后未确认正常状态；未进入拖动示教。")
                return 2

            print(f"安装方向已按 {args.mount} (0x{mount_code:02X}) 下发；发送拖动示教开始指令。")
            # 0x150 Byte 2 = 0x01: 开始拖动示教记录。
            send_frame(bus, 0x150, [0, 0, 1, 0, 0, 0, 0, 0])
            teaching = wait_status(
                robot,
                lambda item: fresh(item)
                and (int(item.msg.teach_status) == 1 or int(item.msg.arm_status) == 11),
            )
            if teaching is None:
                send_frame(bus, 0x150, [0, 0, 2, 0, 0, 0, 0, 0])
                print("未确认进入拖动示教，已发送结束指令；请检查机械臂状态。")
                return 2

            time.sleep(0.3)
            if not (fresh(robot.get_joint_angles()) and fresh(robot.get_flange_pose())):
                send_frame(bus, 0x150, [0, 0, 2, 0, 0, 0, 0, 0])
                print("拖动模式下关节角或位姿反馈中断，已发送结束指令。")
                return 2
            print("已进入拖动示教，关节角和法兰位姿反馈正常。")
            print("结束时执行：python -m experiments.single_arm.can_setup.can_drag_teach --stop")
            print("此单次命令不自动结束或回位；限时会话使用 python -m experiments.single_arm.teaching.teach_session --run。")
            return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
