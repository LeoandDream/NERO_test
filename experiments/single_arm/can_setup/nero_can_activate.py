#!/usr/bin/env python3
"""通过 CAN 检查 Nero 状态；可显式选择使能关节。"""

import argparse
import time

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--channel", default="can0", help="SocketCAN 通道（默认 can0）")
parser.add_argument(
    "--enable",
    action="store_true",
    help="发送关节使能命令；刹车释放时机械臂可能运动",
)
args = parser.parse_args()

# 固件档位来自先前的只读版本查询；更换控制器后须重新核对。
config = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V121,
    channel=args.channel,
    local_loopback=True,
)
robot = AgxArmFactory.create_arm(config)
robot.connect()
try:
    deadline = time.monotonic() + 3.0
    status = robot.get_arm_status()
    while status is None and time.monotonic() < deadline:
        time.sleep(0.1)
        status = robot.get_arm_status()

    if status is None:
        print("未收到机械臂状态；检查 can0、接线和供电。")
        raise SystemExit(1)

    print("控制模式：", status.msg.ctrl_mode)
    print("机械臂状态：", status.msg.arm_status)
    print("示教状态：", status.msg.teach_status)
    print(f"关节错误码：0x{status.msg.err_code:04X}")
    print("关节驱动状态：")
    # CAN 驱动反馈异步到达，短暂轮询收齐七个关节后再评估使能条件。
    drivers = [None] * 7
    driver_deadline = time.monotonic() + 2.0
    while any(driver is None for driver in drivers) and time.monotonic() < driver_deadline:
        for index in range(7):
            if drivers[index] is None:
                drivers[index] = robot.get_driver_states(index + 1)
        if any(driver is None for driver in drivers):
            time.sleep(0.05)
    for joint in range(1, 8):
        driver = drivers[joint - 1]
        if driver is None:
            print(f"  关节 {joint}: 无反馈")
            continue
        foc = driver.msg.foc_status
        print(
            f"  关节 {joint}: 电压={driver.msg.vol:.1f}V, "
            f"使能={foc.driver_enable_status}, "
            f"欠压={foc.voltage_too_low}, "
            f"驱动故障={foc.driver_error_status}"
        )
    # 缺少显式 --enable 时始终止于只读状态，不调用 SDK 的使能命令。
    if not args.enable:
        all_driver_enable_bits = all(
            driver is not None and driver.msg.foc_status.driver_enable_status
            for driver in drivers
        )
        arm_status = int(status.msg.arm_status)
        if arm_status == 1:
            # 急停可采用阻尼停止；驱动使能位为 True 并不代表仍在保持位置。
            print("只读检查完成；机械臂仍在急停，不要据驱动使能位判断可以运动或保持位置。")
        elif arm_status == 6:
            # 复位后常见的待使能状态；这里仍只报告状态，不自动释放刹车。
            print("只读检查完成；关节刹车未释放，当前不能执行运动。")
        elif arm_status != 0:
            print("只读检查完成；机械臂报告其他异常状态，当前不能执行运动。")
        elif all_driver_enable_bits:
            print("只读检查完成；七个驱动使能位均为 True，机械臂状态正常。")
        else:
            print("只读检查完成；开机红灯及关节刹车未释放可表示尚未使能。")
            print("确认机械臂固定且周围无人、无障碍后，可显式使用 --enable。")
        raise SystemExit(0)

    # 欠压、驱动故障、示教进行中或状态不明时，不尝试释放关节刹车。
    if int(status.msg.arm_status) not in (0, 6):
        print("机械臂报告其他异常，已取消使能。")
        raise SystemExit(2)
    if status.msg.err_code or int(status.msg.teach_status) != 0:
        print("关节错误码非零或示教正在进行，已取消使能。")
        raise SystemExit(2)
    if any(driver is None for driver in drivers):
        print("关节驱动反馈不完整，已取消使能。")
        raise SystemExit(2)
    if any(
        driver.msg.foc_status.voltage_too_low
        or driver.msg.foc_status.driver_error_status
        for driver in drivers
    ):
        print("关节驱动报告欠压或故障，已取消使能。")
        raise SystemExit(2)

    print("发送一次 CAN 使能命令；请观察机械臂及指示灯。")
    status_timestamp = float(status.timestamp)
    enabled = robot.enable(timeout=1.5)
    print("关节使能反馈：", robot.get_joints_enable_status_list())
    # 使能反馈和整机状态通过不同 CAN 帧异步更新。必须确认比使能前更新的
    # NORMAL 状态，不能将驱动使能位 True 或旧的 0x6 缓存当成可运动状态。
    normal_status = wait_status(
        robot,
        lambda item: fresh(item) and float(item.timestamp) > status_timestamp
        and int(item.msg.arm_status) == 0 and int(item.msg.err_code) == 0,
        timeout=3.0,
    )
    new_status = robot.get_arm_status()
    print("使能后机械臂状态：", new_status.msg.arm_status if fresh(new_status) else "无新鲜反馈")
    if not enabled:
        print("未确认所有关节使能；不要发送运动指令，请检查 CAN 总线和状态反馈。")
        raise SystemExit(2)
    if normal_status is None:
        print("驱动已报告使能，但未确认整机恢复 NORMAL；不要发送运动指令，也不要盲目重复使能。")
        raise SystemExit(2)
    print("七个关节均已使能，整机状态 NORMAL；请现场确认指示灯和机械臂姿态。")
finally:
    robot.disconnect()
