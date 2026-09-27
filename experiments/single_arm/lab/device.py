"""只读实时状态和可注入的设备工厂；导入模块不连接 CAN。"""

from contextlib import contextmanager
from datetime import datetime, timezone
import math
import time


def default_robot_factory(channel):
    from experiments.single_arm.continuous_replay.continuous_session import robot_instance
    return robot_instance(channel)


@contextmanager
def connected(config, robot_factory=None):
    factory = robot_factory or default_robot_factory
    robot = factory(config["channel"])
    robot.connect()
    try:
        yield robot
    finally:
        robot.disconnect()


def read_state(robot, *, require_stable=True):
    """只读反馈；返回七轴与各故障标志，不发送使能或运动命令。"""
    from experiments.single_arm.can_setup.can_drag_teach import fresh
    # SDK 建立连接后会先填充缓存；给实时流最多 3 秒，不用刚连接时的空缓存判故障。
    deadline = time.monotonic() + 3.0
    while True:
        status = robot.get_arm_status()
        joints = robot.get_joint_angles()
        flange = robot.get_flange_pose()
        if all(fresh(item, 0.25) for item in (status, joints, flange)):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("控制器、关节或法兰反馈缺失/过期")
        time.sleep(0.05)
    first = [float(value) for value in joints.msg]
    pose = [float(value) for value in flange.msg]
    if len(first) != 7 or len(pose) != 6 or not all(math.isfinite(x) for x in first+pose):
        raise RuntimeError("实时关节或法兰反馈无效")
    drivers = []
    for index in range(1, 8):
        while True:
            item = robot.get_driver_states(index)
            if fresh(item, 0.5):
                break
            if time.monotonic() >= deadline:
                raise RuntimeError(f"关节 {index} 驱动反馈缺失/过期")
            time.sleep(0.05)
        foc = item.msg.foc_status
        drivers.append({"joint": index, "enabled": bool(foc.driver_enable_status),
                        "undervoltage": bool(foc.voltage_too_low),
                        "driver_error": bool(foc.driver_error_status),
                        "voltage_v": float(item.msg.vol)})
    variation = 0.0
    if require_stable:
        time.sleep(0.25)
        newer = robot.get_joint_angles()
        status = robot.get_arm_status()
        flange = robot.get_flange_pose()
        if not fresh(status, 0.25) or not fresh(flange, 0.25):
            raise RuntimeError("停稳复核时控制器或法兰反馈过期")
        if not fresh(newer, 0.25):
            raise RuntimeError("停稳复核时关节反馈过期")
        second = [float(value) for value in newer.msg]
        variation = max(abs(a-b) for a,b in zip(first, second))
        first = second
        pose = [float(value) for value in flange.msg]
        for index in range(1, 8):
            item = robot.get_driver_states(index)
            if not fresh(item, 0.5):
                raise RuntimeError("停稳复核时驱动反馈过期")
            foc = item.msg.foc_status
            drivers[index-1].update(
                enabled=bool(foc.driver_enable_status),
                undervoltage=bool(foc.voltage_too_low),
                driver_error=bool(foc.driver_error_status),
                voltage_v=float(item.msg.vol))
    reasons = []
    if int(status.msg.ctrl_mode) != 1:
        reasons.append("控制器未处于 CAN 控制模式")
    if int(status.msg.arm_status) != 0 or status.msg.err_code:
        reasons.append("机械臂状态异常或有错误码")
    if int(status.msg.teach_status) not in (0,2,6):
        reasons.append("示教尚未退出")
    if any(not d["enabled"] or d["undervoltage"] or d["driver_error"] for d in drivers):
        reasons.append("七轴中存在失能、欠压或驱动故障")
    if require_stable and variation > 0.002:
        reasons.append("七轴反馈尚未停稳")
    if math.dist(robot.fk(first)[:3], pose[:3]) > 0.02:
        reasons.append("正运动学与法兰反馈不一致")
    return {"captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "control_mode": int(status.msg.ctrl_mode),
            "arm_status": int(status.msg.arm_status),
            "teach_status": int(status.msg.teach_status),
            "motion_status": int(status.msg.motion_status),
            "error_code": int(status.msg.err_code),
            "joint_rad": first, "flange_pose_m_rad": pose,
            "joint_variation_0_25s_rad": variation,
            "drivers": drivers, "ready_for_motion": not reasons,
            "reasons": reasons}


def require_ready(snapshot):
    if not snapshot["ready_for_motion"]:
        raise RuntimeError("实时预检未通过：" + "；".join(snapshot["reasons"]))
    return snapshot
