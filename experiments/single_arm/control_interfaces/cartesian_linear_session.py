#!/usr/bin/env python3
"""单次发布法兰目标给 Nero 控制器：本机预检，控制器执行 IK 与直线运动。"""

import argparse
import csv
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
from experiments.single_arm.control_interfaces.ik_preview import pose_error, solve_nearby_pose
from experiments.single_arm.control_interfaces.publishers import (
    CartesianLinearPublisher, CartesianPointPublisher,
)
from experiments.single_arm.start_transfer.go_to_start import stable_joint_feedback
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, point_segment_distance, validate_joints,
)


REPORT_DIR = Path("experiments/single_arm/control_interfaces/data/runs")
MAX_LINEAR_DISTANCE_M = 0.03
MAX_JOINT_CHANGE_RAD = 0.18
# move_l 的关节逆解由控制器决定，本机近邻 IK 的关节折线仅作诊断。
# 实时门禁检查控制器承诺的法兰直线，并限制实际关节相对起点的变化。
MAX_FLANGE_LINE_ERROR_M = 0.005
MAX_ACTUAL_JOINT_CHANGE_RAD = 0.25
MAX_ACTUAL_ROTATION_ERROR_RAD = 0.03
MAX_POINT_TARGET_DISTANCE_M = 0.005
MAX_POINT_EXCURSION_M = 0.03
TARGET_POSITION_TOLERANCE_M = 0.0005
TARGET_ROTATION_TOLERANCE_RAD = 0.02
RETURN_JOINT_TOLERANCE_RAD = 0.03


class StationaryIncomplete(RuntimeError):
    """控制器报告运动结束且反馈稳定，无需追加阻尼急停。"""


def checked_pose(message):
    """读取新鲜的法兰六维反馈；SDK 坐标单位为 m/rad。"""
    if not fresh(message):
        raise RuntimeError("法兰位姿反馈缺失或过期")
    pose = [float(value) for value in message.msg]
    if len(pose) != 6 or not all(math.isfinite(value) for value in pose):
        raise RuntimeError("法兰位姿反馈无效")
    return pose


def read_anchor_recording(path, robot, config):
    """从连续只读 CSV 的末端提取临时起点，不修改历史 S1 配置。"""
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 5:
        raise ValueError("临时起点至少需要 5 条连续采样")
    tail = rows[-5:]
    joints = [
        validate_joints(
            [float(row[f"joint_{i}_rad"]) for i in range(1, 8)],
            config["zero_exclusion_radius_rad"],
        ) for row in tail
    ]
    anchor = joints[-1]
    if any(max(abs(a-b) for a,b in zip(point,anchor)) > 0.002 for point in joints):
        raise ValueError("临时起点的末 5 条关节反馈未停稳")
    names = ("x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad")
    pose = [float(tail[-1][name]) for name in names]
    if not all(math.isfinite(value) for value in pose):
        raise ValueError("临时起点法兰位姿无效")
    for row in tail[:-1]:
        sample = [float(row[name]) for name in names]
        deviation = pose_error(pose, sample)
        if (math.dist(deviation[:3], [0.0] * 3) > 0.002
                or max(abs(value) for value in deviation[3:]) > 0.02):
            raise ValueError("临时起点的末 5 条法兰反馈未停稳")
    if math.dist(robot.fk(anchor)[:3], pose[:3]) > 0.005:
        raise ValueError("临时起点的关节 FK 与法兰记录不一致")
    return anchor, pose


def preview_line(robot, start_joints, start_pose, target_pose, config, anchor_joints=None):
    """沿预期笛卡尔直线每 2 mm 求近邻解，只用于事前筛查。

    真正执行时仅发一次 move_l；控制器内部逆解可能不同，所以实机仍监控关节。
    """
    travel = math.dist(start_pose[:3], target_pose[:3])
    rotation = max(abs(value) for value in pose_error(target_pose, start_pose)[3:])
    if travel < 0.001 or travel > MAX_LINEAR_DISTANCE_M or rotation > 0.01:
        raise ValueError("单次 move_l 仅允许平移 1～30 mm，姿态变化不超过 0.01 rad")
    if config.get("flange_workspace_m") is None:
        # 尚未测得现场碰撞边界，只允许已记录且停稳的锚点附近试验。
        anchor = anchor_joints if anchor_joints is not None else config["safe_start_joint_rad"]
        if max(abs(a - b) for a, b in zip(start_joints, anchor)) > 0.18:
            raise ValueError("未配置法兰工作空间，仅允许已记录起点附近试验")
    steps = max(1, math.ceil(travel / 0.002))
    route = [list(start_joints)]
    for index in range(1, steps + 1):
        fraction = index / steps
        pose = [a + fraction * (b - a) for a, b in zip(start_pose, target_pose)]
        for axis, value in zip(("x", "y", "z"), pose[:3]):
            bounds = config.get("flange_workspace_m")
            if bounds is not None and not bounds[axis][0] <= value <= bounds[axis][1]:
                raise ValueError(f"法兰 {axis} 超出配置的工作空间")
        next_joints = solve_nearby_pose(
            robot, route[-1], pose, config["zero_exclusion_radius_rad"]
        )
        if max(abs(a - b) for a, b in zip(next_joints, start_joints)) > MAX_JOINT_CHANGE_RAD:
            raise ValueError("预测关节变化超过 0.18 rad；拒绝单次直线运动")
        route.append(next_joints)
    return route


def preview_point(robot, start_joints, start_pose, target_pose, config, anchor_joints=None):
    """只为小范围点位试验筛查终点；控制器的实际 P 路线仍未知。"""
    travel = math.dist(start_pose[:3], target_pose[:3])
    if travel > MAX_POINT_TARGET_DISTANCE_M:
        raise ValueError("首次 move_p 点位试验仅允许平移 1～5 mm")
    # 复用已有的锚点、关节限位和近邻 IK 预检；直线采样结果只作候选，
    # 不能据此假定 P 模式实际会沿这条线运动。
    return preview_line(robot, start_joints, start_pose, target_pose, config,
                        anchor_joints)


def route_error(joints, route):
    """实际反馈与离线预测关节折线的最小距离。"""
    return min(point_segment_distance(joints, a, b) for a, b in zip(route, route[1:]))


def check_motion_drivers(robot):
    """运动中读取可用的驱动故障位，不等待较慢的驱动状态帧。

    七轴完整反馈在发送目标前由 check_drivers() 检查。运动过程中
    CAN 状态帧可能晚于高频位置帧；等待它们会阻塞运动进展检查。
    过期的关节编号写入运行报告，由实时整机状态和位置反馈继续监控。
    """
    stale = []
    for index in range(1, 8):
        driver = robot.get_driver_states(index)
        if not fresh(driver):
            stale.append(index)
            continue
        foc = driver.msg.foc_status
        if not foc.driver_enable_status or foc.voltage_too_low or foc.driver_error_status:
            raise RuntimeError(f"关节 {index} 未使能、欠压或有驱动故障")
    return stale


def run_one_line(robot, start_pose, target_pose, route, speed_percent, report, aborted,
                 expected_joint_rad=None, zero_radius_rad=0.25, motion_mode="l",
                 timeout_s=30.0, max_flange_excursion_m=None,
                 max_rotation_change_rad=MAX_ACTUAL_ROTATION_ERROR_RAD,
                 max_joint_change_rad=MAX_ACTUAL_JOINT_CHANGE_RAD):
    """仅发布一个法兰目标；L 检查直线，P 只检查点位活动范围。"""
    if motion_mode not in ("l", "p"):
        raise ValueError("法兰运动模式必须是 l 或 p")
    if aborted():
        raise InterruptedError("发送前取消；未发送运动命令")
    robot.set_speed_percent(speed_percent)
    if aborted():
        raise InterruptedError("发送前取消；未发送运动命令")
    publisher = CartesianLinearPublisher() if motion_mode == "l" else CartesianPointPublisher()
    publisher.publish(robot, target_pose)
    report["command_count"] = 1
    report["command_sent_at_utc"] = datetime.now(timezone.utc).isoformat()
    command_start = time.monotonic()
    deadline = command_start + timeout_s
    if max_flange_excursion_m is None:
        max_flange_excursion_m = MAX_POINT_EXCURSION_M
    first_joint = list(route[0])
    last_change = time.monotonic()
    report["feedback_samples"] = []
    try:
        while time.monotonic() < deadline:
            if aborted():
                raise InterruptedError("人工中断")
            status = healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            stale_drivers = check_motion_drivers(robot)
            joints = joint_point(robot)
            pose = checked_pose(robot.get_flange_pose())
            validate_joints(joints, zero_radius_rad)
            report["last_joint_rad"] = joints
            report["last_flange_pose"] = pose
            report["last_motion_status"] = str(status.msg.motion_status)
            # SDK 只在 move_l/p/c 后提供此反馈；它是控制器选出的解，
            # 不能替代发送命令前的只读 IK 预检。
            ik_feedback = robot.get_ik_joint_angles()
            if fresh(ik_feedback):
                report["controller_ik_joint_rad"] = [float(v) for v in ik_feedback.msg]
            path_error = route_error(joints, route)
            flange_line_error = point_segment_distance(
                pose[:3], start_pose[:3], target_pose[:3]
            )
            flange_excursion = math.dist(pose[:3], start_pose[:3])
            rotation_error = max(abs(value) for value in pose_error(pose, start_pose)[3:])
            joint_change = max(abs(a-b) for a, b in zip(joints, route[0]))
            report["feedback_samples"].append({
                "elapsed_s": round(time.monotonic() - command_start, 4),
                "motion_status": str(status.msg.motion_status),
                "joint_rad": list(joints),
                "flange_pose": list(pose),
                "predicted_joint_path_error_rad": path_error,
                "flange_line_error_m": flange_line_error,
                "flange_excursion_m": flange_excursion,
                "rotation_change_rad": rotation_error,
                "max_joint_change_from_start_rad": joint_change,
                "controller_ik_joint_rad": report.get("controller_ik_joint_rad"),
                "stale_driver_feedback": stale_drivers,
            })
            if max(abs(a - b) for a, b in zip(joints, first_joint)) > 0.001:
                last_change = time.monotonic()
                first_joint = joints
            error = pose_error(target_pose, pose)
            if (math.dist(error[:3], [0.0] * 3) <= TARGET_POSITION_TOLERANCE_M
                    and max(abs(v) for v in error[3:]) <= TARGET_ROTATION_TOLERANCE_RAD
                    and int(status.msg.motion_status) == 0):
                report["position_error_m"] = math.dist(error[:3], [0.0] * 3)
                report["flange_reached"] = True
                if expected_joint_rad is not None:
                    joint_error = max(abs(a-b) for a,b in zip(joints, expected_joint_rad))
                    report["return_joint_error_rad"] = joint_error
                    if joint_error > RETURN_JOINT_TOLERANCE_RAD:
                        raise StationaryIncomplete(
                            f"法兰已到目标，但关节未回原姿态；最大误差 {joint_error:.4f} rad"
                        )
                report["completed"] = True
                return
            if ((motion_mode == "l" and flange_line_error > MAX_FLANGE_LINE_ERROR_M)
                    or (motion_mode == "p" and flange_excursion > max_flange_excursion_m)
                    or rotation_error > max_rotation_change_rad
                    or joint_change > max_joint_change_rad):
                if int(status.msg.motion_status) == 0:
                    raise StationaryIncomplete("控制器报告已停止，实际姿态超出法兰路线或关节范围")
                raise RuntimeError("实测法兰或关节超出本次运动范围")
            if time.monotonic() - last_change > 5.0:
                if int(status.msg.motion_status) == 0:
                    # 控制器明确称目标已结束、反馈也稳定；急停反而会让关节失去保持。
                    raise StationaryIncomplete("控制器报告运动结束但未到法兰目标")
                raise RuntimeError("控制器仍报告运动中，但反馈停滞")
            time.sleep(0.1)
        raise TimeoutError(f"单次 {motion_mode} 模式运动超过 {timeout_s:.0f} 秒")
    except (RuntimeError, TimeoutError, InterruptedError, ValueError) as exc:
        # 已被控制器报告为结束且关节稳定时，不再引入可能下垂的阻尼急停。
        if not isinstance(exc, StationaryIncomplete):
            robot.electronic_emergency_stop()
            report["electronic_emergency_stop_sent"] = True
            raise type(exc)(f"{exc}；已发送阻尼电子急停，请支撑并检查状态") from exc
        raise


def main(motion_mode="l"):
    if motion_mode not in ("l", "p"):
        raise ValueError("法兰运动模式必须是 l 或 p")
    command = "move_l" if motion_mode == "l" else "move_p"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--dx-m", type=float, help="从当前法兰沿基座 X 轴平移，首次建议 -0.002")
    destination.add_argument("--to-s1", action="store_true", help="从 S1 附近单次直线回到 S1 法兰位姿")
    destination.add_argument("--to-anchor", action="store_true", help="回到临时起点 CSV 的末次法兰位姿")
    parser.add_argument("--anchor-recording", type=Path,
                        help="临时起点的连续采样 CSV；不会修改 S1 配置")
    parser.add_argument("--speed-percent", type=int, choices=(5, 10), default=5)
    parser.add_argument("--run", action="store_true", help="显式执行；默认只读预检")
    args = parser.parse_args()
    if args.to_anchor and args.anchor_recording is None:
        parser.error("--to-anchor 必须指定 --anchor-recording")
    if args.to_s1 and args.anchor_recording is not None:
        parser.error("--to-s1 不与 --anchor-recording 同时使用")
    robot = None
    try:
        config = load_config(args.config)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        if wait_status(robot, fresh, timeout=3.0) is None:
            raise RuntimeError("控制器状态反馈缺失")
        healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
        check_drivers(robot)
        start = validate_joints(stable_joint_feedback(robot), config["zero_exclusion_radius_rad"])
        start_pose = checked_pose(robot.get_flange_pose())
        anchor_joints = None
        anchor_pose = None
        if args.anchor_recording is not None:
            anchor_joints, anchor_pose = read_anchor_recording(
                args.anchor_recording, robot, config
            )
        if args.to_s1:
            target_pose = [float(value) for value in robot.fk(config["safe_start_joint_rad"])]
            expected_joint_rad = config["safe_start_joint_rad"]
        elif args.to_anchor:
            target_pose = anchor_pose
            expected_joint_rad = anchor_joints
        else:
            expected_joint_rad = None
            if not math.isfinite(args.dx_m):
                raise ValueError("X 位移必须是有限数值")
            if anchor_joints is not None and (
                max(abs(a-b) for a,b in zip(start,anchor_joints)) > 0.005
                or math.dist(start_pose[:3], anchor_pose[:3]) > 0.005
            ):
                raise ValueError("当前位置偏离临时起点，不能开始外行程")
            target_pose = list(start_pose)
            target_pose[0] += args.dx_m
        preview = preview_line if motion_mode == "l" else preview_point
        route = preview(robot, start, start_pose, target_pose, config, anchor_joints)
        print("当前法兰位姿 (m/rad)：", start_pose)
        print("单次目标位姿 (m/rad)：", target_pose)
        print(f"预计平移 {math.dist(start_pose[:3], target_pose[:3]):.4f} m；"
              f"离线预检 {len(route)-1} 个采样点；实际只发送一次 {command}；速度 {args.speed_percent}%。")
        if motion_mode == "p":
            print("P 模式只指定法兰终点；控制器实际中间路线未知，预检采样不是路径预测。")
        print("预测最大单关节变化 (rad)：",
              max(max(abs(a-b) for a,b in zip(point,start)) for point in route))
        if args.anchor_recording is not None:
            print("临时起点记录：", args.anchor_recording)
        if not args.run:
            print("只读预检完成；未发送运动指令。")
            return 0
        stopped = False

        def on_signal(_signum, _frame):
            nonlocal stopped
            stopped = True

        old_int = signal.signal(signal.SIGINT, on_signal)
        old_term = signal.signal(signal.SIGTERM, on_signal)
        report = {
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "start_joint_rad": start, "start_flange_pose": start_pose,
            "target_flange_pose": target_pose, "speed_percent": args.speed_percent,
            "control_command": command, "command_count": 0, "completed": False,
            "anchor_recording": str(args.anchor_recording) if args.anchor_recording else None,
        }
        try:
            for remaining in range(5, 0, -1):
                if stopped:
                    raise InterruptedError("倒计时取消；未发送运动命令")
                print(f"{remaining} 秒后发送一次 {command}；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            now = stable_joint_feedback(robot)
            if max(abs(a-b) for a,b in zip(now,start)) > 0.005:
                raise RuntimeError("倒计时期间起点变化；未发送运动命令")
            if stopped:
                raise InterruptedError("倒计时期间取消；未发送运动命令")
            run_one_line(robot, start_pose, target_pose, route, args.speed_percent, report,
                         lambda: stopped, expected_joint_rad=expected_joint_rad,
                         zero_radius_rad=config["zero_exclusion_radius_rad"],
                         motion_mode=motion_mode)
            print(f"单次 {command} 到位；法兰位置误差", report["position_error_m"], "m")
        except Exception as exc:
            report["error"] = str(exc)
            raise
        finally:
            signal.signal(signal.SIGINT, old_int)
            signal.signal(signal.SIGTERM, old_term)
            report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
            REPORT_DIR.mkdir(parents=True, exist_ok=True)
            path = REPORT_DIR / (f"nero_{command}_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json")
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print("实验记录：", path, flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError, can.CanError) as exc:
        print(f"单次 {command} 未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if robot is not None:
            robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
