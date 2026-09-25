#!/usr/bin/env python3
"""Nero 单臂按时钟连续发布关节目标；默认只生成或复核离线计划。"""

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

from experiments.single_arm.can_setup.can_drag_teach import MOUNT_CODES, fresh, send_frame, wait_status
from experiments.single_arm.continuous_replay.trajectory import (
    DEFAULT_SOURCE, EXPECTED_SOURCE_SHA256, make_plan, read_source,
)
from experiments.single_arm.replay.can_trace import ReplayCanTrace
from experiments.single_arm.start_transfer.go_to_start import stable_joint_feedback
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, validate_joints,
)


PLAN_DIR = Path("experiments/single_arm/continuous_replay/data/plans")
RUN_DIR = Path("experiments/single_arm/continuous_replay/data/runs")
CAN_IDS = (0x150, 0x151, 0x155, 0x156, 0x157, 0x170, 0x2A1,
           0x2A5, 0x2A6, 0x2A7, 0x2A9, 0x471)
START_TOLERANCE_RAD = 0.005
FINAL_TOLERANCE_RAD = 0.005
FEEDBACK_MAX_AGE_S = 0.20
DRIVER_MAX_AGE_S = 0.5
MAX_LATE_S = 0.10
TRACKING_WARN_RAD = 0.06
TRACKING_HARD_RAD = 0.10
TRACKING_WARN_DURATION_S = 0.30
FLANGE_WARN_M = 0.03
FLANGE_HARD_M = 0.05
FLANGE_SOURCE_MARGIN_M = 0.02


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def robot_instance(channel):
    return AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121,
        channel=channel, local_loopback=True,
    ))


def source_envelope(source):
    return [[min(point[axis] for point in source["flange_xyz"]),
             max(point[axis] for point in source["flange_xyz"])]
            for axis in range(3)]


def geometry_audit(robot, plan, source, config):
    """离线验证全目标、记录 FK 一致性和源法兰包围盒外扩范围。"""
    envelope = source_envelope(source)
    max_recorded_fk_error = 0.0
    for joints, measured in zip(source["joints"], source["flange_xyz"]):
        predicted = robot.fk(joints)
        max_recorded_fk_error = max(max_recorded_fk_error, math.dist(predicted[:3], measured))
    if max_recorded_fk_error > 0.02:
        raise ValueError("示教源 CAN 法兰反馈与 SDK 正运动学不一致")
    limits = [[float("inf"), float("-inf")] for _ in range(3)]
    for item in plan["waypoints"]:
        joints = validate_joints(item["joint_rad"], config["zero_exclusion_radius_rad"])
        pose = robot.fk(joints)
        for axis in range(3):
            value = float(pose[axis])
            if not math.isfinite(value) or not (envelope[axis][0] - FLANGE_SOURCE_MARGIN_M
                                                 <= value <= envelope[axis][1] + FLANGE_SOURCE_MARGIN_M):
                raise ValueError(f"计划法兰第 {axis + 1} 轴超出示教源包围盒外扩 0.02 m")
            limits[axis][0] = min(limits[axis][0], value)
            limits[axis][1] = max(limits[axis][1], value)
    return {
        "source_flange_xyz_range_m": dict(zip("xyz", envelope)),
        "planned_flange_xyz_range_m": dict(zip("xyz", limits)),
        "max_source_fk_feedback_error_m": max_recorded_fk_error,
        "flange_margin_m": FLANGE_SOURCE_MARGIN_M,
    }


def create_plan(source_path, config, until_source_s=None, frequency_hz=20,
                turnaround_hold_s=0.0):
    source = read_source(source_path, config)
    if Path(source_path).resolve() == DEFAULT_SOURCE.resolve() and source["sha256"] != EXPECTED_SOURCE_SHA256:
        raise ValueError("固定 20 秒源记录校验和发生变化，拒绝使用")
    plan = make_plan(source, config, until_source_s=until_source_s,
                     frequency_hz=frequency_hz,
                     turnaround_hold_s=turnaround_hold_s)
    plan["mount"] = config["mount"]
    plan["safe_start_joint_rad"] = config["safe_start_joint_rad"]
    plan["created_at_utc"] = utc_now()
    plan["geometry_audit"] = geometry_audit(robot_instance(config["channel"]), plan, source, config)
    return plan


def check_plan(path, config):
    """重建每一周期目标，拒绝文件篡改、配置变化或源记录替换。"""
    plan = json.loads(Path(path).read_text(encoding="utf-8"))
    if plan.get("schema_version") != 1 or plan.get("mount") != config["mount"]:
        raise ValueError("连续轨迹计划版本或安装方向不匹配")
    if plan.get("safe_start_joint_rad") != config["safe_start_joint_rad"]:
        raise ValueError("连续轨迹计划与当前 S1 配置不匹配")
    expected = create_plan(
        Path(plan["recording"]), config,
        until_source_s=plan["until_source_s"], frequency_hz=plan["frequency_hz"],
        turnaround_hold_s=plan.get("turnaround_hold_s", 0.0),
    )
    if plan.get("turnaround_hold_s", 0.0) != expected["turnaround_hold_s"]:
        raise ValueError("转向停留时间与重新计算结果不符")
    for key in (
        "source_sha256", "source_total_rows", "source_teaching_rows",
        "source_used_rows", "time_scale", "total_duration_s", "return_to_start",
        "max_velocity_rad_s", "max_acceleration_rad_s2", "min_time_scale",
        "geometry_audit",
    ):
        if plan.get(key) != expected[key]:
            raise ValueError(f"连续轨迹计划字段 {key} 与重新计算结果不符")
    waypoints = plan.get("waypoints")
    if not isinstance(waypoints, list) or len(waypoints) != len(expected["waypoints"]):
        raise ValueError("连续轨迹目标数量不符")
    for index, (actual, recomputed) in enumerate(zip(waypoints, expected["waypoints"])):
        if (actual.get("phase") != recomputed["phase"]
                or abs(float(actual["t_s"]) - recomputed["t_s"]) > 1e-9
                or max(abs(float(a)-b) for a,b in zip(actual["joint_rad"], recomputed["joint_rad"])) > 1e-9):
            raise ValueError(f"连续轨迹第 {index} 周期目标与重新计算结果不符")
    return plan


def read_realtime(robot, with_timestamps=False):
    status = robot.get_arm_status()
    if not fresh(status, FEEDBACK_MAX_AGE_S):
        raise RuntimeError("整机状态反馈超过 200 ms")
    if (int(status.msg.ctrl_mode) != 1 or int(status.msg.teach_status) not in CAN_IDLE_TEACH_STATUSES
            or int(status.msg.arm_status) != 0 or status.msg.err_code):
        raise RuntimeError("整机状态或故障码不允许连续运动")
    joint = robot.get_joint_angles()
    flange = robot.get_flange_pose()
    if not fresh(joint, FEEDBACK_MAX_AGE_S) or not fresh(flange, FEEDBACK_MAX_AGE_S):
        raise RuntimeError("关节或法兰反馈超过 200 ms")
    # SDK 把 J1..J7 与 XYZ 姿态分别从数帧拼起来，聚合消息的 timestamp
    # 只采用最后一帧。逐组检查可避免某一对关节停发而其它组仍在更新。
    parser = robot._parser
    for part in ("joint_12", "joint_34", "joint_56", "joint_7",
                 "end_pose_xy", "end_pose_zrx", "end_pose_ryrz"):
        if not fresh(getattr(parser, part, None), FEEDBACK_MAX_AGE_S):
            raise RuntimeError(f"CAN 反馈组成帧 {part} 超过 200 ms")
    result = (status, [float(value) for value in joint.msg],
              [float(value) for value in flange.msg])
    if with_timestamps:
        return (*result, float(joint.timestamp), float(flange.timestamp))
    return result


def driver_snapshot(robot):
    drivers = []
    for index in range(1, 8):
        item = robot.get_driver_states(index)
        if not fresh(item, DRIVER_MAX_AGE_S):
            raise RuntimeError(f"关节 {index} 驱动反馈超过 500 ms")
        foc = item.msg.foc_status
        if not foc.driver_enable_status or foc.voltage_too_low or foc.driver_error_status:
            raise RuntimeError(f"关节 {index} 失能、欠压或驱动故障")
        drivers.append({"joint": index, "voltage_v": float(item.msg.vol),
                        "enabled": bool(foc.driver_enable_status)})
    return {"time_utc": utc_now(), "drivers": drivers}


def verify_start(robot, plan, config):
    if wait_status(robot, lambda item: fresh(item), timeout=3) is None:
        raise RuntimeError("CAN 整机状态反馈缺失")
    healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
    check_drivers(robot)
    current = stable_joint_feedback(robot)
    if max(abs(a-b) for a,b in zip(current, plan["waypoints"][0]["joint_rad"])) > START_TOLERANCE_RAD:
        raise RuntimeError("实时起点偏离示教源首点超过 0.005 rad")
    validate_joints(current, config["zero_exclusion_radius_rad"])
    return current


def configure_joint_mode(robot, config, speed_percent=10):
    """沿用已验证的左侧装 CAN 关节模式，只在循环开始前设置一次。"""
    with can.Bus(channel=config["channel"], interface="socketcan", local_loopback=True) as bus:
        baseline = robot.get_arm_status()
        if not fresh(baseline):
            raise RuntimeError("切换模式前缺少新鲜状态")
        stamp = float(baseline.timestamp)
        send_frame(bus, 0x151, [1, 1, speed_percent, 0, 0, MOUNT_CODES[config["mount"]], 0, 0])
        status = wait_status(robot, lambda item: fresh(item)
                             and float(item.timestamp) > stamp
                             and int(item.msg.ctrl_mode) == 1, timeout=3)
        if status is None:
            raise RuntimeError("CAN 关节模式未得到新反馈确认")
    robot.set_speed_percent(speed_percent)
    robot.set_motion_mode("j")
    mode = wait_status(robot, lambda item: fresh(item) and int(item.msg.ctrl_mode) == 1
                       and int(item.msg.mode_feedback) == 1 and int(item.msg.arm_status) == 0,
                       timeout=3)
    if mode is None:
        raise RuntimeError("控制器未确认 move_j 模式")
    robot.set_auto_set_motion_mode_enabled(False)
    check_drivers(robot)


def write_json_exclusive(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def print_plan(plan, path):
    print("连续目标计划：", path)
    print("源记录 SHA-256：", plan["source_sha256"])
    print(f"使用 {plan['source_used_rows']}/{plan['source_total_rows']} 条源反馈；"
          f"源时长 {plan['source_used_duration_s']:.3f} s；往返 {plan['total_duration_s']:.1f} s")
    print(f"发送频率 {plan['frequency_hz']} Hz；时间伸缩 {plan['time_scale']:.1f} 倍；"
          f"总目标 {len(plan['waypoints'])} 条")
    print(f"最大源轨迹偏差 {plan['max_path_deviation_rad']:.6f} rad；"
          f"目标速度上限 {plan['max_velocity_rad_s']:.2f} rad/s；"
          f"加速度上限 {plan['max_acceleration_rad_s2']:.2f} rad/s")
    if plan.get("turnaround_hold_s", 0):
        print(f"源前缀终点停留 {plan['turnaround_hold_s']:.1f} 秒，须连续反馈确认到位后才反向。")
    print("法兰预测范围 (m)：", plan["geometry_audit"]["planned_flange_xyz_range_m"])


def require_prior_stage(plan):
    """完整实验须由同源数据的前一规模实机成功结果逐级解锁。"""
    used = plan["source_used_duration_s"]
    if used <= 1.81:
        return
    if used <= 2.21:
        lower, upper = 1.7, 1.81
    elif used <= 5.01:
        lower, upper = 2.1, 2.21
    else:
        lower, upper = 4.9, 5.01
    for path in RUN_DIR.glob("nero_continuous_*.json"):
        if path.stem.endswith("_analysis"):
            continue
        try:
            report = json.loads(path.read_text(encoding="utf-8"))
            analysis = json.loads(path.with_name(path.stem + "_analysis.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if (report.get("source_sha256") == plan["source_sha256"]
                and lower <= report.get("source_used_duration_s", -1) <= upper
                and report.get("completed") is True
                and analysis.get("acceptance_pass") is True):
            return
    raise RuntimeError(
        f"尚无同源且已独立分析通过的 {lower:.1f}～{upper:.1f} 秒前缀实机结果；"
        "必须按 1.8→2.2→5.0→完整记录逐级验证"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--source-until-s", type=float, help="只规划源记录开头的小段；默认完整记录")
    parser.add_argument("--turnaround-hold-s", type=float, default=0.0,
                        help="仅源前缀可用：终点保持秒数，再沿原轨迹回 S1")
    parser.add_argument("--frequency-hz", type=int, choices=(10,20), default=20)
    parser.add_argument("--plan", type=Path, help="复核并执行现有计划")
    parser.add_argument("--run", action="store_true", help="显式执行实机连续运动")
    args = parser.parse_args()
    if args.run and not args.plan:
        parser.error("--run 必须指定已复核的 --plan")
    if args.plan and args.turnaround_hold_s:
        parser.error("复核已保存计划时，停留时长由计划文件决定")
    try:
        config = load_config(args.config)
        if args.plan:
            plan = check_plan(args.plan, config)
            path = args.plan
        else:
            plan = create_plan(args.recording, config, args.source_until_s,
                               args.frequency_hz, args.turnaround_hold_s)
            path = PLAN_DIR / ("nero_continuous_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
            write_json_exclusive(path, plan)
        print_plan(plan, path)
        if not args.run:
            print("离线规划/复核完成；未连接机械臂或发送运动指令。")
            return 0
        require_prior_stage(plan)
        from experiments.single_arm.continuous_replay.run_stream import run
        run(plan, path, config)
    except (OSError, ValueError, RuntimeError, can.CanError) as exc:
        print(f"连续回放未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
