#!/usr/bin/env python3
"""历史示教记录的只读倒序路线检查；新的位置回位用 lab.cli return。"""

import argparse
import csv
import json
from pathlib import Path
import sys

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.pose_recording.record_poses import JOINT_COLUMNS
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, plan_reverse_return, validate_joints,
    validate_route_workspace,
)


def load_recording(path, config, allow_completed=False):
    """读取 CSV 与同名元数据，验证本次记录完整、起点匹配且尚可回位。"""
    metadata_path = path.with_suffix(".json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    # 已成功回位的记录不能再次当作“从末端恢复”执行；回放另有专门脚本。
    if metadata.get("return_completed") and not allow_completed:
        raise ValueError("记录已标记为回位完成，拒绝重复执行")
    if metadata.get("stop_reason") == "error" or not metadata.get("post_stop_samples", 0):
        raise ValueError("记录在示教错误后中断或缺少退出拖动后的停稳轨迹，禁止倒序回位")
    if metadata.get("mount") != config["mount"]:
        raise ValueError("记录与当前安装方向配置不一致")
    if metadata.get("safe_start_name") != config["safe_start_name"]:
        raise ValueError("记录与当前安全起点配置不一致")
    original = validate_joints(
        metadata["original_joint_rad"], config["zero_exclusion_radius_rad"]
    )
    if max(abs(a - b) for a, b in zip(original, config["safe_start_joint_rad"])) > config[
        "start_max_joint_error_rad"
    ]:
        raise ValueError("记录的示教前位置偏离当前配置起点")

    # 同时检查文件行数、顺序与反馈时间；单凭 JSON 的完成标志不足以证明轨迹完整。
    recorded = []
    last_elapsed = float("-inf")
    last_feedback = float("-inf")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or not set(JOINT_COLUMNS).issubset(reader.fieldnames):
            raise ValueError("记录缺少七个关节角列")
        for index, row in enumerate(reader):
            if int(row["sample_index"]) != index:
                raise ValueError("示教记录序号不连续")
            elapsed = float(row["elapsed_s"])
            feedback = float(row["joint_feedback_unix_s"])
            if elapsed < last_elapsed or feedback < last_feedback:
                raise ValueError("示教时间或关节反馈时间倒退")
            last_elapsed, last_feedback = elapsed, feedback
            point = validate_joints(
                [float(row[column]) for column in JOINT_COLUMNS],
                config["zero_exclusion_radius_rad"],
            )
            recorded.append(point)
    if len(recorded) < 2 or len(recorded) != metadata.get("recorded_samples"):
        raise ValueError("记录条数不足或与元数据不一致")
    return metadata_path, metadata, original, recorded


def main():
    """仅检查历史记录与当前端点；不再提供倒序运动入口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="本次未完成回位的示教 CSV")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true", help="已停用；会拒绝并提示新入口")
    args = parser.parse_args()

    if args.run:
        print("倒序运动入口已停用。请从实时姿态运行 lab.cli return plan/run。",
              file=sys.stderr, flush=True)
        return 1

    try:
        config = load_config(args.config)
        metadata_path, metadata, original, recorded = load_recording(args.recording, config)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, lambda item: fresh(item), timeout=3) is None:
                raise RuntimeError("机械臂状态反馈缺失")
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            current = validate_joints(joint_point(robot), config["zero_exclusion_radius_rad"])
            route = plan_reverse_return(recorded, current, original, config)
            validate_route_workspace(robot, route, config)
            print(f"起点 {config['safe_start_name']}；当前与记录末端吻合；倒序路径 {len(route)-1} 个小步。")
            print("只读检查完成。倒序运动入口已停用；回位请从实时姿态重新规划。")
            return 0
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError, can.CanError) as exc:
        print(f"回位未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
