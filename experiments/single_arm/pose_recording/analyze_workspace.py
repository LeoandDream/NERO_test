#!/usr/bin/env python3
"""离线统计示教记录中的法兰位置范围，并用 SDK 正运动学交叉核对。"""

import argparse
import csv
import json
import math
from pathlib import Path

from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.pose_recording.record_poses import JOINT_COLUMNS, POSE_COLUMNS


def bounds(points):
    """计算一组法兰位置在 X/Y/Z 三轴的最小值和最大值。"""
    return {
        axis: [min(point[index] for point in points), max(point[index] for point in points)]
        for index, axis in enumerate(("x", "y", "z"))
    }


def analyze(paths):
    """按录制文件汇总法兰坐标，并用 SDK 正运动学交叉检查关节数据。"""
    robot = AgxArmFactory.create_arm(create_agx_arm_config(
        robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel="can0"
    ))
    results = []
    for path in paths:
        fk_positions = []
        reported_positions = []
        joint_values = [[] for _ in range(7)]
        synchronized_errors = []
        with path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            required = set(JOINT_COLUMNS + POSE_COLUMNS[:3])
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                raise ValueError(f"记录缺少关节角或法兰位置列：{path}")
            for row in reader:
                joints = [float(row[column]) for column in JOINT_COLUMNS]
                reported = [float(row[column]) for column in POSE_COLUMNS[:3]]
                if not all(math.isfinite(value) for value in joints + reported):
                    raise ValueError(f"记录包含无效数值：{path}")
                fk_position = [float(value) for value in robot.fk(joints)[:3]]
                fk_positions.append(fk_position)
                reported_positions.append(reported)
                for index, value in enumerate(joints):
                    joint_values[index].append(value)
                if "joint_feedback_unix_s" in row and "pose_feedback_unix_s" in row:
                    age_gap = abs(float(row["joint_feedback_unix_s"]) - float(row["pose_feedback_unix_s"]))
                    if age_gap <= 0.005:
                        synchronized_errors.append(math.dist(fk_position, reported))
        if not fk_positions:
            raise ValueError(f"空记录：{path}")
        results.append({
            "file": str(path),
            "samples": len(fk_positions),
            "observed_fk_flange_bounds_m": bounds(fk_positions),
            "observed_reported_flange_bounds_m": bounds(reported_positions),
            "observed_joint_bounds_rad": {
                f"joint{index + 1}": [min(values), max(values)]
                for index, values in enumerate(joint_values)
            },
            "near_synchronous_fk_feedback_samples": len(synchronized_errors),
            "near_synchronous_max_position_difference_m": (
                max(synchronized_errors) if synchronized_errors else None
            ),
        })
    return results


def main():
    """解析输入与输出路径，运行只读的离线工作空间统计。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path, help="包含七关节角和法兰位姿的 CSV")
    parser.add_argument("--output", type=Path, help="将观测统计写为 JSON")
    args = parser.parse_args()
    results = analyze(args.recordings)
    document = {"kind": "observed_flange_range_only", "results": results}
    rendered = json.dumps(document, ensure_ascii=False, indent=2) + "\n"
    print(rendered, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(rendered)


if __name__ == "__main__":
    main()
