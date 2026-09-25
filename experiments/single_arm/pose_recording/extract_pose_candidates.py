#!/usr/bin/env python3
"""从 Nero 拖动示教记录中提取有效轨迹和短暂停留的候选点。"""

import argparse
import csv
import math
from pathlib import Path


JOINT_COLUMNS = [f"joint_{index}_rad" for index in range(1, 8)]
POSE_COLUMNS = ["x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"]


def joint_values(row):
    """从一行 CSV 提取七个有限关节角；缺失或异常值不参与候选点计算。"""
    values = [float(row[column]) for column in JOINT_COLUMNS]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("关节角包含非有限值")
    return values


def main():
    """先筛选拖动状态的样本，再从低速停留段生成候选点 CSV。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="record_poses.py 生成的原始 CSV")
    parser.add_argument("--window", type=int, default=5, help="停留检测的半窗口采样数，默认 5")
    parser.add_argument("--threshold", type=float, default=0.006,
                        help="窗口首尾的关节空间距离阈值，单位 rad，默认 0.006")
    args = parser.parse_args()
    if args.window < 1 or args.threshold <= 0:
        parser.error("--window 必须 >= 1，--threshold 必须 > 0")

    with args.recording.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames:
            parser.error("CSV 没有表头")
        required = set(JOINT_COLUMNS + POSE_COLUMNS + [
            "sample_index", "captured_at_utc", "elapsed_s",
            "control_mode", "arm_status", "teach_status",
        ])
        if not required.issubset(reader.fieldnames):
            parser.error(f"CSV 缺少字段：{sorted(required - set(reader.fieldnames))}")
        fieldnames = reader.fieldnames
        rows = list(reader)

    # 非拖动示教阶段的静止样本不能被误认为人工挑选的目标点。
    active = []
    for row in rows:
        if (row["control_mode"], row["arm_status"], row["teach_status"]) != (
            "TEACHING_MODE", "NORMAL", "START_RECORDING"
        ):
            continue
        joint_values(row)
        if not all(math.isfinite(float(row[column])) for column in POSE_COLUMNS):
            raise ValueError(f"样本 {row['sample_index']} 位姿包含非有限值")
        active.append(row)
    if not active:
        parser.error("没有找到 TEACHING_MODE / NORMAL / START_RECORDING 样本")

    teaching_path = args.recording.with_name(args.recording.stem + "_teaching_only.csv")
    candidates_path = Path("experiments/single_arm/pose_recording/data/poses") / (args.recording.stem + "_held_candidates.csv")
    candidates_path.parent.mkdir(parents=True, exist_ok=True)
    for path in (teaching_path, candidates_path):
        if path.exists():
            parser.error(f"输出文件已存在：{path}")

    angles = [joint_values(row) for row in active]
    times = [float(row["elapsed_s"]) for row in active]
    # 比较时间窗口两端的七维关节距离，连续满足阈值的片段才算停留。
    stable = []
    window = args.window
    for index in range(window, len(active) - window):
        if (times[index + window] - times[index - window] <= 1.5
                and math.dist(angles[index - window], angles[index + window])
                <= args.threshold):
            stable.append(index)

    # 将相邻稳定样本合并，取每段中心样本代表一个候选姿态。
    groups = []
    for index in stable:
        if not groups or index != groups[-1][-1] + 1:
            groups.append([index])
        else:
            groups[-1].append(index)

    candidate_fields = [
        "candidate_id", "source_recording", "source_sample_index",
        "captured_at_utc", "stable_window_s", *JOINT_COLUMNS, *POSE_COLUMNS,
    ]
    candidates = []
    for group in groups:
        if len(group) < 5:
            continue
        center = group[len(group) // 2]
        row = active[center]
        candidates.append({
            "candidate_id": f"P{len(candidates) + 1}",
            "source_recording": str(args.recording),
            "source_sample_index": row["sample_index"],
            "captured_at_utc": row["captured_at_utc"],
            "stable_window_s": round(times[group[-1]] - times[group[0]], 3),
            **{column: row[column] for column in JOINT_COLUMNS + POSE_COLUMNS},
        })

    with teaching_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(active)
    with candidates_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=candidate_fields)
        writer.writeheader()
        writer.writerows(candidates)

    print(f"示教轨迹：{len(active)} 条 -> {teaching_path}")
    print(f"停留候选点：{len(candidates)} 个 -> {candidates_path}")


if __name__ == "__main__":
    main()
