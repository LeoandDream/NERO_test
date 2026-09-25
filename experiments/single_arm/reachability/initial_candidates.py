#!/usr/bin/env python3
"""从原始实测示教姿态筛选非零初始化候选；纯离线，不发送 CAN。"""

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from experiments.single_arm.continuous_replay.continuous_session import robot_instance
from experiments.single_arm.continuous_replay.trajectory import DEFAULT_SOURCE, read_source
from experiments.single_arm.replay.replay_session import build_cycle
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, JOINT_LIMITS, load_config


OUT = Path("experiments/single_arm/reachability/data/candidates")
CONTINUOUS_RUNS = Path("experiments/single_arm/continuous_replay/data/runs")
REPLAY_REPORT = Path(
    "experiments/single_arm/replay/data/replays/nero_replay_20260925T152748Z.json"
)


def grid_error_deg(joints, grid_deg=5):
    """每关节到最近 5° 整数倍的绝对偏差，单位 degree。"""
    return [abs(math.degrees(value) - round(math.degrees(value) / grid_deg) * grid_deg)
            for value in joints]


def latest_verified_full_path(source_sha256):
    """仅把独立验收通过的全程实测反馈用作路径近邻证据。"""
    for path in sorted(CONTINUOUS_RUNS.glob("nero_continuous_*.json"), reverse=True):
        if path.stem.endswith("_analysis"):
            continue
        analysis_path = path.with_name(path.stem + "_analysis.json")
        if not analysis_path.exists():
            continue
        report = json.loads(path.read_text(encoding="utf-8"))
        analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
        if not (report.get("completed") and analysis.get("acceptance_pass")
                and analysis.get("full_source_used")
                and report.get("source_sha256") == source_sha256):
            continue
        with Path(report["samples_csv"]).open(newline="", encoding="utf-8") as stream:
            samples = [(int(row["index"]), json.loads(row["joint_feedback_rad"]))
                       for row in csv.DictReader(stream) if row["phase"] == "forward"]
        return str(path), samples
    return None, []


def candidates(source, config, count=5):
    report = json.loads(REPLAY_REPORT.read_text(encoding="utf-8"))
    if (report.get("source_recording") != source["path"]
            or len(report.get("cycles", [])) != 1
            or not report["cycles"][0].get("completed")):
        raise ValueError("旧全程回放证据缺失，不可标注旧路径近邻")
    cycle, forward_steps = build_cycle(source["joints"], source["original"],
                                       source["original"], config)
    forward = cycle[:forward_steps + 1]
    robot = robot_instance(config["channel"])
    full_path, full_feedback = latest_verified_full_path(source["sha256"])
    s1 = config["safe_start_joint_rad"]
    pool = []
    for index in range(0, source["teaching_rows"], 5):
        q = source["joints"][index]
        distance = max(abs(a-b) for a, b in zip(q, s1))
        margin = min(min(value-low, high-value)
                     for value, (low, high) in zip(q, JOINT_LIMITS))
        flange = [float(value) for value in robot.fk(q)]
        if not (0.10 <= distance <= 1.50 and margin >= 0.15 and flange[0] >= 0.22):
            continue
        error = grid_error_deg(q)
        nearest_step, nearest_distance = min(
            ((step, max(abs(a-b) for a, b in zip(q, point)))
             for step, point in enumerate(forward)), key=lambda pair: pair[1]
        )
        pool.append({
            "source_row": index,
            "source_elapsed_s": source["times"][index],
            "joint_rad": q,
            "joint_deg": [math.degrees(value) for value in q],
            "flange_pose_m_rad": flange,
            "limit_margin_rad": margin,
            "max_joint_distance_from_s1_rad": distance,
            "grid_deg": 5,
            "per_joint_grid_error_deg": error,
            "mean_grid_error_deg": sum(error)/7,
            "max_grid_error_deg": max(error),
            "nearest_old_replay_forward_step": nearest_step,
            "nearest_old_replay_max_joint_error_rad": nearest_distance,
            "source": str(DEFAULT_SOURCE),
            "evidence_level": "historical_manual_sample_only",
            "path_evidence": ("old_replay_nearby" if nearest_distance <= 0.01
                              else "historical_teaching_only"),
            "current_site_arrival_verified": False,
            "usable_initial_pose": False,
        })
        if full_feedback:
            sample_index, sample_distance = min(
                ((sample_index, max(abs(a-b) for a,b in zip(q, joints)))
                 for sample_index, joints in full_feedback), key=lambda pair: pair[1]
            )
            pool[-1]["new_full_replay_source"] = full_path
            pool[-1]["new_full_replay_nearest_feedback_index"] = sample_index
            pool[-1]["new_full_replay_nearest_max_joint_error_rad"] = sample_distance
            pool[-1]["new_full_replay_nearby_not_held"] = True
    selected = []
    for item in sorted(pool, key=lambda row: (row["mean_grid_error_deg"],
                                              -row["limit_margin_rad"])):
        if all(abs(item["source_elapsed_s"] - prior["source_elapsed_s"]) >= 1.5
               and max(abs(a-b) for a, b in zip(item["joint_rad"], prior["joint_rad"])) >= 0.15
               for prior in selected):
            selected.append(item)
            if len(selected) == count:
                break
    if len(selected) < count:
        raise ValueError(f"符合多样性和边界条件的候选只有 {len(selected)} 个")
    for number, item in enumerate(selected, 1):
        item["name"] = f"C{number}"
    return selected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    args = parser.parse_args()
    config = load_config(args.config)
    source = read_source(args.source, config)
    selected = candidates(source, config)
    output = OUT / ("nero_init_candidates_" +
                    datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump({
            "schema_version": 1,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "source_sha256": source["sha256"],
            "score_definition": "mean of seven absolute errors to nearest 5 degree grid; lower is tidier",
            "selection_constraints": "max |q-S1| in 0.10..1.50 rad, min SDK limit margin >=0.15 rad, flange base X>=0.22m, pairwise time separation>=1.5s and max joint separation>=0.15rad",
            "old_replay_evidence": str(REPLAY_REPORT),
            "new_full_replay_evidence": latest_verified_full_path(source["sha256"])[0],
            "candidates": selected,
            "warning": "历史拖动样本和旧路径近邻不证明当前现场可安全到位；当前全部为未实机验证候选。",
        }, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print("非零初始化候选：", output)
    for item in selected:
        print(item["name"], f"源行 {item['source_row']}",
              f"平均整五度误差 {item['mean_grid_error_deg']:.2f}°",
              f"最小限位余量 {item['limit_margin_rad']:.3f} rad",
              f"当前实机验证：{item['current_site_arrival_verified']}")
    print("仅离线筛选；未连接机械臂、未发送运动指令。")


if __name__ == "__main__":
    main()
