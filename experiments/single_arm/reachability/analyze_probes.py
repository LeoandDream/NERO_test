#!/usr/bin/env python3
"""独立复核本次逐段探测的原始反馈、CAN 指令和段间连续性；不连接机械臂。"""

from collections import Counter
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path


RUN_DIR = Path("experiments/single_arm/reachability/data/runs")
ANALYSIS_DIR = Path("experiments/single_arm/reachability/data/analysis")
LINE_LIMIT_M = 0.005
JOINT_CHANGE_LIMIT_RAD = 0.25
ROTATION_CHANGE_LIMIT_RAD = 0.03
FINAL_POSITION_LIMIT_M = 0.0005


def analyze_one(path):
    report = json.loads(path.read_text(encoding="utf-8"))
    plan = json.loads(Path(report["plan"]).read_text(encoding="utf-8"))
    samples = report.get("feedback_samples", [])
    with Path(report["can_trace_csv"]).open(newline="", encoding="utf-8") as stream:
        can_counts = Counter(row["can_id"].upper() for row in csv.DictReader(stream))
    errors = []
    if not report.get("completed") or report.get("command_count") != 1 or report.get("error"):
        errors.append("未完成或运动指令数不是 1")
    if len(samples) < 5:
        errors.append("到位反馈不足 5 条")
    if samples and any(a["elapsed_s"] >= b["elapsed_s"] for a,b in zip(samples, samples[1:])):
        errors.append("反馈时间未严格递增")
    if any(sample["arm_status"] != 0 for sample in samples):
        errors.append("反馈中存在非 NORMAL 状态")
    max_line = max((item["line_error_m"] for item in samples), default=math.inf)
    max_joint = max((item["joint_change_rad"] for item in samples), default=math.inf)
    max_rotation = max((item["rotation_change_rad"] for item in samples), default=math.inf)
    if max_line > LINE_LIMIT_M or max_joint > JOINT_CHANGE_LIMIT_RAD or max_rotation > ROTATION_CHANGE_LIMIT_RAD:
        errors.append("实测路径越过预设门槛")
    if not report.get("final_flange_pose") or math.dist(
            report["final_flange_pose"][:3], plan["target_flange_pose"][:3]) > FINAL_POSITION_LIMIT_M:
        errors.append("末端未到达计划目标")
    if (len(samples) < 5 or any(item["motion_status"] != 0 or
                                 item["position_error_m"] > FINAL_POSITION_LIMIT_M
                                 for item in samples[-5:])):
        errors.append("末尾连续 5 条反馈未确认停稳到位")
    snapshots = report.get("driver_snapshots", [])
    if len(snapshots) < 2 or any(len(item.get("drivers", [])) != 7 or
                                 any(not motor["enabled"] for motor in item["drivers"])
                                 for item in snapshots):
        errors.append("七轴使能快照不足或异常")
    for frame_id in ("0X152", "0X153", "0X154"):
        if can_counts[frame_id] != 1:
            errors.append(f"CAN 指令帧 {frame_id} 数量不是 1")
    return {
        "report": str(path), "plan": report["plan"],
        "axis": plan["axis"], "delta_m": plan["delta_m"],
        "feedback_count": len(samples), "driver_snapshot_count": len(snapshots),
        "max_line_error_m": max_line,
        "max_joint_change_rad": max_joint,
        "max_rotation_change_rad": max_rotation,
        "final_position_error_m": report.get("final_position_error_m"),
        "can_command_frame_counts": {frame_id: can_counts[frame_id]
                                     for frame_id in ("0X152", "0X153", "0X154")},
        "initial_joint_rad": plan["start_joint_rad"],
        "initial_flange_pose": plan["start_flange_pose"],
        "final_joint_rad": report.get("final_joint_rad"),
        "final_flange_pose": report.get("final_flange_pose"),
        "accepted": not errors, "errors": errors,
    }


def analyze_series(paths):
    segments = [analyze_one(path) for path in paths]
    connections = []
    for left, right in zip(segments, segments[1:]):
        joint_gap = (max(abs(a-b) for a,b in zip(left["final_joint_rad"], right["initial_joint_rad"]))
                     if left["final_joint_rad"] else None)
        flange_gap = (math.dist(left["final_flange_pose"][:3], right["initial_flange_pose"][:3])
                      if left["final_flange_pose"] else None)
        connections.append({"from_report": left["report"], "to_report": right["report"],
                            "joint_gap_rad": joint_gap, "flange_gap_m": flange_gap,
                            "accepted": joint_gap is not None and flange_gap is not None
                            and joint_gap <= 0.005 and flange_gap <= 0.003})
    return {"schema_version": 1,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "segments": segments, "connections": connections,
            "accepted": bool(segments) and all(item["accepted"] for item in segments)
            and all(item["accepted"] for item in connections),
            "note": "反馈和 CAN 证明这些离散连接已执行；不证明包围盒内任意点可达，也没有现场目视观察。"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reports", type=Path, nargs="*", help="不填时复核本次全部 nero_probe_*.json")
    args = parser.parse_args()
    paths = args.reports or sorted(RUN_DIR.glob("nero_probe_*.json"))
    result = analyze_series(paths)
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output = ANALYSIS_DIR / f"nero_probe_series_{stamp}.json"
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(f"探测序列：{len(result['segments'])} 段，{len(result['connections'])} 条段间连接；独立验收：{result['accepted']}")
    print("独立分析：", output)
    return 0 if result["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
