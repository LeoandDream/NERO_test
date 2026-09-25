#!/usr/bin/env python3
"""独立核对候选关节位点的实机到达、保持和原路返回证据。"""

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import sys

from experiments.single_arm.continuous_replay.analyze_run import analyze
from experiments.single_arm.continuous_replay.continuous_session import check_plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


OUT_DIR = Path("experiments/single_arm/reachability/data/visits")


def analyze_visit(candidates_path, name, run_path, config_path=DEFAULT_CONFIG):
    config = load_config(config_path)
    candidates = json.loads(Path(candidates_path).read_text(encoding="utf-8"))
    selected = next((item for item in candidates["candidates"] if item["name"] == name), None)
    if selected is None:
        raise ValueError(f"没有候选位点 {name}")
    report = json.loads(Path(run_path).read_text(encoding="utf-8"))
    plan = check_plan(report["plan"], config)
    independent = analyze(run_path, config_path)
    with Path(report["samples_csv"]).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    holds = [row for row in rows if row["phase"] == "hold"]
    candidate_q = selected["joint_rad"]
    hold_feedback = [json.loads(row["joint_feedback_rad"]) for row in holds]
    max_target_error = max((max(abs(a-b) for a,b in zip(q, candidate_q))
                            for q in hold_feedback), default=float("inf"))
    variation = max((max(q[joint] for q in hold_feedback)
                     - min(q[joint] for q in hold_feedback)
                     for joint in range(7)), default=float("inf"))
    final_target = next(item["joint_rad"] for item in plan["waypoints"]
                        if item["phase"] == "hold")
    exact_source_target = max(abs(a-b) for a,b in zip(final_target, candidate_q)) <= 1e-9
    arrived_and_held = bool(
        independent["acceptance_pass"]
        and report.get("turnaround_hold_confirmed") is True
        and exact_source_target
        and len(holds) >= 40
        and max_target_error <= 0.005
        and variation <= 0.002
        and report.get("max_return_joint_error_rad", 1) <= 0.005
    )
    return {
        "schema_version": 1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "candidate": name,
        "candidate_source": str(candidates_path),
        "candidate_joint_rad": candidate_q,
        "candidate_joint_deg": selected["joint_deg"],
        "candidate_flange_pose_m_rad": selected["flange_pose_m_rad"],
        "candidate_limit_margin_rad": selected["limit_margin_rad"],
        "mean_grid_error_deg": selected["mean_grid_error_deg"],
        "run_report": str(run_path),
        "plan": report["plan"],
        "independent_run_acceptance_pass": independent["acceptance_pass"],
        "hold_samples": len(holds),
        "hold_duration_s": plan["turnaround_hold_s"],
        "hold_max_joint_error_to_raw_candidate_rad": max_target_error,
        "hold_max_joint_variation_rad": variation,
        "exact_source_joint_target": exact_source_target,
        "return_joint_error_rad": report.get("max_return_joint_error_rad"),
        "current_site_arrival_and_hold_verified": arrived_and_held,
        "aesthetic_review": "AWAITING_USER",
        "configured_as_default_start": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--candidates", required=True, type=Path)
    parser.add_argument("--name", required=True)
    parser.add_argument("--run-report", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    try:
        result = analyze_visit(args.candidates, args.name, args.run_report, args.config)
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        path = OUT_DIR / (f"nero_candidate_{args.name}_"
                          + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
        with path.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        print("候选实机到位核验：", path)
        print(json.dumps({key: result[key] for key in (
            "candidate", "hold_samples", "hold_max_joint_error_to_raw_candidate_rad",
            "hold_max_joint_variation_rad", "return_joint_error_rad",
            "current_site_arrival_and_hold_verified", "aesthetic_review")}, ensure_ascii=False))
        return 0 if result["current_site_arrival_and_hold_verified"] else 1
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"候选核验失败：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
