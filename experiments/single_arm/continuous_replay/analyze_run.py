#!/usr/bin/env python3
"""独立复核连续回放日志；不连接 CAN，也不信任运行时的完成标志。"""

import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

from experiments.single_arm.continuous_replay.continuous_session import check_plan
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


JOINT_COMMAND_IDS = ("0x155", "0x156", "0x157", "0x170")


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def analyze(report_path, config_path=DEFAULT_CONFIG):
    report_path = Path(report_path)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    plan = check_plan(Path(report["plan"]), load_config(config_path))
    with Path(report["samples_csv"]).open(newline="", encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    with Path(report["can_trace_csv"]).open(newline="", encoding="utf-8") as stream:
        can_rows = list(csv.DictReader(stream))
    counts = Counter(row["can_id"] for row in can_rows)
    if not samples:
        raise ValueError("没有逐周期实测样本")
    periods = [float(row["period_s"]) for row in samples if row["period_s"]]
    errors = [float(row["tracking_max_rad"]) for row in samples]
    flange_errors = [float(row["flange_error_m"]) for row in samples]
    phases = Counter(row["phase"] for row in samples)
    target_changes = 0
    feedback_motion_periods = 0
    moving_target_periods = 0
    stationary_stretches = []
    still_since = None
    for earlier, later in zip(samples, samples[1:]):
        q0, q1 = json.loads(earlier["joint_target_rad"]), json.loads(later["joint_target_rad"])
        f0, f1 = json.loads(earlier["joint_feedback_rad"]), json.loads(later["joint_feedback_rad"])
        dt = float(later["actual_t_s"]) - float(earlier["actual_t_s"])
        if dt <= 0:
            raise ValueError("逐周期时间戳非递增")
        target_speed = max(abs(a-b) for a, b in zip(q0, q1)) / dt
        feedback_speed = max(abs(a-b) for a, b in zip(f0, f1)) / dt
        if max(abs(a-b) for a, b in zip(q0, q1)) > 1e-5:
            target_changes += 1
        if target_speed > 0.01:
            moving_target_periods += 1
            if feedback_speed > 0.002:
                feedback_motion_periods += 1
                if still_since is not None:
                    stationary_stretches.append(float(earlier["actual_t_s"]) - still_since)
                    still_since = None
            elif still_since is None:
                still_since = float(earlier["actual_t_s"])
        elif still_since is not None:
            stationary_stretches.append(float(earlier["actual_t_s"]) - still_since)
            still_since = None
    if still_since is not None:
        stationary_stretches.append(float(samples[-1]["actual_t_s"]) - still_since)
    status_ok = all(int(row["arm_status"]) == 0 and int(row["error_code"]) == 0
                    for row in samples)
    start = report.get("actual_start_joint_rad", [])
    offset = ([a-b for a,b in zip(start, plan["waypoints"][0]["joint_rad"])]
              if len(start) == 7 else None)
    all_targets = (len(samples) == len(plan["waypoints"])
                   and offset is not None
                   and all(int(row["index"]) == index
                           and row["phase"] == plan["waypoints"][index]["phase"]
                           and max(abs(a-b-c) for a,b,c in zip(
                               json.loads(row["joint_target_rad"]),
                               plan["waypoints"][index]["joint_rad"], offset)) <= 1e-6
                           for index, row in enumerate(samples)))
    hold_rows = [row for row in samples if row["phase"] == "hold"]
    expected_hold_count = sum(item["phase"] == "hold" for item in plan["waypoints"])
    stable_hold_tail = 0
    for row in reversed(hold_rows):
        target, feedback = (json.loads(row[key]) for key in
                            ("joint_target_rad", "joint_feedback_rad"))
        if (int(row["arm_status"]) == 0 and int(row["motion_status"]) in (0, 1)
                and max(abs(a-b) for a,b in zip(target, feedback)) <= 0.005):
            stable_hold_tail += 1
        else:
            break
    stable_tail_rows = hold_rows[-stable_hold_tail:] if stable_hold_tail else []
    stable_tail_variation = max(
        max(json.loads(row["joint_feedback_rad"])[joint] for row in stable_tail_rows)
        - min(json.loads(row["joint_feedback_rad"])[joint] for row in stable_tail_rows)
        for joint in range(7)
    ) if stable_tail_rows else None
    hold_verified = (len(hold_rows) == expected_hold_count
                     and (expected_hold_count == 0 or (
                         stable_hold_tail >= max(10, plan["frequency_hz"] // 2)
                         and stable_tail_variation is not None
                         and stable_tail_variation <= 0.002)))
    drivers = report.get("driver_snapshots", [])
    drivers_healthy = all(len(snapshot.get("drivers", [])) == 7
                          and all(item.get("enabled") and item.get("voltage_v", 0) > 0
                                  for item in snapshot["drivers"])
                          for snapshot in drivers)
    command_frames_complete = all(counts[key] >= len(samples) for key in JOINT_COMMAND_IDS)
    period_p99 = percentile(periods, 0.99) if periods else None
    return_error = report.get("max_return_joint_error_rad")
    result = {
        "schema_version": 1,
        "report": str(report_path),
        "plan": report["plan"],
        "source_sha256": plan["source_sha256"],
        "full_source_used": plan["source_used_rows"] == plan["source_total_rows"],
        "planned_targets": len(plan["waypoints"]),
        "logged_targets": len(samples),
        "phase_counts": dict(phases),
        "all_targets_logged_in_order": all_targets,
        "turnaround_hold_expected_periods": expected_hold_count,
        "turnaround_hold_logged_periods": len(hold_rows),
        "turnaround_stable_tail_periods": stable_hold_tail,
        "turnaround_stable_tail_variation_rad": stable_tail_variation,
        "turnaround_hold_verified": hold_verified,
        "changed_target_periods": target_changes,
        "moving_target_periods": moving_target_periods,
        "feedback_moving_periods": feedback_motion_periods,
        "stationary_during_target_motion_count_over_0_3_s": sum(
            value >= 0.3 for value in stationary_stretches),
        "longest_stationary_during_target_motion_s": max(stationary_stretches, default=0),
        "period_p99_s": period_p99,
        "period_max_s": max(periods, default=None),
        "max_tracking_error_rad": max(errors),
        "max_flange_error_m": max(flange_errors),
        "return_joint_error_rad": return_error,
        "all_logged_status_normal": status_ok,
        "can_frame_counts": dict(counts),
        "all_four_joint_command_ids_recorded_per_target": command_frames_complete,
        "driver_snapshot_count": len(drivers),
        "all_driver_snapshots_healthy": drivers_healthy,
        "run_completed_flag": report.get("completed", False),
    }
    result["acceptance_pass"] = bool(
        result["run_completed_flag"] and all_targets and hold_verified
        and command_frames_complete
        and status_ok and period_p99 is not None and period_p99 <= 0.08
        and max(errors) <= 0.10 and max(flange_errors) <= 0.05
        and return_error is not None and return_error <= 0.005
        and target_changes >= 10 and feedback_motion_periods >= 10
        and result["stationary_during_target_motion_count_over_0_3_s"] == 0
        and result["driver_snapshot_count"] >= 2 and drivers_healthy
    )
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    result = analyze(args.report, args.config)
    output = args.report.with_name(args.report.stem + "_analysis.json")
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print("独立事后分析：", output)
    return 0 if result["acceptance_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
