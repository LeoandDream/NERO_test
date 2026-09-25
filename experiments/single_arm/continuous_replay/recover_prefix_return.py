#!/usr/bin/env python3
"""在候选停留误判后，沿已验证的源前缀原路回 S1。"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

from experiments.single_arm.continuous_replay.continuous_session import (
    PLAN_DIR, check_plan, load_config, require_prior_stage, write_json_exclusive,
)
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG


def derive_return(failed_report_path, config):
    """只从完整校验过的原计划及明确停稳的失败报告构造倒序路线。"""
    path = Path(failed_report_path)
    report = json.loads(path.read_text(encoding="utf-8"))
    parent_path = Path(report["plan"])
    parent = check_plan(parent_path, config)
    if (report.get("completed") is not False
            or report.get("stop_action") != "move_j_hold_current_confirmed"
            or report.get("source_sha256") != parent["source_sha256"]
            or parent.get("turnaround_hold_s", 0) <= 0
            or report.get("error") != "源前缀终点未连续停稳至少 0.5 秒，不反向"):
        raise ValueError("失败报告不是本次已保持住姿态的候选停留误判")
    waypoints = parent["waypoints"]
    last = int(report["last_published_index"])
    if last + 1 >= len(waypoints) or waypoints[last]["phase"] != "hold":
        raise ValueError("报告中断位置不在前缀终点停留段")
    endpoint = list(waypoints[last]["joint_rad"])
    if (max(abs(a-b) for a,b in zip(endpoint, report["hold_joint_rad"])) > 0.005
            or any(item["phase"] != "return" for item in waypoints[last+2:])):
        raise ValueError("停稳姿态或倒序路线与已验证计划不一致")
    # 把最后一个静止目标作为恢复起点，随后逐点使用原计划的倒序关节目标。
    items = [dict(waypoints[last + 1], phase="return", t_s=0.0)]
    baseline = waypoints[last + 1]["t_s"]
    items.extend(dict(item, t_s=item["t_s"]-baseline)
                 for item in waypoints[last + 2:])
    derived = dict(parent)
    derived.update({
        "waypoints": items,
        "turnaround_hold_s": 0.0,
        "total_duration_s": items[-1]["t_s"],
        "return_to_start": False,
        "recovery_mode": True,
        "recovery_parent_plan": str(parent_path),
        "recovery_parent_plan_sha256": hashlib.sha256(parent_path.read_bytes()).hexdigest(),
        "recovery_parent_report": str(path),
        "recovery_parent_report_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "recovery_source_start_index": last+1,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
    })
    return derived


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--failed-report", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    try:
        config = load_config(args.config)
        plan = derive_return(args.failed_report, config)
        require_prior_stage(plan)
        path = PLAN_DIR / ("nero_recover_prefix_" + datetime.now(timezone.utc).strftime(
            "%Y%m%dT%H%M%S%fZ") + ".json")
        write_json_exclusive(path, plan)
        print("倒序恢复计划：", path)
        print(f"沿原记录返回 S1：{len(plan['waypoints'])} 个连续目标，"
              f"{plan['total_duration_s']:.1f} 秒，{plan['frequency_hz']} Hz。")
        if not args.run:
            print("只读生成计划；未连接机械臂或发送运动指令。")
            return 0
        from experiments.single_arm.continuous_replay.run_stream import run
        run(plan, path, config)
        return 0
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"倒序恢复未完成：{exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
