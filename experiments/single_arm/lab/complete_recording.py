"""Attest a separately completed S1 return without rewriting a failed session.

Only a time-limited teaching session that recorded settling feedback and failed
before publishing its automatic return can use this path. The original CSV and
metadata remain unchanged; the completion sidecar binds a successful live
move_j return report to their hashes. This command sends no robot commands.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


PLAN_DIR = Path("experiments/single_arm/lab/data/planned_returns")
RUN_DIR = Path("experiments/single_arm/lab/data/planned_return_runs")
RECORDING_DIR = Path("experiments/single_arm/teaching/data/recordings")
PREFLIGHT_ERRORS = (
    "当前未停稳或控制器/七轴状态不允许回位",
    "回位实时预检未通过：",
    "退出拖动后 3 次只读复测仍未停稳",
)


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _within(path, directory):
    path, directory = Path(path).resolve(), Path(directory).resolve()
    if not path.is_relative_to(directory):
        raise ValueError("文件不在指定实验目录")
    return path


def _max_error(first, second):
    if len(first) != 7 or len(second) != 7:
        raise ValueError("回位凭证需要七个关节角")
    return max(abs(float(a)-float(b)) for a, b in zip(first, second))


def _check_source(metadata):
    if (metadata.get("schema_version") != 2 or
            metadata.get("stop_reason") != "time_limit" or
            metadata.get("return_completed") is not False or
            metadata.get("return_plan") is not None or
            metadata.get("post_stop_samples", 0) < 5 or
            metadata.get("emergency_stop_requested") or
            not str(metadata.get("error", "")).startswith(PREFLIGHT_ERRORS)):
        raise ValueError("只允许完整限时示教、停稳样本齐全且仅回位预检失败的记录")


def _last_joint(recording_path):
    with Path(recording_path).open(newline="", encoding="utf-8") as stream:
        last = None
        for last in csv.DictReader(stream):
            pass
    if last is None:
        raise ValueError("原始示教 CSV 没有采样")
    return [float(last[f"joint_{index}_rad"]) for index in range(1, 8)]


def validate_completion(recording_path, metadata, config, *, config_path,
                        recording_sha256, metadata_sha256):
    """Validate the sidecar and both immutable return artifacts, offline."""
    _check_source(metadata)
    recording_path = _within(recording_path, RECORDING_DIR)
    completion_path = recording_path.with_suffix(".completion.json")
    completion = json.loads(completion_path.read_text(encoding="utf-8"))
    if (completion.get("schema_version") != 1 or
            completion.get("kind") != "external_s1_return_completion" or
            completion.get("recording_sha256") != recording_sha256 or
            completion.get("metadata_sha256") != metadata_sha256 or
            completion.get("config_sha256") != _sha(config_path)):
        raise ValueError("独立回位凭证与原始记录或配置哈希不匹配")
    plan_path = _within(completion["return_plan"], PLAN_DIR)
    report_path = _within(completion["return_report"], RUN_DIR)
    if (completion.get("return_plan_sha256") != _sha(plan_path) or
            completion.get("return_report_sha256") != _sha(report_path)):
        raise ValueError("独立回位计划或报告已变化")
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if (plan.get("kind") != "live_planned_return_to_s1" or
            plan.get("target_name") != "S1" or
            plan.get("config_sha256") != completion["config_sha256"] or
            plan.get("source_recording") is not None or
            plan.get("reversed_recording") is not False or
            plan.get("controller_targets", 0) < 1 or
            _max_error(plan.get("target_joint_rad", []),
                       config["safe_start_joint_rad"]) > 1e-9 or
            _max_error(plan.get("start_joint_rad", []),
                       _last_joint(recording_path)) > 0.005):
        raise ValueError("独立回位计划未从本次示教终点到 S1")
    final = report.get("final_state") or {}
    attested = completion.get("attested_state") or {}
    if (not report.get("completed") or not report.get("motion_attempted") or
            Path(report.get("plan", "")).resolve() != plan_path or
            report.get("error") or
            report.get("final_max_joint_error_rad", 1) > 0.005 or
            not final.get("ready_for_motion") or
            final.get("control_mode") != 1 or final.get("arm_status") != 0 or
            final.get("error_code") != 0 or
            _max_error(final.get("joint_rad", []),
                       config["safe_start_joint_rad"]) > 0.005 or
            not attested.get("ready_for_motion") or
            _max_error(attested.get("joint_rad", []),
                       final.get("joint_rad", [])) > 0.005):
        raise ValueError("独立回位报告或现场复核未证明到达 S1")
    if not (datetime.fromisoformat(metadata["finished_at_utc"])
            <= datetime.fromisoformat(plan["created_at_utc"])
            <= datetime.fromisoformat(report["started_at_utc"])
            <= datetime.fromisoformat(report["finished_at_utc"])
            <= datetime.fromisoformat(completion["attested_at_utc"])):
        raise ValueError("示教、回位及完成凭证的时间顺序错误")
    return completion_path


def attest(recording_path, report_path, *, config_path=DEFAULT_CONFIG,
           robot_factory=None):
    """Read the current S1 state and create one completion sidecar; no motion."""
    recording_path = _within(recording_path, RECORDING_DIR)
    config_path = Path(config_path)
    config = load_config(config_path)
    metadata_path = recording_path.with_suffix(".json")
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    _check_source(metadata)
    report_path = _within(report_path, RUN_DIR)
    report = json.loads(report_path.read_text(encoding="utf-8"))
    plan_path = _within(report["plan"], PLAN_DIR)
    with connected(config, robot_factory) as robot:
        state = require_ready(read_state(robot))
    if _max_error(state["joint_rad"], config["safe_start_joint_rad"]) > 0.005:
        raise ValueError("当前机械臂未在 S1；不生成完成凭证")
    completion_path = recording_path.with_suffix(".completion.json")
    if completion_path.exists():
        raise ValueError("这份记录已有独立完成凭证；不会覆盖")
    completion = {
        "schema_version": 1, "kind": "external_s1_return_completion",
        "recording_sha256": _sha(recording_path),
        "metadata_sha256": _sha(metadata_path),
        "config_sha256": _sha(config_path),
        "return_plan": str(plan_path), "return_plan_sha256": _sha(plan_path),
        "return_report": str(report_path), "return_report_sha256": _sha(report_path),
        "attested_at_utc": datetime.now(timezone.utc).isoformat(),
        "attested_state": state,
    }
    with completion_path.open("x", encoding="utf-8") as stream:
        json.dump(completion, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    try:
        validate_completion(recording_path, metadata, config,
                            config_path=config_path,
                            recording_sha256=completion["recording_sha256"],
                            metadata_sha256=completion["metadata_sha256"])
    except Exception:
        completion_path.unlink()
        raise
    return completion_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", type=Path, required=True)
    parser.add_argument("--return-report", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        path = attest(args.recording, args.return_report)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as exc:
        print(f"示教完成凭证未生成：{exc}", file=sys.stderr)
        return 1
    print("独立回位完成凭证：", path)
    print("原始 CSV/JSON 未改动；可执行 records latest 做只读校验。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
