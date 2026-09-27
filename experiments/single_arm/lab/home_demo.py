"""Record a hand-guided route to a proposed supported shutdown pose.

The route is evidence for later planning, never a command to replay or disable.
"""

import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

from experiments.single_arm.teaching.teach_session import load_config


DATA_DIR = Path("experiments/single_arm/lab/data/home_demos")


def _command(runner, command):
    result = runner(command, capture_output=True, text=True, check=False)
    return {"command": command, "returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr}


def _summary(path):
    with Path(path).open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("回零演示没有有效关节和法兰样本")
    def joints(row):
        return [float(row[f"joint_{i}_rad"]) for i in range(1, 8)]
    def xyz(row):
        return [float(row[key]) for key in ("x_m", "y_m", "z_m")]
    start_q, end_q = joints(rows[0]), joints(rows[-1])
    start_xyz, end_xyz = xyz(rows[0]), xyz(rows[-1])
    last_elapsed = float(rows[-1]["elapsed_s"])
    tail = [row for row in rows if float(row["elapsed_s"]) >= last_elapsed - 1.0]
    summary = {"samples": len(rows), "elapsed_s": last_elapsed,
            "start_joint_rad": start_q, "end_joint_rad": end_q,
            "start_flange_xyz_m": start_xyz, "end_flange_xyz_m": end_xyz,
            "max_joint_excursion_rad": max(
                max(abs(a-b) for a, b in zip(start_q, joints(row))) for row in rows),
            "max_flange_excursion_m": max(
                math.dist(start_xyz, xyz(row)) for row in rows),
            "final_joint_variation_last_1s_rad": max(
                max(abs(a-b) for a, b in zip(end_q, joints(row))) for row in tail),
            "final_flange_variation_last_1s_m": max(
                math.dist(end_xyz, xyz(row)) for row in tail)}
    summary["demonstrated_motion"] = (
        summary["max_joint_excursion_rad"] >= 0.03
        or summary["max_flange_excursion_m"] >= 0.01)
    summary["endpoint_stable"] = (
        summary["final_joint_variation_last_1s_rad"] <= 0.005
        and summary["final_flange_variation_last_1s_m"] <= 0.003)
    return summary


def _post_stop_reasons(state):
    """Drag may exit with ctrl_mode=2; that is safe for recording, not motion."""
    reasons = []
    if state.get("control_mode") not in (1, 2):
        reasons.append("控制模式既不是 CAN 也不是示教模式")
    if state.get("arm_status") != 0 or state.get("error_code") != 0:
        reasons.append("机械臂状态异常或有错误码")
    if state.get("teach_status") not in (0, 2, 6):
        reasons.append("拖动示教尚未退出")
    if state.get("joint_variation_0_25s_rad", float("inf")) > 0.002:
        reasons.append("七轴反馈尚未停稳")
    drivers = state.get("drivers", [])
    if len(drivers) != 7 or any(
        not item.get("enabled") or item.get("undervoltage")
        or item.get("driver_error") for item in drivers
    ):
        reasons.append("七轴中存在失能、欠压或驱动故障")
    # read_state checks FK consistency and feedback freshness before returning.
    return reasons


def _final_assessment(summary, state):
    summary["endpoint_shift_after_stop_rad"] = max(
        abs(a-b) for a, b in zip(summary["end_joint_rad"], state["joint_rad"]))
    reasons = _post_stop_reasons(state)
    if not summary["demonstrated_motion"]:
        reasons.append("几乎没有记录到拖动，不能作为回零路线")
    if not summary["endpoint_stable"]:
        reasons.append("录制终点仍在移动；须在回零位停稳至少 2 秒")
    if summary["endpoint_shift_after_stop_rad"] > 0.02:
        reasons.append("退出拖动后姿态发生变化；不能把录制末端当作回零位")
    return reasons


def record_home_demo(api, *, duration_s=30, rate_hz=20,
                     command_runner=None, data_dir=None):
    """Precheck, enter drag, record, exit drag, and preserve a full report.

    Always attempts to end drag after attempting to start it. The resulting
    endpoint is only a candidate until physical support is confirmed on site.
    """
    if not (math.isfinite(duration_s) and 3 <= duration_s <= 300):
        raise ValueError("回零演示时长须在 3～300 秒")
    if not (math.isfinite(rate_hz) and 5 <= rate_hz <= 20):
        raise ValueError("回零演示采样频率须在 5～20 Hz")
    config_path = Path(api.config_path)
    config = load_config(config_path)
    state = api.status()
    if not state["ready_for_motion"]:
        raise RuntimeError("拖动开始前设备未就绪：" + "；".join(state["reasons"]))
    directory = Path(data_dir or DATA_DIR)
    recording_dir = directory / "recordings"
    directory.mkdir(parents=True, exist_ok=True)
    recording_dir.mkdir(parents=True, exist_ok=True)
    report_path = directory / ("nero_home_demo_" + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    report = {"schema_version": 1, "kind": "hand_guided_supported_home_demo",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "config_path": str(config_path),
              "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
              "mount": config["mount"], "channel": config["channel"],
              "initial_state": state, "duration_s": duration_s,
              "rate_hz": rate_hz, "physical_support_confirmed": False,
              "automatic_return": False, "automatic_disable": False,
              "route_approved_for_motion": False}
    with report_path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    runner = command_runner or subprocess.run
    base = [sys.executable, "-m", "experiments.single_arm.can_setup.can_drag_teach"]
    start_command = base + ["--start", "--mount", config["mount"],
                            "--channel", config["channel"]]
    stop_command = base + ["--stop", "--channel", config["channel"]]
    failure = None
    started = False
    old_files = set(recording_dir.glob("*.csv"))
    try:
        started = True
        report["start"] = _command(runner, start_command)
        if report["start"]["returncode"] != 0:
            raise RuntimeError("CAN 拖动开始命令未确认")
        print(f"已进入拖动，开始记录 {duration_s:g} 秒；请沿回零路线移动，"
              "终点在实体支撑位停稳至少 2 秒。", flush=True)
        report["recording"] = api.record_poses(
            duration_s=duration_s, rate_hz=rate_hz,
            output_dir=recording_dir, required_state="drag")
    except (Exception, KeyboardInterrupt) as exc:
        failure = exc
    finally:
        if started:
            try:
                report["stop"] = _command(runner, stop_command)
                if report["stop"]["returncode"] != 0:
                    raise RuntimeError("CAN 拖动结束命令未确认")
            except (Exception, KeyboardInterrupt) as exc:
                report["stop_error"] = str(exc)
                failure = failure or exc
    new_files = sorted(set(recording_dir.glob("*.csv")) - old_files)
    if new_files:
        report["raw_recording_path"] = str(new_files[-1])
    if failure is None:
        try:
            report["trajectory_summary"] = _summary(report["recording"]["recording_path"])
            report["post_stop_state"] = api.status()
            summary = report["trajectory_summary"]
            reasons = _final_assessment(summary, report["post_stop_state"])
            if reasons:
                raise RuntimeError("录制结束复核未通过：" + "；".join(reasons))
            report["requires_can_mode_before_motion"] = (
                report["post_stop_state"]["control_mode"] != 1)
            report["completed"] = True
        except (Exception, KeyboardInterrupt) as exc:
            failure = exc
    if failure is not None:
        report["error_type"] = type(failure).__name__
        report["error"] = str(failure)
    report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    if failure is not None:
        raise RuntimeError(f"回零拖动演示未完成：{failure}；报告 {report_path}") from failure
    return {"report_path": str(report_path),
            "recording_path": report["recording"]["recording_path"],
            "trajectory_summary": report["trajectory_summary"],
            "post_stop_state": report["post_stop_state"],
            "requires_can_mode_before_motion": report["requires_can_mode_before_motion"],
            "automatic_return": False, "automatic_disable": False}


def reassess_saved_demo(report_path, *, assessment_dir=None):
    """Validate an earlier report and CSV without connecting to the robot.

    Writes new evidence; the original run report and CSV remain byte-for-byte
    unchanged. This also corrects a former false failure on idle teach mode.
    """
    source = Path(report_path)
    original_bytes = source.read_bytes()
    old = json.loads(original_bytes)
    if old.get("kind") != "hand_guided_supported_home_demo":
        raise ValueError("不是回零拖动演示报告")
    recording = old.get("recording", {})
    csv_path = Path(recording["recording_path"])
    csv_sha = hashlib.sha256(csv_path.read_bytes()).hexdigest()
    if csv_sha != recording.get("sha256"):
        raise ValueError("原始 CSV 哈希与报告不符")
    if (old.get("start", {}).get("returncode") != 0
            or old.get("stop", {}).get("returncode") != 0):
        raise ValueError("拖动开始或结束命令未成功；不能复核为完整记录")
    summary = _summary(csv_path)
    state = old["post_stop_state"]
    reasons = _final_assessment(summary, state)
    directory = Path(assessment_dir or source.parent)
    directory.mkdir(parents=True, exist_ok=True)
    result = {"schema_version": 1,
              "kind": "home_demo_offline_reassessment",
              "assessed_at_utc": datetime.now(timezone.utc).isoformat(),
              "source_report": str(source),
              "source_report_sha256": hashlib.sha256(original_bytes).hexdigest(),
              "recording_path": str(csv_path), "recording_sha256": csv_sha,
              "recording_accepted": not reasons, "reasons": reasons,
              "trajectory_summary": summary,
              "post_stop_control_mode": state["control_mode"],
              "requires_can_mode_before_motion": state["control_mode"] != 1,
              "physical_support_confirmed": False,
              "route_approved_for_motion": False}
    output = directory / ("nero_home_demo_assessment_" +
                          datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".json")
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return {"assessment_path": str(output), **result}
