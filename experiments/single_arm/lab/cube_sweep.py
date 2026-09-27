"""一次命令录制拖动边界，再离线提取立方体极值候选。

复用现场已验证的 can_drag_teach 单次开始/结束命令。即使录制失败也
单独发送结束拖动；原始 CSV 和失败报告保留，不发送任何回位运动。
"""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

from experiments.single_arm.teaching.teach_session import load_config


REPORT_DIR = Path("experiments/single_arm/workspace_cube/data/sweeps")


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _save(report, path):
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")


def _command(runner, command):
    result = runner(command, capture_output=True, text=True, check=False)
    return {"command": command, "returncode": result.returncode,
            "stdout": result.stdout, "stderr": result.stderr}


def run_sweep(api, *, duration_s=30, rate_hz=20, margin_m=0.01,
              command_runner=None, report_dir=None):
    """状态预检后开启拖动、采样、退出拖动并生成几何候选。

    接受可注入的子进程执行器供离线验证。只有 can_drag_teach 子命令
    会写 CAN；极值分析只读取本次新 CSV，不使用历史计划。
    """
    if not (math.isfinite(duration_s) and 3 <= duration_s <= 300):
        raise ValueError("边界拖动时长须在 3～300 秒")
    if not (math.isfinite(rate_hz) and 5 <= rate_hz <= 20):
        raise ValueError("边界采样频率须在 5～20 Hz")
    if not (math.isfinite(margin_m) and margin_m >= 0):
        raise ValueError("立方体候选内缩余量须为非负有限米数")

    config_path = Path(api.config_path)
    config = load_config(config_path)
    state = api.status()
    if not state["ready_for_motion"]:
        raise RuntimeError("拖动开始前设备未就绪：" + "；".join(state["reasons"]))

    directory = Path(report_dir or REPORT_DIR)
    directory.mkdir(parents=True, exist_ok=True)
    report_path = directory / ("nero_cube_sweep_" + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    report = {"schema_version": 1, "kind": "cube_boundary_drag_sweep",
              "started_at_utc": _utc_now(), "completed": False,
              "config_path": str(config_path),
              "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
              "mount": config["mount"], "channel": config["channel"],
              "zero_reference": config["safe_start_source"],
              "coordinate_frame": "robot_base", "reference_point": "flange_center",
              "tool_offset_m": [0, 0, 0],
              "duration_s": duration_s, "rate_hz": rate_hz,
              "candidate_margin_m": margin_m, "initial_state": state,
              "automatic_return": False, "geometry_valid": False,
              "six_faces_sampled": False, "motion_region_verified": False}
    with report_path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")

    runner = command_runner or subprocess.run
    base = [sys.executable, "-m",
            "experiments.single_arm.can_setup.can_drag_teach"]
    start_command = base + ["--start", "--mount", config["mount"],
                            "--channel", config["channel"]]
    stop_command = base + ["--stop", "--channel", config["channel"]]
    failure = None
    start_attempted = False
    try:
        start_attempted = True
        report["start"] = _command(runner, start_command)
        if report["start"]["returncode"] != 0:
            raise RuntimeError("CAN 拖动开始命令未确认")
        recording = api.record_poses(duration_s=duration_s, rate_hz=rate_hz,
                                     required_state="drag")
        report["recording"] = recording
    except (Exception, KeyboardInterrupt) as exc:
        failure = exc
    finally:
        if start_attempted:
            try:
                report["stop"] = _command(runner, stop_command)
                if report["stop"]["returncode"] != 0:
                    raise RuntimeError("CAN 拖动结束命令未确认")
            except (Exception, KeyboardInterrupt) as exc:
                report["stop_error"] = str(exc)
                failure = failure or exc

    if failure is None:
        try:
            result = api.cube_extrema_from_recording(
                report["recording"]["recording_path"], margin_m=margin_m)
            report["map_path"] = result["map_path"]
            report["candidate_path"] = result["candidate_path"]
            report["candidate_status"] = result["candidate"]["status"]
            report["completed"] = True
        except (Exception, KeyboardInterrupt) as exc:
            failure = exc
    if failure is not None:
        report["error_type"] = type(failure).__name__
        report["error"] = str(failure)
    report["finished_at_utc"] = _utc_now()
    _save(report, report_path)
    if failure is not None:
        raise RuntimeError(f"边界拖动或极值分析未完成：{failure}；报告 {report_path}") from failure
    return {"report_path": str(report_path),
            "recording_path": report["recording"]["recording_path"],
            "map_path": report["map_path"],
            "candidate_path": report["candidate_path"],
            "candidate_status": report["candidate_status"],
            "automatic_return": False}
