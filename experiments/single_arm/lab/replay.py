"""新示教的显式逐点回放：发现→只读计划→复核→执行。"""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.recordings import RECORDING_DIR, resolve_recording
from experiments.single_arm.replay.replay_session import (
    REPLAY_MAX_WAYPOINT_SECONDS, REPLAY_TARGET_TOLERANCE_RAD,
    build_cycle, verify_return,
)
from experiments.single_arm.start_transfer.go_to_start import (
    RoutePaused, execute_route, flange_envelope, stable_joint_feedback,
)
from experiments.single_arm.teaching.return_session import load_recording
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_route_workspace,
)


PLAN_DIR = Path("experiments/single_arm/lab/data/replay_plans")
RUN_DIR = Path("experiments/single_arm/lab/data/replays")


def _write_new(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def _fresh_path(directory, prefix):
    return Path(directory) / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")


def prepare_replay(selector="latest", *, mode="point", config_path=DEFAULT_CONFIG,
                   recording_dir=RECORDING_DIR, speed_percent=5,
                   robot_factory=None):
    """只读连接并生成完整关节路线；新记录明确采用逐点 move_j。"""
    if mode != "point":
        raise ValueError("新示教快速回放仅支持显式 point 模式；固定 20 秒连续源请用 continuous_replay 接口")
    if speed_percent not in (5, 10):
        raise ValueError("快速回放仅允许 5% 或 10% 速度")
    config_path = Path(config_path)
    config = load_config(config_path)
    info = resolve_recording(selector, config_path, recording_dir)
    _, _, original, recorded = load_recording(info.path, config, allow_completed=True)
    with connected(config, robot_factory) as robot:
        snapshot = require_ready(read_state(robot))
        route, forward_steps = build_cycle(recorded, snapshot["joint_rad"], original, config)
        validate_route_workspace(robot, route, config)
        envelope = flange_envelope(robot, route, config)
    return {"schema_version": 1, "kind": "quick_teach_replay",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "replay_mode": "point_move_j", "recording_id": info.recording_id,
            "recording": str(info.path), "recording_sha256": info.sha256,
            "metadata": str(info.metadata_path),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "observed_start_joint_rad": snapshot["joint_rad"],
            "observed_start_flange_pose_m_rad": snapshot["flange_pose_m_rad"],
            "speed_percent": speed_percent,
            "forward_steps": forward_steps, "total_steps": len(route)-1,
            "route_joint_rad": route, "flange_xyz_envelope_m": envelope,
            "source_sample_count": info.sample_count,
            "note": "逐点到位后发下一目标；完整正向后原路返回本轮实时起点。"}


def verify_replay_plan(plan, config_path=DEFAULT_CONFIG, recording_dir=RECORDING_DIR):
    config_path = Path(config_path)
    config = load_config(config_path)
    if (plan.get("schema_version") != 1 or plan.get("kind") != "quick_teach_replay"
            or plan.get("replay_mode") != "point_move_j"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("speed_percent") not in (5,10)):
        raise ValueError("回放计划版本、模式、速度或配置不匹配")
    info = resolve_recording(plan["recording"], config_path, recording_dir)
    if (plan.get("recording_id") != info.recording_id
            or plan.get("recording_sha256") != info.sha256
            or plan.get("metadata") != str(info.metadata_path)
            or plan.get("source_sample_count") != info.sample_count):
        raise ValueError("回放计划的原始记录或元数据已改变")
    _, _, original, recorded = load_recording(info.path, config, allow_completed=True)
    expected, forward = build_cycle(recorded, plan["observed_start_joint_rad"],
                                    original, config)
    actual = plan.get("route_joint_rad")
    if (not isinstance(actual, list) or len(actual) != len(expected)
            or plan.get("forward_steps") != forward
            or plan.get("total_steps") != len(expected)-1):
        raise ValueError("回放计划路线数量或去程长度已改变")
    for item, target in zip(actual, expected):
        if len(item) != 7 or max(abs(a-b) for a,b in zip(item, target)) > 1e-9:
            raise ValueError("回放计划关节路线已改变")
    return plan


def save_replay_plan(plan, directory=PLAN_DIR):
    path = _fresh_path(directory, "nero_quick_replay_")
    _write_new(path, plan)
    return path


def execute_replay(plan_path, *, config_path=DEFAULT_CONFIG,
                   recording_dir=RECORDING_DIR, robot_factory=None,
                   trace_factory=None, run_dir=RUN_DIR, countdown_s=5):
    """显式执行一轮逐点往返；失败报告保存最后目标与实际反馈。"""
    from experiments.single_arm.replay.can_trace import ReplayCanTrace
    plan_path = Path(plan_path)
    plan = verify_replay_plan(json.loads(plan_path.read_text(encoding="utf-8")),
                              config_path, recording_dir)
    config = load_config(Path(config_path))
    if not 0 <= countdown_s <= 30:
        raise ValueError("倒计时秒数无效")
    report_path = _fresh_path(run_dir, "nero_quick_replay_")
    trace_path = report_path.with_suffix(".can.csv")
    report = {"schema_version": 1, "kind": "quick_teach_replay",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "plan": str(plan_path), "recording": plan["recording"],
              "recording_sha256": plan["recording_sha256"],
              "replay_mode": "point_move_j", "speed_percent": plan["speed_percent"],
              "total_steps": plan["total_steps"], "last_reached_step": 0,
              "last_attempted_step": 0, "last_published_step": 0,
              "feedback_samples": [], "command_events": [],
              "status_samples": [], "completed": False,
              "can_trace_csv": str(trace_path)}
    _write_new(report_path, report)
    trace = None
    previous = None
    interrupted = False

    def request_stop(_number, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            snapshot = require_ready(read_state(robot))
            if max(abs(a-b) for a,b in zip(snapshot["joint_rad"],
                                            plan["observed_start_joint_rad"])) > 0.002:
                raise RuntimeError("实时姿态已偏离回放计划起点 0.002 rad；须重新规划")
            route = plan["route_joint_rad"]
            validate_route_workspace(robot, route, config)
            current_envelope = flange_envelope(robot, route, config)
            planned_envelope = plan.get("flange_xyz_envelope_m")
            if (not isinstance(planned_envelope, dict)
                    or any(axis not in planned_envelope
                           or len(planned_envelope[axis]) != 2
                           or any(abs(a-b) > 1e-8 for a,b in zip(
                               planned_envelope[axis], current_envelope[axis]))
                           for axis in ("x", "y", "z"))):
                raise ValueError("回放计划的法兰预测范围已改变；请重新规划")
            trace = (trace_factory or ReplayCanTrace)(config["channel"], trace_path)
            trace.start()
            previous = (signal.signal(signal.SIGINT, request_stop),
                        signal.signal(signal.SIGTERM, request_stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消，未发送运动指令")
                print(f"{remaining} 秒后逐点回放；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            require_ready(read_state(robot))
            if max(abs(a-b) for a,b in zip(robot.get_joint_angles().msg, route[0])) > 0.002:
                raise RuntimeError("倒计时期间起点改变")
            started = time.monotonic()

            def on_feedback(joints):
                report["feedback_samples"].append({
                    "elapsed_s": round(time.monotonic()-started, 3),
                    "joint_rad": list(joints),
                    "last_attempted_step": report["last_attempted_step"]})

            def on_command(index, target):
                report["last_attempted_step"] = index
                report["last_target_joint_rad"] = list(target)
                report["command_events"].append({"step": index,
                    "at_utc": datetime.now(timezone.utc).isoformat(),
                    "target_joint_rad": list(target)})

            execute_route(robot, config, route, lambda: interrupted,
                          on_waypoint=lambda index,_: report.update(last_reached_step=index),
                          on_command=on_command,
                          on_command_published=lambda index,_: report.update(last_published_step=index),
                          on_feedback=on_feedback,
                          on_status=lambda status: report["status_samples"].append({
                              "elapsed_s": round(time.monotonic()-started, 3),
                              "arm_status": int(status.msg.arm_status),
                              "motion_status": int(status.msg.motion_status),
                              "error_code": int(status.msg.err_code)}),
                          target_tolerance_rad=REPLAY_TARGET_TOLERANCE_RAD,
                          speed_percent=plan["speed_percent"],
                          waypoint_timeout_s=float(config["return_waypoint_timeout_s"]),
                          waypoint_max_timeout_s=REPLAY_MAX_WAYPOINT_SECONDS,
                          pause_on_stationary_timeout=True,
                          stationary_retry_limit=1)
            final, error = verify_return(robot, snapshot["joint_rad"])
            final_state = require_ready(read_state(robot))
            if error > 0.005 or max(abs(a-b) for a,b in zip(
                    final_state["joint_rad"], snapshot["joint_rad"])) > 0.005:
                raise RuntimeError("最终关节误差超过 0.005 rad")
            report.update(final_joint_rad=final_state["joint_rad"],
                          final_flange_pose_m_rad=final_state["flange_pose_m_rad"],
                          final_max_joint_error_rad=error,
                          completed=True)
    except Exception as exc:
        report["error"] = str(exc)
        report["error_type"] = type(exc).__name__
        raise RuntimeError(f"快速回放未完成：{exc}；报告 {report_path}") from exc
    finally:
        if trace is not None:
            try:
                trace.stop()
                report["can_trace_frames"] = trace.count
            except Exception as exc:
                report["trace_error"] = str(exc)
                report["completed"] = False
                report["error"] = "CAN 轨迹日志关闭失败；本轮不能作为完整验收证据"
        if previous is not None:
            signal.signal(signal.SIGINT, previous[0])
            signal.signal(signal.SIGTERM, previous[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
    if report.get("trace_error"):
        raise RuntimeError(f"{report['error']}；报告 {report_path}")
    return report_path
