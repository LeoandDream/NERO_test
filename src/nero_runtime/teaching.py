"""Guided Recording Core：只组合 F4 Drag 与 F3 Recording，不包含 Return READY。

调用方负责站点/人工许可、已核实的模式请求和反馈判据；本模块不连接
SDK，不选路线，不发运动、使能、失能或急停，也不提供普通 Teach 入口。
"""

from dataclasses import dataclass
import math
from pathlib import Path

from .acquisition import AcquisitionResult
from .drag import DragResult, enter_drag, exit_drag, inspect_drag_state
from .recording import RecordingParameters, record_trajectory, recording_paths
from .snapshot import RobotSnapshot


@dataclass(frozen=True)
class GuidedRecordingResult:
    workflow_outcome: str  # completed / cancelled / fault / unconfirmed_exit / rejected
    phases: tuple[str, ...]
    reason: str | None
    enter_result: DragResult | None
    recording_summary: dict | None
    exit_result: DragResult | None
    recording_id: str
    raw_path: Path | None
    summary_path: Path | None
    final_snapshot: RobotSnapshot | None
    final_state: str  # known / unknown
    final_joint_rad: tuple[float, ...] | None
    final_flange_pose: tuple[float, ...] | None
    return_attempted: bool = False
    motion_attempted: bool = False


class _GuidedRecordingFault(RuntimeError):
    """在该轮样本写入之后停止 F3；保留造成拒绝的原始快照。"""


def _positive(value, name: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be finite and positive")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite and positive") from exc
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return number


def _session_problem(result: AcquisitionResult, predicate) -> str | None:
    inspection = inspect_drag_state(read_snapshot=lambda: result,
                                    drag_active_predicate=predicate)
    if inspection.state != "active":
        return f"Drag feedback {inspection.state} during recording"
    if inspection.controller_fault is True:
        return f"controller fault/status {result.snapshot.error_code}/{result.snapshot.arm_status}"
    drivers = result.snapshot.drivers
    if len(drivers) != 7 or {driver.joint for driver in drivers} != set(range(1, 8)):
        return "driver identities incomplete during recording"
    for driver in drivers:
        if (driver.quality.state != "valid" or driver.enabled is not True or
                driver.undervoltage is not False or driver.driver_error is not False):
            return f"J{driver.joint} driver feedback invalid, disabled or faulted"
    return None


def _final_observation(read_snapshot, sleep_s, poll_s, exit_result):
    """退出确认后只取一轮新反馈；来源不新或不全时不虚构终点。"""
    try:
        sleep_s(poll_s)
        result = read_snapshot()
        if type(result) is not AcquisitionResult or type(result.snapshot) is not RobotSnapshot:
            raise TypeError("final read must return F1 AcquisitionResult")
        snapshot = result.snapshot
        previous = exit_result.observations[-1].snapshot
        sources = (("status_quality",), ("joints_quality",), ("flange_quality",))
        fresh = all(
            getattr(snapshot, name).state == "valid" and
            getattr(previous, name).timestamp_s is not None and
            getattr(snapshot, name).timestamp_s is not None and
            getattr(snapshot, name).timestamp_s > getattr(previous, name).timestamp_s
            for (name,) in sources
        )
        if (not fresh or snapshot.robot_id != previous.robot_id or
                snapshot.time_basis != previous.time_basis or
                snapshot.joint_rad is None or snapshot.flange_pose_m_rad is None):
            return snapshot, "unknown", None, None, "final feedback missing, stale or not newer"
        return snapshot, "known", snapshot.joint_rad, snapshot.flange_pose_m_rad, None
    except BaseException as exc:
        return None, "unknown", None, None, f"final read {type(exc).__name__}: {exc}"


def run_guided_recording(
    *, recording_id: str, output_directory: str | Path,
    parameters: RecordingParameters, read_snapshot, request_enter_drag,
    request_exit_drag, drag_active_predicate, monotonic_s, sleep_s,
    cancel_requested, finish_requested, wall_clock_s,
    enter_timeout_s, exit_timeout_s, poll_interval_s,
    transition_joint_delta_limit_rad=None, transition_limit_source=None,
) -> GuidedRecordingResult:
    """先核文件，再真实组合 F4 enter → F3 record → F4 exit → F1 终点。

    `completed` 只代表 Guided Recording 三段正常；绝不代表 Teach v1 完成。
    """
    phases = ["before_enter"]
    if type(parameters) is not RecordingParameters:
        raise TypeError("RecordingParameters are required")
    for name, value in (("read_snapshot", read_snapshot),
                        ("request_enter_drag", request_enter_drag),
                        ("request_exit_drag", request_exit_drag),
                        ("drag_active_predicate", drag_active_predicate),
                        ("monotonic_s", monotonic_s), ("sleep_s", sleep_s),
                        ("cancel_requested", cancel_requested),
                        ("finish_requested", finish_requested),
                        ("wall_clock_s", wall_clock_s)):
        if not callable(value):
            raise TypeError(f"{name} callable is required before Drag")
    enter_timeout = _positive(enter_timeout_s, "enter_timeout_s")
    exit_timeout = _positive(exit_timeout_s, "exit_timeout_s")
    poll = _positive(poll_interval_s, "poll_interval_s")
    if transition_joint_delta_limit_rad is not None:
        _positive(transition_joint_delta_limit_rad, "transition_joint_delta_limit_rad")
        if type(transition_limit_source) is not str or not transition_limit_source.strip():
            raise ValueError("transition limit source must be explicit")
    elif transition_limit_source is not None:
        raise ValueError("transition limit source requires a limit")

    try:
        raw_path, summary_path = recording_paths(output_directory, recording_id)
    except (ValueError, FileExistsError) as exc:
        phases.append("finished")
        return GuidedRecordingResult("rejected", tuple(phases), f"recording preflight: {exc}",
                                     None, None, None, recording_id, None, None,
                                     None, "unknown", None, None)

    enter = enter_drag(
        read_snapshot=read_snapshot, request_enter=request_enter_drag,
        request_exit=request_exit_drag, drag_active_predicate=drag_active_predicate,
        monotonic_s=monotonic_s, sleep_s=sleep_s,
        cancel_requested=cancel_requested, timeout_s=enter_timeout,
        poll_interval_s=poll,
        transition_joint_delta_limit_rad=transition_joint_delta_limit_rad,
        transition_limit_source=transition_limit_source)
    if enter.outcome != "confirmed" or not enter.enter_confirmed:
        # F4 已负责进入请求后的单次回退；本层不能再发送第二次。
        phases.append("finished")
        outcome = enter.outcome if enter.outcome in ("rejected", "cancelled") else "fault"
        return GuidedRecordingResult(outcome, tuple(phases), enter.reason,
                                     enter, None, None, recording_id,
                                     raw_path, summary_path, None, "unknown", None, None)

    phases.extend(("drag_entered", "recording"))
    pending_problem = None

    def monitored_read():
        nonlocal pending_problem
        result = read_snapshot()
        problem = _session_problem(result, drag_active_predicate)
        if problem is not None:
            pending_problem = problem
        return result  # F3 先保留该轮真实快照，再由取消回调抛故障。

    def monitored_cancel():
        if pending_problem is not None:
            raise _GuidedRecordingFault(pending_problem)
        return cancel_requested()

    summary = None
    recording_error = None
    try:
        summary = record_trajectory(
            recording_id=recording_id, output_directory=output_directory,
            parameters=parameters, read_snapshot=monitored_read,
            monotonic_s=monotonic_s, sleep_s=sleep_s,
            cancel_requested=monitored_cancel, finish_requested=finish_requested,
            wall_clock_s=wall_clock_s)
    except BaseException as exc:
        # 打开文件/摘要失败可能发生在 F3 的采样循环外；仍必须退出 Drag。
        recording_error = f"{type(exc).__name__}: {exc}"
    phases.append("exiting_drag")
    try:
        exit_result = exit_drag(
            read_snapshot=read_snapshot, request_exit=request_exit_drag,
            drag_active_predicate=drag_active_predicate,
            monotonic_s=monotonic_s, sleep_s=sleep_s,
            cancel_requested=lambda: False, timeout_s=exit_timeout,
            poll_interval_s=poll, force_request=True)
        exit_error = None
    except BaseException as exc:
        exit_result = None
        exit_error = f"exit invocation {type(exc).__name__}: {exc}"

    final_snapshot = final_joint = final_flange = None
    final_state = "unknown"
    final_error = None
    if exit_result is not None and exit_result.exit_confirmed:
        final_snapshot, final_state, final_joint, final_flange, final_error = _final_observation(
            read_snapshot, sleep_s, poll, exit_result)
    phases.append("finished")

    if recording_error is not None:
        outcome = "cancelled" if recording_error.startswith("KeyboardInterrupt:") else "fault"
    elif summary is None:
        outcome = "fault"
        recording_error = "recording summary unavailable"
    elif summary["stop_reason"] == "cancelled":
        outcome = "cancelled"
    elif summary["stop_reason"] == "fault":
        outcome = "fault"
    elif summary["complete"] and summary["stop_reason"] in ("duration_complete", "user_finish"):
        outcome = "completed" if exit_result is not None and exit_result.exit_confirmed else "unconfirmed_exit"
    else:
        outcome = "fault"

    reasons = [item for item in (recording_error,
                                  summary.get("error") if summary else None,
                                  exit_error,
                                  exit_result.reason if exit_result and not exit_result.exit_confirmed else None,
                                  exit_result.request_error if exit_result and not exit_result.exit_confirmed else None,
                                  final_error) if item]
    return GuidedRecordingResult(
        outcome, tuple(phases), "; ".join(reasons) or None,
        enter, summary, exit_result, recording_id,
        raw_path, summary_path, final_snapshot, final_state,
        final_joint, final_flange)
