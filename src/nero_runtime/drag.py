"""可注入的 Drag 模式切换证据；无 SDK、现场许可、录制或运动入口。

真实 Nero V121 的退出请求尚未适配：其 set_normal_mode() 是 no-op。
调用方须显式提供经核实的请求和反馈判据，不能将合成测试当真机授权。
"""

from dataclasses import dataclass
import math
from typing import Callable

from .acquisition import AcquisitionResult
from .snapshot import RobotSnapshot


@dataclass(frozen=True)
class DragInspection:
    snapshot: RobotSnapshot
    state: str  # active / inactive / unknown；由调用方反馈判据给出
    joint_rad: tuple[float, ...] | None
    controller_fault: bool | None


@dataclass(frozen=True)
class DragResult:
    action: str
    outcome: str  # confirmed / unconfirmed / fault / cancelled / rejected
    reason: str | None
    enter_request_attempted: bool
    enter_requested: bool
    enter_confirmed: bool
    exit_request_attempted: bool
    exit_requested: bool
    exit_confirmed: bool
    start_joint_rad: tuple[float, ...] | None
    latest_joint_rad: tuple[float, ...] | None
    max_joint_delta_rad: float | None
    transition_joint_delta_limit_rad: float | None
    transition_limit_source: str | None
    observations: tuple[DragInspection, ...]
    request_error: str | None = None
    rollback_error: str | None = None


def _finite_positive(value, name: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be finite and positive")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be finite and positive") from exc
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return number


def _clock(monotonic_s, previous=None) -> float:
    value = monotonic_s()
    if type(value) not in (int, float):
        raise ValueError("monotonic clock must be finite and non-negative")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError("monotonic clock must be finite and non-negative") from exc
    if not math.isfinite(value) or value < 0:
        raise ValueError("monotonic clock must be finite and non-negative")
    if previous is not None and value < previous:
        raise ValueError("monotonic clock moved backwards")
    return value


def _read(read_snapshot) -> RobotSnapshot:
    result = read_snapshot()
    if type(result) is not AcquisitionResult or type(result.snapshot) is not RobotSnapshot:
        raise TypeError("read_snapshot must return F1 AcquisitionResult")
    return result.snapshot


def _inspect(snapshot: RobotSnapshot, predicate) -> DragInspection:
    state = "unknown"
    if snapshot.status_quality.state == "valid":
        active = predicate(snapshot)
        if active is True:
            state = "active"
        elif active is False:
            state = "inactive"
        elif active is not None:
            raise TypeError("drag_active_predicate must return bool or None")
    joint = snapshot.joint_rad if snapshot.joints_quality.state == "valid" else None
    fault = None if snapshot.status_quality.state != "valid" else (
        snapshot.error_code != 0 or snapshot.arm_status not in (0, 11))
    return DragInspection(snapshot, state, joint, fault)


def inspect_drag_state(*, read_snapshot: Callable[[], AcquisitionResult],
                       drag_active_predicate: Callable[[RobotSnapshot], bool | None]) -> DragInspection:
    """只读一轮 F1 反馈；不从 teach_status/control_mode 猜测零力模式。"""
    if not callable(read_snapshot) or not callable(drag_active_predicate):
        raise TypeError("read_snapshot and drag_active_predicate callables are required")
    return _inspect(_read(read_snapshot), drag_active_predicate)


def _drivers_problem(snapshot: RobotSnapshot) -> str | None:
    if len(snapshot.drivers) != 7 or {driver.joint for driver in snapshot.drivers} != set(range(1, 8)):
        return "seven driver identities are required"
    for driver in snapshot.drivers:
        if driver.quality.state != "valid":
            return f"J{driver.joint} driver feedback {driver.quality.state}"
        if driver.enabled is not True or driver.undervoltage is not False or driver.driver_error is not False:
            return f"J{driver.joint} disabled, undervoltage or driver fault"
    return None


def _preflight(inspection: DragInspection) -> str | None:
    snapshot = inspection.snapshot
    if snapshot.status_quality.state != "valid":
        return f"status feedback {snapshot.status_quality.state}"
    if snapshot.joints_quality.state != "valid" or inspection.joint_rad is None:
        return f"joint feedback {snapshot.joints_quality.state}"
    if inspection.state != "inactive":
        return f"drag state {inspection.state}; repeated enter is refused"
    if snapshot.error_code != 0 or snapshot.arm_status != 0:
        return f"controller fault/status {snapshot.error_code}/{snapshot.arm_status}"
    # SDK enum: 1 = CAN control; 2 = teaching, 3.. = other control owners.
    if snapshot.control_mode != 1 or snapshot.teach_status not in (0, 2, 6):
        return f"conflicting controller/teaching mode {snapshot.control_mode}/{snapshot.teach_status}"
    return _drivers_problem(snapshot)


def _newer(inspection: DragInspection, baseline: RobotSnapshot) -> bool:
    timestamp = inspection.snapshot.status_quality.timestamp_s
    previous = baseline.status_quality.timestamp_s
    return (inspection.snapshot.status_quality.state == "valid" and
            timestamp is not None and previous is not None and timestamp > previous)


def _result(action, outcome, reason, observations, *, enter_attempted=False,
            enter_requested=False, enter_confirmed=False, exit_attempted=False,
            exit_requested=False, exit_confirmed=False, limit=None, limit_source=None,
            request_error=None, rollback_error=None):
    start = observations[0].joint_rad if observations else None
    latest = next((item.joint_rad for item in reversed(observations)
                   if item.joint_rad is not None), None)
    deltas = [max(abs(a - b) for a, b in zip(start, item.joint_rad))
              for item in observations if start is not None and item.joint_rad is not None]
    return DragResult(action, outcome, reason, enter_attempted, enter_requested,
                      enter_confirmed, exit_attempted, exit_requested, exit_confirmed,
                      start, latest, max(deltas) if deltas else None,
                      limit, limit_source, tuple(observations), request_error, rollback_error)


def _validate_common(read_snapshot, predicate, monotonic_s, sleep_s,
                     cancel_requested, timeout_s, poll_interval_s):
    for name, value in (("read_snapshot", read_snapshot),
                        ("drag_active_predicate", predicate),
                        ("monotonic_s", monotonic_s), ("sleep_s", sleep_s),
                        ("cancel_requested", cancel_requested)):
        if not callable(value):
            raise TypeError(f"{name} callable is required")
    timeout = _finite_positive(timeout_s, "timeout_s")
    poll = _finite_positive(poll_interval_s, "poll_interval_s")
    return timeout, poll


def exit_drag(*, read_snapshot, request_exit, drag_active_predicate,
              monotonic_s, sleep_s, cancel_requested, timeout_s,
              poll_interval_s, force_request=False) -> DragResult:
    """显式退出请求；仅新鲜反馈确认 inactive 才报告 confirmed。"""
    timeout, poll = _validate_common(read_snapshot, drag_active_predicate,
                                     monotonic_s, sleep_s, cancel_requested,
                                     timeout_s, poll_interval_s)
    if not callable(request_exit):
        raise TypeError("request_exit callable is required")
    if type(force_request) is not bool:
        raise TypeError("force_request must be bool")
    observations = []
    try:
        before = inspect_drag_state(read_snapshot=read_snapshot,
                                    drag_active_predicate=drag_active_predicate)
        observations.append(before)
    except KeyboardInterrupt:
        # 显式退出可能发生在取消过程中；只读采集中断不能跳过退出尝试。
        before = None
        read_error = "initial read KeyboardInterrupt"
    except Exception as exc:
        # 反馈缺失不能阻止调用方显式请求退出模式。
        before = None
        read_error = f"initial read {type(exc).__name__}: {exc}"
    else:
        read_error = None
        if before.state == "inactive" and not force_request:
            return _result("exit", "confirmed", "fresh feedback already inactive; no request", observations,
                           exit_confirmed=True)
    attempted = True
    try:
        request_exit()
    except KeyboardInterrupt:
        return _result("exit", "cancelled", "exit request interrupted; state unknown",
                       observations, exit_attempted=attempted,
                       request_error="KeyboardInterrupt during exit request")
    except Exception as exc:
        return _result("exit", "fault", "exit request failed; state unknown", observations,
                       exit_attempted=attempted,
                       request_error=f"{type(exc).__name__}: {exc}")
    requested = True
    try:
        start = _clock(monotonic_s)
    except Exception as exc:
        return _result("exit", "fault", f"exit clock error: {type(exc).__name__}: {exc}",
                       observations, exit_attempted=attempted, exit_requested=requested)
    deadline = start + timeout
    last = start
    while True:
        try:
            now = _clock(monotonic_s, last)
            last = now
            if cancel_requested():
                return _result("exit", "cancelled", "cancelled while confirming exit; state unknown",
                               observations, exit_attempted=attempted, exit_requested=requested)
            inspection = inspect_drag_state(read_snapshot=read_snapshot,
                                            drag_active_predicate=drag_active_predicate)
            observations.append(inspection)
            if (inspection.state == "inactive" and before is not None and
                    _newer(inspection, before.snapshot)):
                return _result("exit", "confirmed", None, observations,
                               exit_attempted=attempted, exit_requested=requested,
                               exit_confirmed=True)
            if inspection.controller_fault is True:
                return _result("exit", "fault", "controller fault during exit confirmation",
                               observations, exit_attempted=attempted, exit_requested=requested)
            now = _clock(monotonic_s, last)
            last = now
            if now >= deadline:
                reason = "exit feedback unconfirmed"
                if read_error:
                    reason += f"; {read_error}"
                return _result("exit", "unconfirmed", reason, observations,
                               exit_attempted=attempted, exit_requested=requested)
            sleep_s(min(poll, deadline - now))
        except KeyboardInterrupt:
            return _result("exit", "cancelled", "cancelled while confirming exit; state unknown",
                           observations, exit_attempted=attempted, exit_requested=requested)
        except Exception as exc:
            return _result("exit", "fault", f"exit feedback error: {type(exc).__name__}: {exc}",
                           observations, exit_attempted=attempted, exit_requested=requested)


def enter_drag(*, read_snapshot, request_enter, request_exit,
               drag_active_predicate, monotonic_s, sleep_s, cancel_requested,
               timeout_s, poll_interval_s, transition_joint_delta_limit_rad=None,
               transition_limit_source=None) -> DragResult:
    """设备事实合格后请求一次进入；失败/取消时显式尝试一次退出回退。"""
    timeout, poll = _validate_common(read_snapshot, drag_active_predicate,
                                     monotonic_s, sleep_s, cancel_requested,
                                     timeout_s, poll_interval_s)
    for name, value in (("request_enter", request_enter), ("request_exit", request_exit)):
        if not callable(value):
            raise TypeError(f"{name} callable is required")
    if transition_joint_delta_limit_rad is None:
        if transition_limit_source is not None:
            raise ValueError("transition limit source requires a limit")
        limit = None
    else:
        limit = _finite_positive(transition_joint_delta_limit_rad,
                                 "transition_joint_delta_limit_rad")
        if type(transition_limit_source) is not str or not transition_limit_source.strip():
            raise ValueError("transition limit source must be explicit")
    observations = []
    try:
        first = inspect_drag_state(read_snapshot=read_snapshot,
                                   drag_active_predicate=drag_active_predicate)
        observations.append(first)
        problem = _preflight(first)
        if problem is not None:
            return _result("enter", "rejected", problem, observations,
                           limit=limit, limit_source=transition_limit_source)
        if cancel_requested():
            return _result("enter", "cancelled", "cancelled before enter request", observations,
                           limit=limit, limit_source=transition_limit_source)
    except KeyboardInterrupt:
        return _result("enter", "cancelled", "cancelled before enter request", observations,
                       limit=limit, limit_source=transition_limit_source)
    except Exception as exc:
        return _result("enter", "rejected", f"preflight error: {type(exc).__name__}: {exc}",
                       observations, limit=limit, limit_source=transition_limit_source)

    attempted = True
    requested = False
    reason = None
    outcome = "fault"
    request_error = None
    try:
        request_enter()
        requested = True
        start = _clock(monotonic_s)
        deadline = start + timeout
        last = start
        while True:
            now = _clock(monotonic_s, last)
            last = now
            if cancel_requested():
                outcome, reason = "cancelled", "cancelled during enter confirmation"
                break
            inspection = inspect_drag_state(read_snapshot=read_snapshot,
                                            drag_active_predicate=drag_active_predicate)
            observations.append(inspection)
            if first.joint_rad is not None and inspection.joint_rad is not None:
                delta = max(abs(a - b) for a, b in zip(first.joint_rad, inspection.joint_rad))
                if limit is not None and delta > limit:
                    reason = f"transition joint delta {delta:.6f} rad exceeds explicit {limit:.6f} rad"
                    break
            if inspection.controller_fault is True or _drivers_problem(inspection.snapshot) is not None:
                reason = "controller or driver fault during enter confirmation"
                break
            if (inspection.state == "active" and inspection.joint_rad is not None and
                    _newer(inspection, first.snapshot)):
                return _result("enter", "confirmed", None, observations,
                               enter_attempted=attempted, enter_requested=requested,
                               enter_confirmed=True, limit=limit,
                               limit_source=transition_limit_source)
            now = _clock(monotonic_s, last)
            last = now
            if now >= deadline:
                reason = "enter feedback unconfirmed"
                break
            sleep_s(min(poll, deadline - now))
    except KeyboardInterrupt:
        outcome, reason = "cancelled", "cancelled during enter request/confirmation"
        request_error = "KeyboardInterrupt during enter request/confirmation"
    except Exception as exc:
        reason = f"enter request/feedback error: {type(exc).__name__}: {exc}"
        request_error = f"{type(exc).__name__}: {exc}"

    # 请求可能部分发送；一次显式模式退出尝试，由 exit_drag 负责实际反馈验收。
    rollback = exit_drag(read_snapshot=read_snapshot, request_exit=request_exit,
                         drag_active_predicate=drag_active_predicate,
                         monotonic_s=monotonic_s, sleep_s=sleep_s,
                         cancel_requested=lambda: False,
                         timeout_s=timeout, poll_interval_s=poll,
                         force_request=True)
    observations.extend(rollback.observations)
    return _result("enter", outcome, reason, observations,
                   enter_attempted=attempted, enter_requested=requested,
                   exit_attempted=rollback.exit_request_attempted,
                   exit_requested=rollback.exit_requested,
                   exit_confirmed=rollback.exit_confirmed,
                   limit=limit, limit_source=transition_limit_source,
                   request_error=request_error,
                   rollback_error=(rollback.request_error or rollback.reason)
                   if rollback.outcome != "confirmed" else None)
