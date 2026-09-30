"""调用者指定的七轴关节路线：离线准备、逐点发布与反馈验收。

本模块不连接设备、不选择路线或停止手段，也不授予现场运动许可。
发布、实时反馈、操作前置检查、取消及停止均由受信上层显式提供。
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable

from .snapshot import RobotSnapshot
from .waypoints import load_waypoint


def _angles(values) -> tuple[float, ...]:
    if type(values) not in (list, tuple) or len(values) != 7:
        raise ValueError("joint point requires J1..J7 in rad")
    if any(type(value) not in (int, float) for value in values):
        raise ValueError("joint point contains a non-numeric angle")
    try:
        result = tuple(float(value) for value in values)
    except OverflowError as exc:
        raise ValueError("joint point contains an overflowing angle") from exc
    if not all(math.isfinite(value) for value in result):
        raise ValueError("joint point contains a non-finite angle")
    return result


@dataclass(frozen=True)
class RoutePoint:
    joint_rad: tuple[float, ...]
    name: str | None
    source: str
    robot_id: str | None = None
    tool_id: str | None = None
    flange_frame: str | None = None

    def __post_init__(self):
        object.__setattr__(self, "joint_rad", _angles(self.joint_rad))
        if type(self.source) is not str or not self.source.strip():
            raise ValueError("route point source is required")
        for field in ("name", "robot_id", "tool_id", "flange_frame"):
            value = getattr(self, field)
            if value is not None and (type(value) is not str or not value.strip()):
                raise ValueError(f"route point {field} is invalid")


@dataclass(frozen=True)
class JointRoute:
    expected_start: RoutePoint
    targets: tuple[RoutePoint, ...]

    def __post_init__(self):
        if type(self.expected_start) is not RoutePoint or type(self.targets) not in (tuple, list):
            raise ValueError("route requires one expected start and ordered targets")
        object.__setattr__(self, "targets", tuple(self.targets))
        if not self.targets or any(type(point) is not RoutePoint for point in self.targets):
            raise ValueError("route requires at least one valid target")
        for field in ("robot_id", "tool_id", "flange_frame"):
            known = {getattr(point, field) for point in
                     (self.expected_start, *self.targets) if getattr(point, field) is not None}
            if len(known) > 1:
                raise ValueError(f"route points disagree on {field}")


def prepare_joint_route(expected_start, targets, *, raw_unit: str | None = None) -> JointRoute:
    """从 F1 JSON 路径或显式 J1..J7 rad 复制完整有序路线。"""
    if raw_unit is not None and raw_unit != "rad":
        raise ValueError("raw joint unit must be rad")
    if type(targets) not in (tuple, list):
        raise ValueError("targets must be an ordered list or tuple")

    def point(value):
        if isinstance(value, Path):
            record = load_waypoint(value)
            if record["joint_unit"] != "rad":
                raise ValueError("waypoint joint unit must be rad")
            return RoutePoint(tuple(record["snapshot"]["joint_rad"]),
                              record["name"], f"{value}: {record['source']}",
                              record["robot_id"], record["tool_id"],
                              record["flange_frame"])
        if type(value) in (tuple, list):
            if raw_unit != "rad":
                raise ValueError("explicit joint points require raw_unit='rad'")
            return RoutePoint(tuple(value), None, "explicit J1..J7 rad")
        raise ValueError("route point must be a waypoint Path or J1..J7 list")

    # 构造前读完整条路线；末点无效时调用方尚未获得可执行对象。
    return JointRoute(point(expected_start), tuple(point(value) for value in targets))


@dataclass(frozen=True)
class RouteParameters:
    start_tolerance_rad: float
    target_tolerance_rad: float
    waypoint_timeout_s: float
    poll_interval_s: float
    settle_window_s: float
    settle_delta_rad: float

    def __post_init__(self):
        for field in ("start_tolerance_rad", "target_tolerance_rad",
                      "waypoint_timeout_s", "poll_interval_s", "settle_window_s",
                      "settle_delta_rad"):
            value = getattr(self, field)
            try:
                valid = (type(value) in (int, float) and
                         math.isfinite(float(value)) and value > 0)
            except OverflowError:
                valid = False
            if not valid:
                raise ValueError(f"{field} must be finite and positive")


@dataclass(frozen=True)
class StopEvidence:
    requested: bool
    feedback_confirmed: bool
    basis: str

    def __post_init__(self):
        if type(self.requested) is not bool or type(self.feedback_confirmed) is not bool:
            raise ValueError("stop evidence flags must be boolean")
        if type(self.basis) is not str:
            raise ValueError("stop evidence basis must be text")
        if self.feedback_confirmed and (not self.requested or not self.basis.strip()):
            raise ValueError("stop confirmation requires a request and feedback basis")


@dataclass(frozen=True)
class MotionResult:
    status: str
    reason: str | None
    attempted: int
    published: int
    reached: int
    last_target_index: int | None
    last_feedback: RobotSnapshot | None
    stop_request_attempted: bool
    stop_requested: bool
    stop_feedback_confirmed: bool
    stop_detail: str | None


def _feedback_problem(snapshot: RobotSnapshot, route: JointRoute,
                      time_basis: str | None = None) -> str | None:
    if type(snapshot) is not RobotSnapshot:
        raise TypeError("read_feedback must return RobotSnapshot")
    if time_basis is not None and snapshot.time_basis != time_basis:
        return "feedback time basis changed"
    known_ids = {point.robot_id for point in (route.expected_start, *route.targets)
                 if point.robot_id is not None}
    if known_ids and snapshot.robot_id not in known_ids:
        return "robot identity missing or mismatched"
    if snapshot.status_quality.state != "valid":
        return f"controller feedback {snapshot.status_quality.state}"
    if snapshot.joints_quality.state != "valid" or snapshot.joint_rad is None:
        return f"joint feedback {snapshot.joints_quality.state}"
    if snapshot.error_code != 0:
        return f"controller error code {snapshot.error_code}"
    for driver in snapshot.drivers:
        if (driver.quality.state != "valid" or driver.enabled is not True or
                driver.undervoltage is not False or driver.driver_error is not False):
            return f"J{driver.joint} driver feedback missing, disabled or faulted"
    return None


def execute_joint_route(
    route: JointRoute, *, parameters: RouteParameters,
    preflight_check: Callable[[], None], read_feedback: Callable[[], RobotSnapshot],
    publish_target: Callable[[list[float]], None],
    cancel_requested: Callable[[], bool],
    stop_motion: Callable[[str], StopEvidence],
    monotonic_s: Callable[[], float], sleep_s: Callable[[float], None],
) -> MotionResult:
    """在显式准备好的来源上按序逐点发布；每点需两次更新的稳定反馈。

    调用方负责连接、使能、模式/速度、环境及路线适用性、现场授权和停止映射。
    本函数不把 F1 组合时间戳当成真实设备已完成逐轴刷新验证。
    """
    if type(route) is not JointRoute or type(parameters) is not RouteParameters:
        raise TypeError("prepared JointRoute and RouteParameters are required")
    for name, value in (("preflight_check", preflight_check),
                        ("read_feedback", read_feedback),
                        ("publish_target", publish_target),
                        ("cancel_requested", cancel_requested),
                        ("stop_motion", stop_motion),
                        ("monotonic_s", monotonic_s), ("sleep_s", sleep_s)):
        if not callable(value):
            raise TypeError(f"{name} callable is required before motion")

    attempted = published = reached = 0
    last_index = None
    last_feedback = None

    def finish(status: str, reason: str | None) -> MotionResult:
        stop_attempted = attempted > 0 and status != "completed"
        stop_requested = stop_confirmed = False
        stop_detail = None
        if stop_attempted:
            try:
                evidence = stop_motion(reason or status)
                if type(evidence) is not StopEvidence:
                    raise TypeError("stop_motion must return StopEvidence")
                stop_requested = evidence.requested
                stop_confirmed = evidence.feedback_confirmed
                stop_detail = evidence.basis
            except BaseException as exc:
                stop_detail = f"stop handling failed: {type(exc).__name__}: {exc}"
        return MotionResult(status, reason, attempted, published, reached,
                            last_index, last_feedback, stop_attempted,
                            stop_requested, stop_confirmed, stop_detail)

    if cancel_requested():
        return finish("cancelled", "cancelled before any control attempt")
    if preflight_check() is not None:
        raise TypeError("preflight_check must return None or raise")
    first = read_feedback()
    last_feedback = first
    problem = _feedback_problem(first, route)
    if problem:
        return finish("fault", problem)
    if max(abs(a - b) for a, b in zip(first.joint_rad,
                                     route.expected_start.joint_rad)) > parameters.start_tolerance_rad:
        return finish("fault", "current joints differ from expected_start")

    baseline_stamp = first.joints_quality.timestamp_s
    basis = first.time_basis
    try:
        for index, target in enumerate(route.targets, 1):
            if cancel_requested():
                return finish("cancelled", "cancelled before next target")
            # SDK 调用一开始即可能部分发送；返回只证明本地调用完成。
            attempted += 1
            last_index = index
            publish_target(list(target.joint_rad))
            published += 1
            deadline = monotonic_s() + parameters.waypoint_timeout_s
            first_near_stamp = None
            previous_near = None
            while True:
                if cancel_requested():
                    return finish("cancelled", f"cancelled at target {index}")
                if monotonic_s() >= deadline:
                    return finish("fault", f"target {index}: feedback timeout")
                snapshot = read_feedback()
                last_feedback = snapshot
                if monotonic_s() > deadline:
                    return finish("fault", f"target {index}: feedback timeout")
                problem = _feedback_problem(snapshot, route, basis)
                if problem:
                    return finish("fault", f"target {index}: {problem}")
                stamp = snapshot.joints_quality.timestamp_s
                if stamp < baseline_stamp:
                    return finish("fault", f"target {index}: joint timestamp regressed")
                if stamp > baseline_stamp:
                    baseline_stamp = stamp
                    error = max(abs(a - b) for a, b in zip(snapshot.joint_rad,
                                                            target.joint_rad))
                    if error <= parameters.target_tolerance_rad:
                        if first_near_stamp is None or previous_near is None or max(
                                abs(a - b) for a, b in zip(snapshot.joint_rad,
                                                            previous_near)) > parameters.settle_delta_rad:
                            first_near_stamp = stamp
                        elif stamp - first_near_stamp >= parameters.settle_window_s:
                            reached += 1
                            break
                        previous_near = snapshot.joint_rad
                    else:
                        first_near_stamp = previous_near = None
                remaining = deadline - monotonic_s()
                if remaining <= 0:
                    return finish("fault", f"target {index}: feedback timeout")
                sleep_s(min(parameters.poll_interval_s, remaining))
    except KeyboardInterrupt:
        return finish("cancelled", "KeyboardInterrupt during motion")
    except (OSError, RuntimeError, ValueError) as exc:
        return finish("fault", f"{type(exc).__name__}: {exc}")
    except BaseException:
        # 未预期的编程错误仍暴露原异常；先尝试显式停止处理。
        finish("fault", "unexpected execution error")
        raise
    return finish("completed", None)
