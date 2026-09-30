"""F7 命名 Start/Park/Return READY：选择环境资产，执行只委托 F2。"""

from dataclasses import dataclass
import math
from pathlib import Path

from .environment import EnvironmentProfile, load_environment_profile
from .motion import JointRoute, MotionResult, RouteParameters, execute_joint_route, prepare_joint_route
from .snapshot import RobotSnapshot
from .teaching import GuidedRecordingResult


@dataclass(frozen=True)
class EnvironmentIdentity:
    environment_id: str | None
    environment_version: str | None
    mount: str | None
    tool_id: str | None


@dataclass(frozen=True)
class OperationPlan:
    operation: str
    environment_id: str
    environment_version: str
    from_point: str | None
    to_point: str | None
    route_id: str | None
    route: JointRoute | None
    route_source_hashes: tuple[tuple[str, str], ...]
    evidence_ref: str | None
    requires_enable: bool
    executable_under_profile: bool
    reasons: tuple[str, ...]
    profile_path: Path
    profile_sha256: str
    identity: EnvironmentIdentity
    prepared_snapshot: RobotSnapshot
    match_tolerance_rad: float


@dataclass(frozen=True)
class OperationResult:
    status: str
    plan: OperationPlan
    motion_result: MotionResult | None
    no_motion: bool
    physical_support_confirmed: bool = False
    disable_permission: bool = False


@dataclass(frozen=True)
class TeachReturnPlan:
    guided_result: GuidedRecordingResult
    return_plan: OperationPlan | None
    reason: str | None


@dataclass(frozen=True)
class TeachV1Result:
    guided_result: GuidedRecordingResult
    return_result: OperationResult | None
    teach_v1_completed: bool
    reason: str | None


def _profile(profile_path: str | Path) -> EnvironmentProfile:
    return load_environment_profile(profile_path)


def _tolerance(value) -> float:
    if type(value) not in (int, float):
        raise ValueError("match_tolerance_rad must be finite and positive")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError("match_tolerance_rad must be finite and positive") from exc
    if not math.isfinite(result) or result <= 0:
        raise ValueError("match_tolerance_rad must be finite and positive")
    return result


def _context_problem(profile: EnvironmentProfile, snapshot: RobotSnapshot,
                     identity: EnvironmentIdentity) -> str | None:
    if type(snapshot) is not RobotSnapshot or type(identity) is not EnvironmentIdentity:
        raise TypeError("RobotSnapshot and EnvironmentIdentity are required")
    if snapshot.robot_id is None or identity.environment_id is None or identity.environment_version is None or identity.mount is None:
        return "identity_unknown"
    if identity.tool_id is None:
        return "tool_unknown"
    if (snapshot.robot_id != profile.robot_id or
            identity.environment_id != profile.environment_id or
            identity.environment_version != profile.environment_version or
            identity.mount != profile.mount or identity.tool_id != profile.tool_id):
        return "profile_mismatch"
    return None


def _feedback_problem(snapshot: RobotSnapshot) -> str | None:
    if snapshot.joints_quality.state != "valid" or snapshot.joint_rad is None:
        return "joint_feedback_unknown"
    if snapshot.status_quality.state != "valid" or snapshot.error_code != 0:
        return "controller_feedback_unknown_or_faulted"
    if len(snapshot.drivers) != 7:
        return "driver_feedback_unknown"
    for driver in snapshot.drivers:
        if driver.quality.state != "valid" or driver.undervoltage is not False or driver.driver_error is not False:
            return "driver_feedback_unknown_or_faulted"
    return None


def _drivers_disabled(snapshot: RobotSnapshot) -> bool:
    return any(driver.enabled is False for driver in snapshot.drivers)


def _matches(snapshot: RobotSnapshot, point, tolerance: float) -> bool:
    return (snapshot.joint_rad is not None and
            max(abs(a - b) for a, b in zip(snapshot.joint_rad, point.joint_rad)) <= tolerance)


def _result_plan(operation: str, profile: EnvironmentProfile, snapshot: RobotSnapshot,
                 identity: EnvironmentIdentity, tolerance: float, *, origin=None,
                 destination=None, asset=None, reasons=(), requires_enable=False,
                 no_motion=False) -> OperationPlan:
    refs = (origin,) if no_motion and origin is not None else (
        tuple(profile.point(name) for name in asset.ordered_point_refs) if asset else ())
    hashes = tuple((point.name, point.source_waypoint_sha256) for point in refs)
    route = None
    if asset is not None:
        route = prepare_joint_route(refs[0].source_waypoint_path,
                                    [point.source_waypoint_path for point in refs[1:]])
    return OperationPlan(
        operation, profile.environment_id, profile.environment_version,
        origin.name if origin else None, destination.name if destination else None,
        asset.route_id if asset else None, route, hashes,
        asset.evidence_ref if asset else None, requires_enable,
        not reasons and not requires_enable, tuple(reasons), profile.source_path,
        profile.source_sha256, identity, snapshot, tolerance)


def _prepare(operation: str, profile_path: str | Path, snapshot: RobotSnapshot,
             identity: EnvironmentIdentity, *, match_tolerance_rad: float,
             route_id: str | None = None,
             route_candidate: JointRoute | None = None) -> OperationPlan:
    profile = _profile(profile_path)
    tolerance = _tolerance(match_tolerance_rad)
    problem = _context_problem(profile, snapshot, identity)
    if problem is None:
        problem = _feedback_problem(snapshot)
    if problem:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            reasons=(problem,))
    role = "park" if operation == "park" else "ready"
    targets = [point for point in profile.points if point.role == role and point.state == "validated"]
    if len(targets) != 1:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            reasons=("validated_target_missing_or_ambiguous",))
    destination = targets[0]
    current = [point for point in profile.points
               if point.state == "validated" and _matches(snapshot, point, tolerance)]
    if len(current) != 1:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            destination=destination, reasons=("no_applicable_route",))
    origin = current[0]
    disabled = _drivers_disabled(snapshot)
    if operation == "start" and origin.name == destination.name:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            origin=origin, destination=destination, no_motion=True,
                            reasons=("driver_disabled",) if disabled else ())
    if operation == "start" and origin.role != "park":
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            origin=origin, destination=destination,
                            reasons=("no_applicable_route",))
    if operation == "return_ready" and route_id is None and route_candidate is None:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            origin=origin, destination=destination,
                            reasons=("no_applicable_route",))
    routes = [route for route in profile.routes if
              route.from_point == origin.name and route.to_point == destination.name and
              (route_id is None or route.route_id == route_id)]
    if route_candidate is not None:
        if type(route_candidate) is not JointRoute:
            raise TypeError("route_candidate must be JointRoute")
        if route_id is None:
            routes = [route for route in routes if route_candidate == prepare_joint_route(
                profile.point(route.from_point).source_waypoint_path,
                [profile.point(name).source_waypoint_path for name in route.ordered_point_refs[1:]])]
    if not routes:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            origin=origin, destination=destination,
                            reasons=("no_applicable_route",))
    if len(routes) > 1:
        return _result_plan(operation, profile, snapshot, identity, tolerance,
                            origin=origin, destination=destination,
                            reasons=("ambiguous_route",))
    asset = routes[0]
    reasons = []
    if asset.state != "validated":
        reasons.append(f"route_{asset.state}")
    if operation not in asset.allowed_operations:
        reasons.append("operation_not_allowed")
    if not asset.directional:
        reasons.append("direction_not_validated")
    if any(profile.point(name).state != "validated" for name in asset.ordered_point_refs):
        reasons.append("route_point_not_validated")
    if route_candidate is not None:
        expected = prepare_joint_route(profile.point(asset.from_point).source_waypoint_path,
                                       [profile.point(name).source_waypoint_path for name in
                                        asset.ordered_point_refs[1:]])
        if route_candidate != expected:
            reasons.append("route_candidate_mismatch")
    if disabled:
        if operation == "start" and origin.role == "park":
            reasons.append("requires_enable_adapter")
        else:
            reasons.append("driver_disabled")
    return _result_plan(operation, profile, snapshot, identity, tolerance,
                        origin=origin, destination=destination, asset=asset,
                        reasons=reasons, requires_enable=disabled and operation == "start" and
                        origin.role == "park")


def prepare_start(profile_path, snapshot, identity, *, match_tolerance_rad,
                  route_id=None) -> OperationPlan:
    return _prepare("start", profile_path, snapshot, identity,
                    match_tolerance_rad=match_tolerance_rad, route_id=route_id)


def prepare_park(profile_path, snapshot, identity, *, match_tolerance_rad,
                 route_id=None) -> OperationPlan:
    return _prepare("park", profile_path, snapshot, identity,
                    match_tolerance_rad=match_tolerance_rad, route_id=route_id)


def prepare_return_ready(profile_path, snapshot, identity, *, match_tolerance_rad,
                         route_id=None, route_candidate=None) -> OperationPlan:
    return _prepare("return_ready", profile_path, snapshot, identity,
                    match_tolerance_rad=match_tolerance_rad, route_id=route_id,
                    route_candidate=route_candidate)


def _execute(plan: OperationPlan, operation: str, *, parameters: RouteParameters,
             preflight_check, read_feedback, publish_target, cancel_requested,
             stop_motion, monotonic_s, sleep_s) -> OperationResult:
    if type(plan) is not OperationPlan or plan.operation != operation:
        raise TypeError(f"{operation} OperationPlan is required")
    if not plan.executable_under_profile:
        status = "no_applicable_route" if "no_applicable_route" in plan.reasons else "rejected"
        return OperationResult(status, plan, None, True)
    if load_environment_profile(plan.profile_path).source_sha256 != plan.profile_sha256:
        raise ValueError("Environment Profile changed after planning")
    if operation == "start":
        fresh = prepare_start(plan.profile_path, plan.prepared_snapshot, plan.identity,
                              match_tolerance_rad=plan.match_tolerance_rad,
                              route_id=plan.route_id)
    elif operation == "park":
        fresh = prepare_park(plan.profile_path, plan.prepared_snapshot, plan.identity,
                             match_tolerance_rad=plan.match_tolerance_rad,
                             route_id=plan.route_id)
    else:
        fresh = prepare_return_ready(plan.profile_path, plan.prepared_snapshot, plan.identity,
                                     match_tolerance_rad=plan.match_tolerance_rad,
                                     route_id=plan.route_id)
    if fresh != plan:
        raise ValueError("OperationPlan differs from verified Environment Profile")
    if plan.route is None:
        if not callable(preflight_check) or not callable(read_feedback) or not callable(cancel_requested):
            raise TypeError("preflight_check, read_feedback and cancel_requested are required")
        if cancel_requested():
            return OperationResult("cancelled", plan, None, True)
        if preflight_check() is not None:
            raise TypeError("preflight_check must return None or raise")
        live = read_feedback()
        if (type(live) is not RobotSnapshot or
                _context_problem(_profile(plan.profile_path), live, plan.identity) or
                _feedback_problem(live) or _drivers_disabled(live) or
                not _matches(live, _profile(plan.profile_path).point(plan.to_point),
                             plan.match_tolerance_rad)):
            return OperationResult("rejected", plan, None, True)
        return OperationResult("completed", plan, None, True)
    motion = execute_joint_route(
        plan.route, parameters=parameters, preflight_check=preflight_check,
        read_feedback=read_feedback, publish_target=publish_target,
        cancel_requested=cancel_requested, stop_motion=stop_motion,
        monotonic_s=monotonic_s, sleep_s=sleep_s)
    return OperationResult(motion.status, plan, motion, False)


def execute_start(plan, *, parameters, preflight_check, read_feedback,
                  publish_target, cancel_requested, stop_motion, monotonic_s,
                  sleep_s) -> OperationResult:
    return _execute(plan, "start", parameters=parameters,
                    preflight_check=preflight_check, read_feedback=read_feedback,
                    publish_target=publish_target, cancel_requested=cancel_requested,
                    stop_motion=stop_motion, monotonic_s=monotonic_s, sleep_s=sleep_s)


def execute_park(plan, *, parameters, preflight_check, read_feedback,
                 publish_target, cancel_requested, stop_motion, monotonic_s,
                 sleep_s) -> OperationResult:
    return _execute(plan, "park", parameters=parameters,
                    preflight_check=preflight_check, read_feedback=read_feedback,
                    publish_target=publish_target, cancel_requested=cancel_requested,
                    stop_motion=stop_motion, monotonic_s=monotonic_s, sleep_s=sleep_s)


def execute_return_ready(plan, *, parameters, preflight_check, read_feedback,
                         publish_target, cancel_requested, stop_motion, monotonic_s,
                         sleep_s) -> OperationResult:
    return _execute(plan, "return_ready", parameters=parameters,
                    preflight_check=preflight_check, read_feedback=read_feedback,
                    publish_target=publish_target, cancel_requested=cancel_requested,
                    stop_motion=stop_motion, monotonic_s=monotonic_s, sleep_s=sleep_s)


def prepare_teach_return(guided_result: GuidedRecordingResult, profile_path,
                         identity: EnvironmentIdentity, *, match_tolerance_rad,
                         route_id=None) -> TeachReturnPlan:
    if type(guided_result) is not GuidedRecordingResult:
        raise TypeError("GuidedRecordingResult is required")
    if (guided_result.workflow_outcome != "completed" or
            guided_result.final_state != "known" or
            guided_result.final_snapshot is None):
        return TeachReturnPlan(guided_result, None, "guided_recording_not_completed")
    plan = prepare_return_ready(profile_path, guided_result.final_snapshot, identity,
                                match_tolerance_rad=match_tolerance_rad, route_id=route_id)
    return TeachReturnPlan(guided_result, plan,
                           None if plan.executable_under_profile else
                           "; ".join(plan.reasons))


def execute_teach_return(teach_plan: TeachReturnPlan, *, parameters,
                         preflight_check, read_feedback, publish_target,
                         cancel_requested, stop_motion, monotonic_s, sleep_s) -> TeachV1Result:
    if type(teach_plan) is not TeachReturnPlan:
        raise TypeError("TeachReturnPlan is required")
    guided = teach_plan.guided_result
    if (type(guided) is not GuidedRecordingResult or
            guided.workflow_outcome != "completed" or guided.final_state != "known" or
            guided.final_snapshot is None or
            (teach_plan.return_plan is not None and
             guided.final_snapshot != teach_plan.return_plan.prepared_snapshot)):
        return TeachV1Result(guided, None, False, "guided_recording_not_completed")
    if teach_plan.return_plan is None or not teach_plan.return_plan.executable_under_profile:
        return TeachV1Result(guided, None, False,
                             teach_plan.reason or "no_applicable_route")
    result = execute_return_ready(
        teach_plan.return_plan, parameters=parameters,
        preflight_check=preflight_check, read_feedback=read_feedback,
        publish_target=publish_target, cancel_requested=cancel_requested,
        stop_motion=stop_motion, monotonic_s=monotonic_s, sleep_s=sleep_s)
    complete = result.status == "completed"
    return TeachV1Result(guided, result, complete,
                         None if complete else result.status)
