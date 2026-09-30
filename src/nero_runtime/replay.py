"""F3 原件到 F2 关节路点执行的组合；候选资格不是现场运动许可。"""

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

from .motion import JointRoute, MotionResult, RouteParameters, RoutePoint, execute_joint_route
from .recording import load_recording_summary


@dataclass(frozen=True)
class ReplayPlan:
    schema_version: int
    recording_id: str
    source_summary_path: Path
    source_raw_path: Path
    source_raw_sha256: str
    robot_id: str | None
    sample_count: int
    expected_start_joint_rad: tuple[float, ...]
    target_joint_rad: tuple[tuple[float, ...], ...]
    source_sample_indices: tuple[int, ...]
    source_elapsed_s: tuple[float, ...]
    playback_mode: str = "joint_waypoint"
    timing_preserved: bool = False


@dataclass(frozen=True)
class ReplayResult:
    status: str
    recording_id: str
    plan: ReplayPlan
    motion_result: MotionResult
    source_raw_sha256: str
    timing_preserved: bool = False


def prepare_replay(summary_path: str | Path) -> ReplayPlan:
    """先由 F3 校验摘要/原件，再检查每一轮能否成为无删减关节路点。"""
    path = Path(summary_path)
    summary = load_recording_summary(path)
    if not summary["complete"] or summary["stop_reason"] not in ("duration_complete", "user_finish"):
        raise ValueError("recording did not finish normally")
    if summary["sample_count"] < 2:
        raise ValueError("replay needs at least two recorded samples")
    raw_path = path.with_name(summary["raw_path"])
    joints = []
    indices = []
    elapsed = []
    identity = None
    time_basis = None
    raw_bytes = raw_path.read_bytes()
    if hashlib.sha256(raw_bytes).hexdigest() != summary["raw_sha256"]:
        raise ValueError("recording raw SHA256 changed during Replay preparation")
    for line in raw_bytes.decode("utf-8").splitlines():
        row = json.loads(line)
        index = row["sample_index"]
        snap = row["snapshot"]
        if index == 0:
            identity = snap["robot_id"]
            time_basis = snap["time_basis"]
        if (snap["robot_id"] != identity or snap["time_basis"] != time_basis or
                (identity is not None and (type(identity) is not str or not identity.strip())) or
                type(time_basis) is not str or not time_basis.strip()):
            raise ValueError(f"sample {index}: robot identity or time basis changed")
        values = snap["joint_rad"]
        if snap["joints_quality"]["state"] != "valid" or type(values) is not list or len(values) != 7:
            raise ValueError(f"sample {index}: joint feedback is not a finite valid J1..J7 point")
        try:
            if any(type(value) not in (int, float) or not math.isfinite(float(value))
                   for value in values):
                raise ValueError("non-finite joint value")
            joint = tuple(float(value) for value in values)
        except (ValueError, OverflowError) as exc:
            raise ValueError(f"sample {index}: joint feedback is not a finite valid J1..J7 point") from exc
        if (snap["status_quality"]["state"] != "valid" or
                type(snap["error_code"]) is not int or snap["error_code"] != 0):
            raise ValueError(f"sample {index}: controller feedback missing or faulted")
        drivers = snap["drivers"]
        if (type(drivers) is not list or len(drivers) != 7 or any(
                type(driver) is not dict or driver.get("joint") != joint or
                type(driver.get("quality")) is not dict or
                driver["quality"].get("state") != "valid" or
                driver.get("enabled") is not True or
                driver.get("undervoltage") is not False or
                driver.get("driver_error") is not False
                for joint, driver in enumerate(drivers, 1))):
            raise ValueError(f"sample {index}: driver feedback missing, disabled or faulted")
        joints.append(joint)
        indices.append(index)
        elapsed.append(float(row["elapsed_s"]))
    if len(joints) != summary["sample_count"] or indices != list(range(len(joints))):
        raise ValueError("recording raw sample sequence changed during Replay preparation")
    return ReplayPlan(
        schema_version=1, recording_id=summary["recording_id"],
        source_summary_path=path, source_raw_path=raw_path,
        source_raw_sha256=summary["raw_sha256"], robot_id=identity,
        sample_count=summary["sample_count"], expected_start_joint_rad=joints[0],
        target_joint_rad=tuple(joints[1:]), source_sample_indices=tuple(indices),
        source_elapsed_s=tuple(elapsed),
    )


def replay_route(plan: ReplayPlan) -> JointRoute:
    """保留所有样本及来源索引，交由 F2 的路线模型验证关节数据。"""
    if type(plan) is not ReplayPlan:
        raise TypeError("ReplayPlan is required")
    points = tuple(RoutePoint(joint_rad=joint_rad, name=None,
                              source=f"recording {plan.recording_id} sample {index}",
                              robot_id=plan.robot_id)
                   for index, joint_rad in zip(plan.source_sample_indices,
                                               (plan.expected_start_joint_rad,
                                                *plan.target_joint_rad)))
    return JointRoute(points[0], points[1:])


def execute_replay(plan: ReplayPlan, *, parameters: RouteParameters,
                   preflight_check, read_feedback, publish_target, cancel_requested,
                   stop_motion, monotonic_s, sleep_s) -> ReplayResult:
    """执行前重新核对原件与计划；现场许可及停止映射仍由调用方提供。"""
    if type(plan) is not ReplayPlan:
        raise TypeError("ReplayPlan is required")
    if prepare_replay(plan.source_summary_path) != plan:
        raise ValueError("ReplayPlan differs from verified recording source")
    route = replay_route(plan)
    motion = execute_joint_route(
        route, parameters=parameters, preflight_check=preflight_check,
        read_feedback=read_feedback, publish_target=publish_target,
        cancel_requested=cancel_requested, stop_motion=stop_motion,
        monotonic_s=monotonic_s, sleep_s=sleep_s)
    return ReplayResult(motion.status, plan.recording_id, plan, motion,
                        plan.source_raw_sha256)


def format_replay_plan(plan: ReplayPlan) -> str:
    """纯离线预览，不表示该路线或当前机器人获准执行。"""
    if type(plan) is not ReplayPlan:
        raise TypeError("ReplayPlan is required")
    return "\n".join((
        "Replay candidate: yes",
        f"Recording: {plan.recording_id}",
        f"Samples: {plan.sample_count}",
        f"Targets: {len(plan.target_joint_rad)}",
        "Expected start: " + ", ".join(f"J{i}={angle:.6f} rad" for i, angle in
                                        enumerate(plan.expected_start_joint_rad, 1)),
        f"Playback mode: {plan.playback_mode}",
        "Timing preserved: no",
        f"Source SHA256: {plan.source_raw_sha256}",
        "Candidate does not grant motion permission.",
    ))
