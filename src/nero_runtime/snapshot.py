"""把调用方已取得的纯内存反馈解析为不可变的设备事实。

输入仅为普通 dict/list/tuple 与内建标量，不接受 SDK 对象。每个非空来源
都须有 timestamp_s；observed_at_s 与所有时间戳由调用方保证采用同一
秒数时基，time_basis 只是该时基的记录标签。本模块不读取时钟，也不
证明这些消息同步到达。缺失、无效或过期来源的字段保留为 None。

status 键为 ctrl_mode、arm_status、teach_status、motion_status、err_code；
joints 键为 joint_rad（7 项 rad）；flange 键为 flange_pose_m_rad
（[x,y,z,roll,pitch,yaw]，位置 m、角度 rad，非 TCP）；drivers 为按 joint
编号 1～7 的记录，含 enabled、undervoltage、driver_error、voltage_v。
这些扁平 dict 是未来采集者的输入契约，不是 SDK 消息协议。
"""

from dataclasses import dataclass, replace
import math
from typing import Literal


QualityState = Literal["valid", "missing", "invalid", "stale"]


@dataclass(frozen=True)
class SourceQuality:
    state: QualityState
    timestamp_s: float | None
    age_s: float | None
    issues: tuple[str, ...]


@dataclass(frozen=True)
class DriverSnapshot:
    joint: int
    enabled: bool | None
    undervoltage: bool | None
    driver_error: bool | None
    voltage_v: float | None
    quality: SourceQuality


@dataclass(frozen=True)
class RobotSnapshot:
    robot_id: str | None
    observed_at_s: float
    time_basis: str
    status_quality: SourceQuality
    joints_quality: SourceQuality
    flange_quality: SourceQuality
    control_mode: int | None
    arm_status: int | None
    teach_status: int | None
    motion_status: int | None
    error_code: int | None
    uninterpreted_status_fields: tuple[str, ...]
    joint_rad: tuple[float, ...] | None
    flange_pose_m_rad: tuple[float, ...] | None
    drivers: tuple[DriverSnapshot, ...]
    issues: tuple[str, ...]


def _finite_number(value: object) -> float | None:
    if type(value) not in (int, float):
        return None
    try:
        number = float(value)
    except OverflowError:
        return None
    return number if math.isfinite(number) else None


def _argument_seconds(value: object, name: str) -> float:
    number = _finite_number(value)
    if number is None or number < 0:
        raise ValueError(f"{name} must be a finite, non-negative number of seconds")
    return number


def _time_quality(
    message: object, source: str, observed_at_s: float, max_age_s: float
) -> SourceQuality:
    if message is None:
        return SourceQuality("missing", None, None, (f"{source}: missing",))
    if type(message) is not dict:
        return SourceQuality("invalid", None, None, (f"{source}: expected dict",))
    timestamp = _finite_number(message.get("timestamp_s"))
    if timestamp is None or timestamp < 0:
        return SourceQuality(
            "invalid", None, None, (f"{source}: timestamp_s missing or invalid",)
        )
    age = observed_at_s - timestamp
    if age < 0:
        return SourceQuality(
            "invalid", timestamp, age, (f"{source}: timestamp_s is in the future",)
        )
    if age > max_age_s:
        return SourceQuality(
            "stale", timestamp, age, (f"{source}: age exceeds max_age_s",)
        )
    return SourceQuality("valid", timestamp, age, ())


def _invalid_field(quality: SourceQuality, source: str, field: str) -> SourceQuality:
    return replace(
        quality, state="invalid", issues=(f"{source}: {field} missing or invalid",)
    )


def _vector(value: object, length: int) -> tuple[float, ...] | None:
    if type(value) not in (list, tuple) or len(value) != length:
        return None
    numbers = tuple(_finite_number(item) for item in value)
    if any(item is None for item in numbers):
        return None
    return numbers


def _parse_status(message: object, quality: SourceQuality):
    if quality.state != "valid":
        return quality, (None,) * 5, ()
    fields = ("ctrl_mode", "arm_status", "teach_status", "motion_status", "err_code")
    for field in fields:
        if type(message.get(field)) is not int:
            return _invalid_field(quality, "status", field), (None,) * 5, ()
    values = tuple(message[field] for field in fields)
    # 仅标记旧源码有明确使用的状态码；这里不推导健康或许可。
    known = ((1,), (0, 6), (0, 2, 6), (0, 1))
    uninterpreted = tuple(
        field for field, value, choices in zip(fields[:4], values[:4], known)
        if value not in choices
    )
    return quality, values, uninterpreted


def _parse_vector(message: object, quality: SourceQuality, source: str,
                  field: str, length: int):
    if quality.state != "valid":
        return quality, None
    vector = _vector(message.get(field), length)
    if vector is None:
        return _invalid_field(quality, source, field), None
    return quality, vector


def _unknown_driver(joint: int, quality: SourceQuality) -> DriverSnapshot:
    return DriverSnapshot(joint, None, None, None, None, quality)


def _parse_drivers(drivers: object, observed_at_s: float, max_age_s: float):
    if drivers is None:
        return tuple(
            _unknown_driver(joint, SourceQuality(
                "missing", None, None, (f"driver[{joint}]: missing",)
            )) for joint in range(1, 8)
        ), ()
    if type(drivers) not in (list, tuple):
        return tuple(
            _unknown_driver(joint, SourceQuality(
                "invalid", None, None, ("drivers: expected list or tuple",)
            )) for joint in range(1, 8)
        ), ("drivers: expected list or tuple",)

    by_joint: dict[int, list[dict]] = {}
    indexing_issues = []
    for position, item in enumerate(drivers):
        if type(item) is not dict or type(item.get("joint")) is not int:
            indexing_issues.append(f"drivers[{position}]: joint missing or invalid")
            continue
        joint = item["joint"]
        if joint not in range(1, 8):
            indexing_issues.append(f"drivers[{position}]: joint {joint} outside 1..7")
            continue
        by_joint.setdefault(joint, []).append(item)

    parsed = []
    for joint in range(1, 8):
        source = f"driver[{joint}]"
        records = by_joint.get(joint, [])
        if not records:
            parsed.append(_unknown_driver(joint, SourceQuality(
                "missing", None, None, (f"{source}: missing",)
            )))
            continue
        if len(records) != 1:
            parsed.append(_unknown_driver(joint, SourceQuality(
                "invalid", None, None, (f"{source}: duplicate joint number",)
            )))
            continue
        record = records[0]
        quality = _time_quality(record, source, observed_at_s, max_age_s)
        if quality.state != "valid":
            parsed.append(_unknown_driver(joint, quality))
            continue
        for field in ("enabled", "undervoltage", "driver_error"):
            if type(record.get(field)) is not bool:
                quality = _invalid_field(quality, source, field)
                break
        voltage = _finite_number(record.get("voltage_v"))
        if quality.state == "valid" and voltage is None:
            quality = _invalid_field(quality, source, "voltage_v")
        if quality.state != "valid":
            parsed.append(_unknown_driver(joint, quality))
        else:
            parsed.append(DriverSnapshot(
                joint, record["enabled"], record["undervoltage"],
                record["driver_error"], voltage, quality
            ))
    return tuple(parsed), tuple(indexing_issues)


def build_snapshot(
    *, robot_id: str | None, observed_at_s: float, time_basis: str,
    status: dict | None, joints: dict | None, flange: dict | None,
    drivers: list[dict] | tuple[dict, ...] | None,
    max_status_age_s: float, max_joints_age_s: float,
    max_flange_age_s: float, max_driver_age_s: float,
) -> RobotSnapshot:
    """解析一次调用方给定的扁平反馈；所有输入时间使用同一秒数时基。

    时间与最大年龄参数无效会抛 ValueError。来源数据异常成为质量问题，
    不补默认健康值；即使所有来源 valid，也不代表停稳或允许运动。
    """
    if robot_id is not None and (type(robot_id) is not str or not robot_id):
        raise ValueError("robot_id must be a non-empty string or None")
    if type(time_basis) is not str or not time_basis.strip():
        raise ValueError("time_basis must identify the caller's shared seconds clock")
    observed = _argument_seconds(observed_at_s, "observed_at_s")
    ages = tuple(_argument_seconds(value, name) for name, value in (
        ("max_status_age_s", max_status_age_s),
        ("max_joints_age_s", max_joints_age_s),
        ("max_flange_age_s", max_flange_age_s),
        ("max_driver_age_s", max_driver_age_s),
    ))
    status_quality, status_values, uninterpreted = _parse_status(
        status, _time_quality(status, "status", observed, ages[0])
    )
    joints_quality, joint_values = _parse_vector(
        joints, _time_quality(joints, "joints", observed, ages[1]),
        "joints", "joint_rad", 7
    )
    flange_quality, flange_values = _parse_vector(
        flange, _time_quality(flange, "flange", observed, ages[2]),
        "flange", "flange_pose_m_rad", 6
    )
    driver_values, indexing_issues = _parse_drivers(drivers, observed, ages[3])
    issues = tuple(
        issue for quality in (status_quality, joints_quality, flange_quality)
        for issue in quality.issues
    ) + tuple(issue for driver in driver_values for issue in driver.quality.issues) + indexing_issues
    return RobotSnapshot(
        robot_id, observed, time_basis, status_quality, joints_quality,
        flange_quality, *status_values, uninterpreted, joint_values,
        flange_values, driver_values, issues
    )
