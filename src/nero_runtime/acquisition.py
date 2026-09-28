"""单轮反馈采集：只读取调用方提供的源并复制现有快照内核所需字段。

源需提供 get_arm_status()、get_joint_angles()、get_flange_pose()、
get_driver_states(joint_index)。返回值按已静态检查的 NERO SDK 形状：
None 或带 timestamp、msg 的对象；状态/驱动字段位于 msg 中。
本模块不导入 SDK，不创建、连接或关闭源。getter 自身是否阻塞由源负责。

clock_s 由调用方注入；其返回值和所有反馈 timestamp 必须使用已经核实的
同一秒数时基，time_basis 只记录调用方的时基声明。本模块不转换时钟。
顺序复制减少可变缓存被事后修改的影响，但不保证跨线程或跨帧原子一致。
"""

from dataclasses import dataclass
import math
from typing import Callable

from .snapshot import RobotSnapshot, build_snapshot


@dataclass(frozen=True)
class AcquisitionResult:
    snapshot: RobotSnapshot
    started_at_s: float
    ended_at_s: float
    read_errors: tuple[str, ...]


def _plain(value: object) -> object:
    """仅保留内建不可变标量；未知对象交给解析器作为无效缺项。"""
    if value is None or type(value) in (bool, int, float, str):
        return value
    if isinstance(value, int):  # SDK 整型枚举若出现，保留其原始数值。
        return int(value)
    return None


def _vector(value: object) -> object:
    if type(value) not in (list, tuple):
        return _plain(value)
    return [_plain(item) for item in value]


def _copy_status(raw: object) -> dict | None:
    if raw is None:
        return None
    msg = getattr(raw, "msg", None)
    return {
        "timestamp_s": _plain(getattr(raw, "timestamp", None)),
        "ctrl_mode": _plain(getattr(msg, "ctrl_mode", None)),
        "arm_status": _plain(getattr(msg, "arm_status", None)),
        "teach_status": _plain(getattr(msg, "teach_status", None)),
        "motion_status": _plain(getattr(msg, "motion_status", None)),
        "err_code": _plain(getattr(msg, "err_code", None)),
    }


def _copy_joints(raw: object) -> dict | None:
    if raw is None:
        return None
    return {
        "timestamp_s": _plain(getattr(raw, "timestamp", None)),
        "joint_rad": _vector(getattr(raw, "msg", None)),
    }


def _copy_flange(raw: object) -> dict | None:
    if raw is None:
        return None
    return {
        "timestamp_s": _plain(getattr(raw, "timestamp", None)),
        "flange_pose_m_rad": _vector(getattr(raw, "msg", None)),
    }


def _copy_driver(raw: object, joint: int) -> dict | None:
    if raw is None:
        return None
    msg = getattr(raw, "msg", None)
    foc = getattr(msg, "foc_status", None)
    return {
        "joint": joint,  # getter 按该编号读取 parser 的对应驱动缓存。
        "timestamp_s": _plain(getattr(raw, "timestamp", None)),
        "enabled": _plain(getattr(foc, "driver_enable_status", None)),
        "undervoltage": _plain(getattr(foc, "voltage_too_low", None)),
        "driver_error": _plain(getattr(foc, "driver_error_status", None)),
        "voltage_v": _plain(getattr(msg, "vol", None)),
    }


def _read(source: object, method_name: str, args: tuple,
          copier: Callable, errors: list[str]) -> object:
    getter = getattr(source, method_name)  # 缺方法是接口错误，不能伪装成缺反馈。
    try:
        return copier(getter(*args))
    except OSError as exc:
        label = method_name + (f"({args[0]})" if args else "")
        errors.append(f"{label}: {type(exc).__name__}: {exc}")
        return None


def _clock_read(clock_s: Callable[[], float]) -> float:
    value = clock_s()
    if type(value) not in (int, float):
        raise ValueError("clock_s must return finite non-negative seconds")
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError("clock_s must return finite non-negative seconds") from exc
    if not math.isfinite(value) or value < 0:
        raise ValueError("clock_s must return finite non-negative seconds")
    return value


def collect_snapshot(
    source: object, *, robot_id: str | None, clock_s: Callable[[], float],
    time_basis: str, max_status_age_s: float, max_joints_age_s: float,
    max_flange_age_s: float, max_driver_age_s: float,
) -> AcquisitionResult:
    """按状态、关节、法兰、驱动 1～7 顺序各读取一次，再调用纯解析。

    仅 OSError（含 TimeoutError/ConnectionError）记为预期读取失败；其它
    异常及 KeyboardInterrupt/SystemExit 原样抛出，绝不触发控制动作。
    `ended_at_s` 是解析观测时刻；来源 timestamp 保持原值。
    """
    started = _clock_read(clock_s)
    errors: list[str] = []
    status = _read(source, "get_arm_status", (), _copy_status, errors)
    joints = _read(source, "get_joint_angles", (), _copy_joints, errors)
    flange = _read(source, "get_flange_pose", (), _copy_flange, errors)
    drivers = []
    for joint in range(1, 8):
        item = _read(source, "get_driver_states", (joint,),
                     lambda raw, index=joint: _copy_driver(raw, index), errors)
        if item is not None:
            drivers.append(item)
    ended = _clock_read(clock_s)
    if ended < started:
        raise ValueError("clock_s moved backwards during acquisition")
    snapshot = build_snapshot(
        robot_id=robot_id, observed_at_s=ended, time_basis=time_basis,
        status=status, joints=joints, flange=flange, drivers=drivers,
        max_status_age_s=max_status_age_s, max_joints_age_s=max_joints_age_s,
        max_flange_age_s=max_flange_age_s, max_driver_age_s=max_driver_age_s,
    )
    return AcquisitionResult(snapshot, started, ended, tuple(errors))
