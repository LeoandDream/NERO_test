"""Authorize bounded drag starts only at the previously stable S1 pose.

A low table pose moved abruptly on drag entry on 2026-09-26. This gate does
not identify the cause or make that low pose safe.
"""

from dataclasses import dataclass
import math
import time


S1_JOINT_RAD = (
    0.017383479349863524, -0.5787511799613198, 0.028047441079548877,
    -0.2995159529347469, 0.011239920382843483, 0.2056695990550118,
    -0.47343801289598186,
)
S1_FLANGE_M = (0.38204661962732317, -0.014074540274628126, 0.572442804950921)
JOINT_TOLERANCE_RAD = 0.01
FLANGE_TOLERANCE_M = 0.015
MAX_FEEDBACK_AGE_S = 1.0
MAX_TOKEN_AGE_S = 2.0


class DragStartBlocked(RuntimeError):
    """The current state has not been authorized for drag transition."""


@dataclass(frozen=True)
class DragStartAuthorization:
    issued_mono: float


def _fresh(message):
    if message is None:
        return False
    try:
        age = time.time() - float(message.timestamp)
    except (AttributeError, TypeError, ValueError):
        return False
    return 0 <= age <= MAX_FEEDBACK_AGE_S


def _read_s1(robot, *, require_can_mode):
    status = robot.get_arm_status()
    joints = robot.get_joint_angles()
    flange = robot.get_flange_pose()
    if not all(_fresh(item) for item in (status, joints, flange)):
        raise DragStartBlocked("拖动启动被拦截：状态、关节或法兰反馈不新鲜。")
    allowed_modes = (1,) if require_can_mode else (1, 2)
    if (int(status.msg.ctrl_mode) not in allowed_modes or int(status.msg.arm_status) != 0
            or int(status.msg.teach_status) not in (0, 6) or int(status.msg.err_code) != 0):
        raise DragStartBlocked("拖动启动被拦截：控制器未处于正常 CAN 空闲状态。")
    try:
        q = tuple(float(value) for value in joints.msg)
        xyz = tuple(float(value) for value in flange.msg[:3])
    except (TypeError, ValueError, AttributeError) as exc:
        raise DragStartBlocked("拖动启动被拦截：反馈格式无效。") from exc
    if (len(q) != 7 or len(xyz) != 3
            or not all(math.isfinite(value) for value in (*q, *xyz))
            or max(abs(a-b) for a, b in zip(q, S1_JOINT_RAD)) > JOINT_TOLERANCE_RAD
            or math.dist(xyz, S1_FLANGE_M) > FLANGE_TOLERANCE_M):
        raise DragStartBlocked("拖动启动被拦截：机械臂不在已验证的 S1 姿态。")
    return q


def authorize_s1_drag_start(robot, *, mount, require_can_mode=False):
    """Require fresh, stable S1 feedback and seven healthy enabled drives."""
    if mount != "left":
        raise DragStartBlocked("拖动启动被拦截：仅允许已核对的左侧安装配置。")
    first = _read_s1(robot, require_can_mode=require_can_mode)
    for index in range(1, 8):
        driver = robot.get_driver_states(index)
        if not _fresh(driver):
            raise DragStartBlocked(f"拖动启动被拦截：关节 {index} 驱动反馈不新鲜。")
        foc = driver.msg.foc_status
        if (not foc.driver_enable_status or foc.voltage_too_low
                or foc.driver_error_status):
            raise DragStartBlocked(f"拖动启动被拦截：关节 {index} 未使能或有故障。")
    time.sleep(0.25)
    second = _read_s1(robot, require_can_mode=require_can_mode)
    if max(abs(a-b) for a, b in zip(first, second)) > 0.001:
        raise DragStartBlocked("拖动启动被拦截：S1 姿态仍在变化。")
    return DragStartAuthorization(time.monotonic())


def require_drag_start_safe(authorization=None):
    """Reject drag transition frames without a recent S1 authorization."""
    if (not isinstance(authorization, DragStartAuthorization)
            or not 0 <= time.monotonic() - authorization.issued_mono <= MAX_TOKEN_AGE_S):
        raise DragStartBlocked(
            "拖动启动被拦截：必须在已验证的 S1 姿态获取实时授权；"
            "桌边低位及未限时的手动拖动仍锁定。"
        )
