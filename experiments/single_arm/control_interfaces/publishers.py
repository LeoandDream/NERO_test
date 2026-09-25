"""关节目标、法兰点位目标与法兰直线目标分别使用 SDK 运动接口。

这里只规定目标类型、单位和下发方式。路线规划、限位、工作空间、
反馈监控和故障处理不能省略，必须由上层实验脚本实现。
"""

import math
from typing import Sequence


def _finite_vector(values: Sequence[float], length: int, name: str) -> list[float]:
    """在发命令之前拒绝缺项、非数字或非有限数值。"""
    if isinstance(values, (str, bytes)) or len(values) != length:
        raise ValueError(f"{name} 必须包含 {length} 个数值")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} 含非有限数值")
    return result


class JointTargetPublisher:
    """发布七轴关节角目标；已有回放和恢复路线均使用 move_j。"""

    mode = "j"

    def publish(self, robot, joints: Sequence[float]) -> None:
        target = _finite_vector(joints, 7, "关节角")
        robot.move_j(target)


class CartesianLinearPublisher:
    """发布法兰六维位姿目标；由控制器进行 IK 和直线运动。"""

    mode = "l"

    def publish(self, robot, pose: Sequence[float]) -> None:
        robot.move_l(_checked_pose(pose))


class CartesianPointPublisher:
    """发布法兰六维终点；控制器选 IK 和点位运动路径。"""

    mode = "p"

    def publish(self, robot, pose: Sequence[float]) -> None:
        robot.move_p(_checked_pose(pose))


def _checked_pose(pose: Sequence[float]) -> list[float]:
    target = _finite_vector(pose, 6, "法兰位姿")
    if not all(-math.pi <= angle <= math.pi for angle in (target[3], target[5])):
        raise ValueError("法兰 roll/yaw 超出 SDK 角度范围")
    if not -math.pi / 2 <= target[4] <= math.pi / 2:
        raise ValueError("法兰 pitch 超出 SDK 角度范围")
    return target
