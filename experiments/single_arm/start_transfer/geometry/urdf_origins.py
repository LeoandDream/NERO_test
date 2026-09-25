"""Nero URDF 的关节/连杆原点正运动学，仅用于离线几何检查。

模型来自 AgileX agx_arm_urdf 的 Nero 裸臂 URDF；这里没有载入碰撞网格，
因此计算出的点和范围不能证明整条连杆、线缆或现场障碍物安全。
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


DEFAULT_URDF = Path(__file__).with_name("nero_description.urdf")


def multiply(left, right):
    """相乘两个 4×4 齐次变换矩阵。"""
    return [
        [sum(left[row][axis] * right[axis][col] for axis in range(4))
         for col in range(4)]
        for row in range(4)
    ]


def identity():
    return [[float(row == col) for col in range(4)] for row in range(4)]


def translation(xyz):
    matrix = identity()
    for axis in range(3):
        matrix[axis][3] = xyz[axis]
    return matrix


def rotate_x(angle):
    matrix = identity()
    cosine, sine = math.cos(angle), math.sin(angle)
    matrix[1][1], matrix[1][2] = cosine, -sine
    matrix[2][1], matrix[2][2] = sine, cosine
    return matrix


def rotate_y(angle):
    matrix = identity()
    cosine, sine = math.cos(angle), math.sin(angle)
    matrix[0][0], matrix[0][2] = cosine, sine
    matrix[2][0], matrix[2][2] = -sine, cosine
    return matrix


def rotate_z(angle):
    matrix = identity()
    cosine, sine = math.cos(angle), math.sin(angle)
    matrix[0][0], matrix[0][1] = cosine, -sine
    matrix[1][0], matrix[1][1] = sine, cosine
    return matrix


def parse_joint_origins(path=DEFAULT_URDF):
    """读取七个转动关节；未知或改变的 URDF 结构直接拒绝。"""
    root = ET.parse(path).getroot()
    if root.get("name") != "nero":
        raise ValueError("URDF 不是 Nero 模型")
    joints = []
    for index in range(1, 8):
        joint = root.find(f"./joint[@name='joint{index}']")
        if joint is None or joint.get("type") != "revolute":
            raise ValueError(f"URDF 缺少转动关节 joint{index}")
        origin = joint.find("origin")
        axis = joint.find("axis")
        if origin is None or axis is None or axis.get("xyz") != "0 0 1":
            raise ValueError(f"joint{index} 的原点或旋转轴不符合预期")
        xyz = [float(value) for value in origin.get("xyz").split()]
        rpy = [float(value) for value in origin.get("rpy").split()]
        if len(xyz) != 3 or len(rpy) != 3:
            raise ValueError(f"joint{index} 的 URDF 原点数据无效")
        joints.append((xyz, rpy))
    return joints


def link_transforms(joints_rad, model):
    """输出 link1..link7 在底座坐标系中的完整 4×4 变换。"""
    if len(joints_rad) != 7 or not all(math.isfinite(value) for value in joints_rad):
        raise ValueError("需要七个有限关节角")
    transform = identity()
    result = {}
    for index, (angle, (xyz, rpy)) in enumerate(zip(joints_rad, model), 1):
        origin = multiply(
            multiply(multiply(translation(xyz), rotate_z(rpy[2])), rotate_y(rpy[1])),
            rotate_x(rpy[0]),
        )
        transform = multiply(multiply(transform, origin), rotate_z(angle))
        result[f"link{index}"] = transform
    return result


def link_origins(joints_rad, model):
    """输出 link1..link7 各自原点在底座坐标系中的 XYZ，单位 m。"""
    return {
        name: [transform[axis][3] for axis in range(3)]
        for name, transform in link_transforms(joints_rad, model).items()
    }


def link_origin_envelopes(stages, model, samples_per_stage=100):
    """逐段线性插值的各连杆原点包围范围；不是实际控制器路径。"""
    bounds = {f"link{index}": [[math.inf, -math.inf] for _ in range(3)]
              for index in range(1, 8)}
    for start, target in zip(stages, stages[1:]):
        for step in range(samples_per_stage + 1):
            fraction = step / samples_per_stage
            joints = [a + fraction * (b - a) for a, b in zip(start, target)]
            for name, position in link_origins(joints, model).items():
                for axis in range(3):
                    bounds[name][axis][0] = min(bounds[name][axis][0], position[axis])
                    bounds[name][axis][1] = max(bounds[name][axis][1], position[axis])
    return {name: dict(zip(("x", "y", "z"), axes)) for name, axes in bounds.items()}
