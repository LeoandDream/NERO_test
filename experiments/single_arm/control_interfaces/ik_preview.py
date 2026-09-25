"""仅用于 move_l 事前筛查的本机六维数值逆解，不向控制器发命令。"""

import math

from experiments.single_arm.teaching.teach_session import validate_joints


def angle_error(goal: float, actual: float) -> float:
    """把欧拉角差折回 [-pi, pi]，避免跨周界时走远路。"""
    return (goal - actual + math.pi) % (2 * math.pi) - math.pi


def pose_error(goal, actual):
    """返回三维位置误差和三维 RPY 误差。"""
    return [goal[i] - actual[i] for i in range(3)] + [
        angle_error(goal[i], actual[i]) for i in range(3, 6)
    ]


def solve_linear(matrix, vector):
    """主元高斯消元；这里最多解 6×6 的阻尼法方程。"""
    size = len(vector)
    rows = [list(matrix[i]) + [vector[i]] for i in range(size)]
    for col in range(size):
        pivot = max(range(col, size), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-12:
            raise ValueError("本机 IK 雅可比矩阵不可解")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        scale = rows[col][col]
        for k in range(col, size + 1):
            rows[col][k] /= scale
        for row in range(size):
            if row == col:
                continue
            factor = rows[row][col]
            for k in range(col, size + 1):
                rows[row][k] -= factor * rows[col][k]
    return [rows[i][size] for i in range(size)]


def solve_nearby_pose(robot, initial, target, zero_radius):
    """寻找初值附近的六维位姿解，只用于筛查直接笛卡尔命令。

    该解不等于控制器内部 IK 的承诺；实机仍须逐帧监控关节路径。
    """
    if len(target) != 6 or not all(math.isfinite(x) for x in target):
        raise ValueError("目标法兰位姿无效")
    q = validate_joints(initial, zero_radius)
    weights = [1.0, 1.0, 1.0, 3.0, 1.0, 1.0, 1.0]
    for _ in range(100):
        current = [float(value) for value in robot.fk(q)]
        error = pose_error(target, current)
        if math.dist(error[:3], [0.0] * 3) < 0.0002 and max(
            abs(value) for value in error[3:]
        ) < 0.003:
            return validate_joints(q, zero_radius)
        jacobian = [[0.0] * 7 for _ in range(6)]
        for joint in range(7):
            probe = list(q)
            probe[joint] += 0.0001
            moved = [float(value) for value in robot.fk(probe)]
            for axis in range(3):
                jacobian[axis][joint] = (moved[axis] - current[axis]) / 0.0001
            for axis in range(3, 6):
                jacobian[axis][joint] = angle_error(moved[axis], current[axis]) / 0.0001
        normal = [
            [sum(jacobian[a][j] * jacobian[b][j] / weights[j] for j in range(7))
             + (0.005 ** 2 if a == b else 0.0) for b in range(6)]
            for a in range(6)
        ]
        correction = solve_linear(normal, error)
        delta = [
            sum(jacobian[axis][joint] * correction[axis] for axis in range(6))
            / weights[joint] for joint in range(7)
        ]
        length = math.dist(delta, [0.0] * 7)
        scale = min(1.0, 0.02 / max(length, 1e-12))
        accepted = False
        baseline = math.dist(error, [0.0] * 6)
        for _ in range(12):
            candidate = [value + scale * change for value, change in zip(q, delta)]
            try:
                validate_joints(candidate, zero_radius)
            except ValueError:
                scale *= 0.5
                continue
            candidate_error = pose_error(target, robot.fk(candidate))
            if math.dist(candidate_error, [0.0] * 6) < baseline:
                q = candidate
                accepted = True
                break
            scale *= 0.5
        if not accepted:
            raise ValueError("本机六维 IK 无法在关节限位内收敛")
    raise ValueError("本机六维 IK 未收敛")
