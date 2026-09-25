#!/usr/bin/env python3
"""从实时 S1 姿态规划法兰沿基座 X 负方向 0.10 m，并原路返回。"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

import can
from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

from experiments.single_arm.can_setup.can_drag_teach import fresh, wait_status
from experiments.single_arm.start_transfer.go_to_start import (
    SPEED_PERCENT, START_TOLERANCE_RAD, check_preplan_drivers,
    execute_route, stable_joint_feedback,
)
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, JOINT_LIMITS, JOINT_LIMIT_MARGIN_RAD, check_drivers,
    healthy_preteach_status, joint_point, load_config, validate_flange_workspace,
    validate_joints,
)


DX_M = -0.10
# 先前每 5 mm 发一次 move_j，现场反馈每个小步交界处有顿挫。新版将
# 执行目标间距放宽到至多 0.10 rad，但仍每 0.01 rad 用 FK 检查中间路径。
CARTESIAN_STEP_M = 0.01
JOINT_STEP_RAD = 0.10
FK_SAMPLE_STEP_RAD = 0.01
XYZ_TOLERANCE_M = 0.003
ENDPOINT_TOLERANCE_M = 0.005
MIN_LIMIT_CLEARANCE_RAD = 0.05
J4_MIN_CLEARANCE_RAD = 0.10
# J4 在早期示教中接近过边界；位置逆解给它更高代价，尽量让其他关节分担位移。
J4_WEIGHT = 3.0
SCHEMA_VERSION = 2


def xyz(robot, joints):
    """从 SDK FK 取得有限的法兰中心 XYZ 坐标，单位米。"""
    result = [float(value) for value in robot.fk(joints)[:3]]
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError("正运动学没有返回有限法兰坐标")
    return result


def solve_3x3(matrix, vector):
    """用带主元选择的高斯消元求解阻尼最小二乘的三维位置校正。"""
    rows = [list(row) + [value] for row, value in zip(matrix, vector)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(rows[row][col]))
        if abs(rows[pivot][col]) < 1e-12:
            raise ValueError("逆运动学雅可比矩阵奇异")
        rows[col], rows[pivot] = rows[pivot], rows[col]
        divisor = rows[col][col]
        for index in range(col, 4):
            rows[col][index] /= divisor
        for row in range(3):
            if row == col:
                continue
            factor = rows[row][col]
            for index in range(col, 4):
                rows[row][index] -= factor * rows[col][index]
    return [rows[row][3] for row in range(3)]


def solve_position(robot, initial, target, zero_radius):
    """只解法兰 XYZ，使用带 J4 权重的阻尼最小二乘；不约束姿态。"""
    q = list(initial)
    weights = [1.0, 1.0, 1.0, J4_WEIGHT, 1.0, 1.0, 1.0]
    for _ in range(80):
        position = xyz(robot, q)
        error = [goal - actual for goal, actual in zip(target, position)]
        if math.dist(position, target) < 0.0002:
            return validate_joints(q, zero_radius)
        # SDK 只提供 FK：对每个关节做很小的正向扰动，数值估计 3×7 位置雅可比。
        jacobian = [[0.0] * 7 for _ in range(3)]
        for joint in range(7):
            perturbed = list(q)
            perturbed[joint] += 0.0001
            moved = xyz(robot, perturbed)
            for axis in range(3):
                jacobian[axis][joint] = (moved[axis] - position[axis]) / 0.0001
        # 解 (J W^-1 J^T + λ²I)u = 位置误差，再由 W^-1 J^T u 映回关节增量。
        # 阻尼 λ 可降低近奇异姿态下的关节突变；W 的 J4 权重由上方常量给出。
        normal = [
            [sum(jacobian[a][j] * jacobian[b][j] / weights[j] for j in range(7))
             + (0.005 ** 2 if a == b else 0.0) for b in range(3)]
            for a in range(3)
        ]
        correction = solve_3x3(normal, error)
        delta = [
            sum(jacobian[axis][joint] * correction[axis] for axis in range(3))
            / weights[joint] for joint in range(7)
        ]
        scale = min(1.0, 0.03 / max(math.dist(delta, [0.0] * 7), 1e-12))
        accepted = False
        # 回溯步长：候选解必须仍在限位内，且 FK 误差要确实下降。
        for _ in range(12):
            candidate = [value + scale * change for value, change in zip(q, delta)]
            try:
                validate_joints(candidate, zero_radius)
            except ValueError:
                scale *= 0.5
                continue
            if math.dist(xyz(robot, candidate), target) < math.dist(position, target):
                q = candidate
                accepted = True
                break
            scale *= 0.5
        if not accepted:
            raise ValueError("逆运动学无法在关节限位内继续收敛")
    raise ValueError("逆运动学未收敛")


def limit_clearance(joints):
    """计算七关节离 SDK 限位加脚本余量边界的最小距离。"""
    return [
        min(value - (lower + JOINT_LIMIT_MARGIN_RAD),
            (upper - JOINT_LIMIT_MARGIN_RAD) - value)
        for value, (lower, upper) in zip(joints, JOINT_LIMITS)
    ]


def plan_outbound(robot, start, config, dx_m=DX_M):
    """把所选基座 X 位移拆成位置目标，再限制每次 move_j 的关节跨度。"""
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    origin = xyz(robot, start)
    anchors = [start]
    count = math.ceil(abs(dx_m) / CARTESIAN_STEP_M)
    for step in range(1, count + 1):
        target = [origin[0] + dx_m * step / count,
                  origin[1], origin[2]]
        anchors.append(solve_position(robot, anchors[-1], target,
                                      config["zero_exclusion_radius_rad"]))
    # IK 锚点间的关节位移可能大于执行器单次目标，因此再次细分。
    route = [start]
    for target in anchors[1:]:
        previous = route[-1]
        segments = max(1, math.ceil(math.dist(previous, target) / JOINT_STEP_RAD))
        for index in range(1, segments + 1):
            fraction = index / segments
            route.append([a + fraction * (b - a) for a, b in zip(previous, target)])
    if len(route) > 200:
        raise ValueError("规划小步过多")
    assess_route(robot, route, config, dx_m)
    return route


def assess_route(robot, route, config, dx_m=DX_M):
    """细采样整条关节路线，检查 X 单调性、Y/Z 漂移、限位和法兰范围。"""
    if not route or len(route) < 2:
        raise ValueError("轨迹点不足")
    origin = xyz(robot, route[0])
    goal = [origin[0] + dx_m, origin[1], origin[2]]
    envelope = {axis: [float("inf"), float("-inf")] for axis in ("x", "y", "z")}
    clearances = [float("inf")] * 7
    max_yz_error = 0.0
    last_x = None
    # move_j 在关节空间行走；只查锚点的 FK 会漏掉两点之间的法兰偏移。
    for first, second in zip(route, route[1:]):
        if math.dist(first, second) > JOINT_STEP_RAD + 1e-9:
            raise ValueError("相邻关节小步过大")
        count = max(1, math.ceil(math.dist(first, second) / FK_SAMPLE_STEP_RAD))
        for part in range(count + 1):
            fraction = part / count
            q = [a + fraction * (b - a) for a, b in zip(first, second)]
            validate_joints(q, config["zero_exclusion_radius_rad"])
            validate_flange_workspace(robot, q, config)
            margin = limit_clearance(q)
            clearances = [min(a, b) for a, b in zip(clearances, margin)]
            point = xyz(robot, q)
            for axis, value in zip(("x", "y", "z"), point):
                envelope[axis][0] = min(envelope[axis][0], value)
                envelope[axis][1] = max(envelope[axis][1], value)
            max_yz_error = max(max_yz_error, abs(point[1] - origin[1]),
                               abs(point[2] - origin[2]))
            if last_x is not None and point[0] > last_x + 0.0005:
                raise ValueError("法兰 X 轨迹不是单调减小")
            last_x = point[0]
            if max_yz_error > XYZ_TOLERANCE_M:
                raise ValueError("法兰 Y/Z 偏离目标直线超过 3 mm")
    if min(clearances) < MIN_LIMIT_CLEARANCE_RAD or clearances[3] < J4_MIN_CLEARANCE_RAD:
        raise ValueError("计算轨迹离关节限位过近")
    endpoint = xyz(robot, route[-1])
    if math.dist(endpoint, goal) > 0.001:
        raise ValueError("法兰终点未达到目标")
    return {
        "start_xyz_m": origin,
        "target_xyz_m": goal,
        "predicted_endpoint_xyz_m": endpoint,
        "flange_xyz_envelope_m": envelope,
        "max_yz_error_m": max_yz_error,
        "joint_limit_clearance_rad": clearances,
        "joint_travel_rad": sum(math.dist(a, b) for a, b in zip(route, route[1:])),
    }


def make_plan(robot, start, config, dx_m=DX_M):
    """从实时起点生成一次性规划及可供现场复核的摘要。"""
    route = plan_outbound(robot, start, config, dx_m)
    summary = assess_route(robot, route, config, dx_m)
    return {
        "schema_version": SCHEMA_VERSION,
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "mount": config["mount"],
        "safe_start_joint_rad": config["safe_start_joint_rad"],
        "flange_workspace_m": config["flange_workspace_m"],
        "displacement_x_m": dx_m,
        "speed_percent": SPEED_PERCENT,
        "outbound_joint_rad": route,
        "summary": summary,
        "executed": False,
    }


def check_plan(robot, plan, config):
    """重新求解并核对所有路点和摘要，拒绝改动或尝试过的规划。"""
    if (plan.get("schema_version") != SCHEMA_VERSION or plan.get("executed")
            or plan.get("execution_attempts")):
        raise ValueError("规划版本错误、已执行或已尝试执行")
    dx_m = plan.get("displacement_x_m")
    if (not isinstance(dx_m, (float, int)) or not math.isfinite(dx_m)
            or not -0.10 <= dx_m <= -0.01):
        raise ValueError("规划 X 位移超出 0.01～0.10 m 的负向试验范围")
    if (plan.get("mount") != config["mount"]
            or plan.get("flange_workspace_m") != config["flange_workspace_m"]
            or plan.get("safe_start_joint_rad") != config["safe_start_joint_rad"]
            or plan.get("speed_percent") != SPEED_PERCENT):
        raise ValueError("规划与当前配置不一致")
    expected = plan_outbound(robot, plan["outbound_joint_rad"][0], config, dx_m)
    recorded = plan["outbound_joint_rad"]
    if len(expected) != len(recorded) or any(
        len(point) != 7 or max(abs(a - b) for a, b in zip(point, solved)) > 1e-8
        for point, solved in zip(recorded, expected)
    ):
        raise ValueError("规划关节路线被修改或算法不匹配")
    summary = assess_route(robot, expected, config, dx_m)
    for key in ("start_xyz_m", "target_xyz_m", "predicted_endpoint_xyz_m",
                "max_yz_error_m", "joint_limit_clearance_rad", "joint_travel_rad"):
        actual = plan["summary"][key]
        calculated = summary[key]
        left = actual if isinstance(actual, list) else [actual]
        right = calculated if isinstance(calculated, list) else [calculated]
        if len(left) != len(right) or any(abs(a - b) > 1e-8 for a, b in zip(left, right)):
            raise ValueError("规划摘要被修改或正运动学不匹配")
    for axis in ("x", "y", "z"):
        if any(abs(a - b) > 1e-8 for a, b in zip(
            plan["summary"]["flange_xyz_envelope_m"][axis],
            summary["flange_xyz_envelope_m"][axis]
        )):
            raise ValueError("规划法兰范围与正运动学不匹配")
    return expected, summary


def run_plan(robot, config, route, summary, aborted):
    """执行外行程并沿关节路线倒序返回，运动中监测法兰通道。"""
    origin = summary["start_xyz_m"]
    target = summary["target_xyz_m"]
    for remaining in range(5, 0, -1):
        if aborted():
            raise InterruptedError("已取消运动")
        print(f"{remaining} 秒后开始法兰 X 负向运动；按 Ctrl+C 取消。", flush=True)
        time.sleep(1)

    def monitor(index, _joint_target):
        actual = xyz(robot, joint_point(robot))
        expected = xyz(robot, route[index])
        if math.dist(actual, expected) > ENDPOINT_TOLERANCE_M:
            raise RuntimeError("实际法兰位置偏离预定小步超过 5 mm")
        if abs(actual[1] - origin[1]) > XYZ_TOLERANCE_M + ENDPOINT_TOLERANCE_M or \
                abs(actual[2] - origin[2]) > XYZ_TOLERANCE_M + ENDPOINT_TOLERANCE_M:
            raise RuntimeError("实际法兰 Y/Z 偏离目标直线")

    def corridor_feedback(joints):
        actual = xyz(robot, joints)
        if (not summary["flange_xyz_envelope_m"]["x"][0] - ENDPOINT_TOLERANCE_M
                <= actual[0] <= summary["flange_xyz_envelope_m"]["x"][1]
                + ENDPOINT_TOLERANCE_M
                or abs(actual[1] - origin[1]) > XYZ_TOLERANCE_M + ENDPOINT_TOLERANCE_M
                or abs(actual[2] - origin[2]) > XYZ_TOLERANCE_M + ENDPOINT_TOLERANCE_M):
            raise RuntimeError("实时法兰位置偏离规划通道")

    execute_route(robot, config, route, aborted, on_waypoint=monitor,
                  on_feedback=corridor_feedback, target_tolerance_rad=0.005)
    endpoint = xyz(robot, joint_point(robot))
    if math.dist(endpoint, target) > ENDPOINT_TOLERANCE_M:
        robot.electronic_emergency_stop()
        raise RuntimeError("外行程终点法兰位置偏差超过 5 mm；已急停，未返回")
    if aborted():
        raise InterruptedError("外行程完成后收到中止信号；未自动返回")
    print("外行程完成，沿原关节路径返回本次实际起点。", flush=True)
    reverse = list(reversed(route))

    def reverse_monitor(index, _joint_target):
        actual = xyz(robot, joint_point(robot))
        expected = xyz(robot, reverse[index])
        if math.dist(actual, expected) > ENDPOINT_TOLERANCE_M:
            raise RuntimeError("返回时实际法兰位置偏离预定小步超过 5 mm")

    execute_route(robot, config, reverse, aborted, on_waypoint=reverse_monitor,
                  on_feedback=corridor_feedback, target_tolerance_rad=0.005)
    final_q = joint_point(robot)
    if max(abs(a - b) for a, b in zip(final_q, route[0])) > START_TOLERANCE_RAD:
        raise RuntimeError("返回后关节位置偏离本次实际起点")
    return final_q


def main():
    """默认只读生成或复核规划；只有 --run 才执行受监控的往返。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--plan", type=Path, help="复核已有规划文件")
    parser.add_argument("--run", action="store_true", help="执行规划的往返运动")
    parser.add_argument("--distance-m", type=float, default=0.10,
                        help="新规划的基座 X 负向位移，0.01～0.10 m；建议先试 0.02 m")
    args = parser.parse_args()
    if args.run and args.plan is None:
        parser.error("--run 需要提供 --plan")
    if not math.isfinite(args.distance_m) or not 0.01 <= args.distance_m <= 0.10:
        parser.error("--distance-m 必须在 0.01～0.10 m 之间")
    if args.plan and args.distance_m != 0.10:
        parser.error("--distance-m 仅用于生成新规划；复核时位移从规划文件读取")
    try:
        config = load_config(args.config)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, lambda item: fresh(item), timeout=3) is None:
                raise RuntimeError("机械臂状态反馈缺失")
            healthy_preteach_status(robot)
            if args.run:
                check_drivers(robot)
            else:
                check_preplan_drivers(robot)
            current = validate_joints(stable_joint_feedback(robot),
                                      config["zero_exclusion_radius_rad"])
            if max(abs(a - b) for a, b in zip(
                current, config["safe_start_joint_rad"]
            )) > START_TOLERANCE_RAD:
                raise RuntimeError("当前位置不在 S1 附近；请先安全返回 S1")
            if args.plan:
                plan = json.loads(args.plan.read_text(encoding="utf-8"))
                route, summary = check_plan(robot, plan, config)
                if max(abs(a - b) for a, b in zip(current, route[0])) > START_TOLERANCE_RAD:
                    raise RuntimeError("当前位置已偏离规划起点；请重新生成规划")
            else:
                plan = make_plan(robot, current, config, -args.distance_m)
                route, summary = plan["outbound_joint_rad"], plan["summary"]
                output = Path("experiments/single_arm/cartesian_x/data/plans") / (
                    "nero_cartesian_x_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
                )
                output.parent.mkdir(parents=True, exist_ok=True)
                with output.open("x", encoding="utf-8") as stream:
                    stream.write(json.dumps(plan, ensure_ascii=False, indent=2) + "\n")
                print(f"只读规划完成：{output}")
            print(f"外行程 {len(route)-1} 小步，速度 {SPEED_PERCENT}%，再原路返回")
            print("法兰起点/目标 (m)：", summary["start_xyz_m"], summary["target_xyz_m"])
            print("法兰预测范围 (m)：", summary["flange_xyz_envelope_m"])
            print("最小关节限位余量 (rad)：", summary["joint_limit_clearance_rad"])
            print(f"法兰 Y/Z 最大预测偏差 {summary['max_yz_error_m']:.6f} m")
            if not args.run:
                print("未发送运动指令。")
                return 0

            aborted = False

            def on_signal(_signum, _frame):
                nonlocal aborted
                aborted = True

            previous_int = signal.signal(signal.SIGINT, on_signal)
            previous_term = signal.signal(signal.SIGTERM, on_signal)
            attempt = {"started_at_utc": datetime.now(timezone.utc).isoformat(),
                       "start_joint_rad": current, "completed": False}
            plan.setdefault("execution_attempts", []).append(attempt)
            args.plan.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            try:
                final_q = run_plan(robot, config, route, summary, lambda: aborted)
                attempt["final_joint_rad"] = final_q
                attempt["return_max_joint_error_rad"] = max(
                    abs(a - b) for a, b in zip(final_q, route[0])
                )
                attempt["completed"] = True
                plan["executed"] = True
                print("已完成外行程并返回本次实际起点。", flush=True)
            except Exception as exc:
                attempt["error"] = str(exc)
                raise
            finally:
                attempt["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                args.plan.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                signal.signal(signal.SIGINT, previous_int)
                signal.signal(signal.SIGTERM, previous_term)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError,
            KeyError, TypeError, IndexError, can.CanError) as exc:
        print(f"法兰 X 运动未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
