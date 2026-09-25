#!/usr/bin/env python3
"""从急停恢复后的实测姿态分段规划到 S1；每次显式执行一段。"""

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
    MAX_TRAVEL_RAD, execute_route, flange_envelope, interpolate,
    stable_joint_feedback,
)
from experiments.single_arm.teaching.teach_session import (
    CAN_IDLE_TEACH_STATUSES, DEFAULT_CONFIG, check_drivers, healthy_status,
    joint_point, load_config, validate_joints, validate_route_workspace,
)


PLAN_DIR = Path("experiments/single_arm/start_transfer/data/plans")
START_TOLERANCE_RAD = 0.005
STAGE_TARGET_TOLERANCE_RAD = 0.005
STAGE_DESCRIPTIONS = (
    "仅调整 J4 到 S1；J2 保持在当前正角，避免两关节同时接近零位",
    "保持 J4 弯曲，J2 移至中间角",
    "保持 J4 弯曲，J2 移至 S1 角",
    "J7 移至中间角；法兰位置几乎不变，但腕部姿态改变",
    "J7 移至 S1 角；法兰位置几乎不变，但腕部姿态改变",
    "对齐 J1、J3、J5、J6 至 S1",
)
COUPLED_DESCRIPTIONS = (
    "J4 先弯至 S1，同时让 J7 向目标移动四分之一",
    "保持 J4 弯曲，J2 移至中间角，J7 同步移动",
    "保持 J4 弯曲，J2 移至 S1 角，J7 同步移动",
    "J1、J3、J5、J6 对齐 S1，同时完成 J7 剩余行程",
)
COUPLED_STEP_RAD = 0.025
AFTER_REPLAY_DESCRIPTIONS = (
    "保持 J2/J4 弯曲，先展开 J3 到中间角",
    "继续展开 J3 到 S1 角，J2/J4 保持不变",
    "保持 J4 弯曲，J2 移到中间角",
    "保持 J4 弯曲，J2 移到 S1 角",
    "对齐 J1、J4、J5、J6、J7 到 S1",
)


def build_stages(start, target, config):
    """按单关节先弯曲、再跨 J2 零位的顺序生成六段小步路线。"""
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    target = validate_joints(target, config["zero_exclusion_radius_rad"])
    if start[1] < 0.1 or not -0.01 <= start[3] <= 0.15 or target[3] >= -0.2:
        raise ValueError("当前 J2/J4 不符合本次恢复路线的起点形态；拒绝套用")
    points = [list(start)]
    j4_bent = list(start)
    j4_bent[3] = target[3]
    points.append(j4_bent)

    j2_middle = list(j4_bent)
    j2_middle[1] = (start[1] + target[1]) / 2
    points.append(j2_middle)
    j2_done = list(j2_middle)
    j2_done[1] = target[1]
    points.append(j2_done)

    j7_middle = list(j2_done)
    j7_middle[6] = (start[6] + target[6]) / 2
    points.append(j7_middle)
    j7_done = list(j7_middle)
    j7_done[6] = target[6]
    points.append(j7_done)
    points.append(list(target))

    stages = []
    for number, (first, last) in enumerate(zip(points, points[1:]), 1):
        distance = math.dist(first, last)
        if distance > MAX_TRAVEL_RAD:
            raise ValueError(f"第 {number} 段长度 {distance:.3f} rad 超过单段上限")
        route = [
            validate_joints(point, config["zero_exclusion_radius_rad"])
            for point in interpolate(first, last)
        ]
        stages.append({
            "number": number,
            "description": STAGE_DESCRIPTIONS[number - 1],
            "joint_distance_rad": distance,
            "waypoints_joint_rad": route,
        })
    return stages


def build_coupled_stages(start, target, config):
    """从第二次急停恢复姿态出发，使每个 J7 小步伴随其他关节变化。"""
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    target = validate_joints(target, config["zero_exclusion_radius_rad"])
    if not (-0.2 <= start[1] <= 0.2 and -0.2 <= start[3] <= -0.03
            and 0.35 <= start[6] <= 0.55 and target[3] < -0.2):
        raise ValueError("实时 J2/J4/J7 不符合联动回 S1 的起点形态")

    points = [list(start)]
    # 先把 J4 弯至 S1，再让 J2 穿过零位；J7 行程均匀分摊到四段。
    j4_bent = list(start)
    j4_bent[3] = target[3]
    j4_bent[6] = start[6] + (target[6] - start[6]) * 0.25
    points.append(j4_bent)

    j2_middle = list(j4_bent)
    j2_middle[1] = (start[1] + target[1]) / 2
    j2_middle[6] = start[6] + (target[6] - start[6]) * 0.50
    points.append(j2_middle)

    j2_done = list(j2_middle)
    j2_done[1] = target[1]
    j2_done[6] = start[6] + (target[6] - start[6]) * 0.75
    points.append(j2_done)
    points.append(list(target))

    stages = []
    for number, (first, last) in enumerate(zip(points, points[1:]), 1):
        distance = math.dist(first, last)
        if distance > MAX_TRAVEL_RAD:
            raise ValueError(f"第 {number} 段关节距离超过单段上限")
        steps = max(1, math.ceil(distance / COUPLED_STEP_RAD))
        route = []
        for index in range(steps + 1):
            fraction = index / steps
            point = [a + fraction * (b - a) for a, b in zip(first, last)]
            route.append(validate_joints(point, config["zero_exclusion_radius_rad"]))
        route[0] = list(first)
        route[-1] = list(last)
        for previous, current in zip(route, route[1:]):
            j7_change = abs(current[6] - previous[6])
            other_change = max(abs(current[i] - previous[i]) for i in range(6))
            if j7_change < 0.008 or other_change < 0.005:
                raise ValueError("联动路线存在仅 J7 变化或变化过小的小步")
        stages.append({
            "number": number,
            "description": COUPLED_DESCRIPTIONS[number - 1],
            "joint_distance_rad": distance,
            "waypoints_joint_rad": route,
        })
    return stages


def build_after_replay_stages(start, target, config):
    """从本次长回放阻尼下垂姿态出发，先展开 J3，再让 J2 跨零位。"""
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    target = validate_joints(target, config["zero_exclusion_radius_rad"])
    if not (0.1 <= start[1] <= 0.45 and -1.5 <= start[2] <= -0.9
            and -0.45 <= start[3] <= -0.2 and -0.6 <= start[6] <= -0.3):
        raise ValueError("实时 J2/J3/J4/J7 不符合本次长回放后的恢复姿态")
    points = [list(start)]
    j3_middle = list(start)
    j3_middle[2] = (start[2] + target[2]) / 2
    points.append(j3_middle)
    j3_done = list(j3_middle)
    j3_done[2] = target[2]
    points.append(j3_done)
    j2_middle = list(j3_done)
    j2_middle[1] = (start[1] + target[1]) / 2
    points.append(j2_middle)
    j2_done = list(j2_middle)
    j2_done[1] = target[1]
    points.append(j2_done)
    points.append(list(target))

    stages = []
    for number, (first, last) in enumerate(zip(points, points[1:]), 1):
        distance = math.dist(first, last)
        if distance > MAX_TRAVEL_RAD:
            raise ValueError(f"第 {number} 段关节距离超过单段上限")
        route = [
            validate_joints(point, config["zero_exclusion_radius_rad"])
            for point in interpolate(first, last)
        ]
        stages.append({
            "number": number,
            "description": AFTER_REPLAY_DESCRIPTIONS[number - 1],
            "joint_distance_rad": distance,
            "waypoints_joint_rad": route,
        })
    return stages


def route_builder(strategy):
    """计划创建和复核使用同一生成器，拒绝未定义的恢复策略。"""
    builders = {
        "original": build_stages,
        "coupled-j7": build_coupled_stages,
        "after-replay-sag": build_after_replay_stages,
    }
    if strategy not in builders:
        raise ValueError("未知恢复策略")
    return builders[strategy]


def validate_plan(plan, config):
    """重算全部路线，防止修改目标、阶段或安装方向后误执行旧计划。"""
    strategy = plan.get("strategy", "original")
    expected_schema = {"original": 1, "coupled-j7": 2,
                       "after-replay-sag": 3}.get(strategy)
    if (expected_schema is None
            or plan.get("schema_version") != expected_schema
            or plan.get("mount") != config["mount"]):
        raise ValueError("计划版本或安装方向不匹配")
    if plan.get("flange_workspace_m") != config["flange_workspace_m"]:
        raise ValueError("法兰工作空间配置发生变化")
    target = config["safe_start_joint_rad"]
    if len(plan["target_joint_rad"]) != 7 or max(
        abs(a - b) for a, b in zip(plan["target_joint_rad"], target)
    ) > 1e-9:
        raise ValueError("S1 目标与当前配置不匹配")
    builder = route_builder(strategy)
    expected = builder(plan["start_joint_rad"], target, config)
    stages = plan.get("stages")
    if not isinstance(stages, list) or len(stages) != len(expected):
        raise ValueError("阶段数量不匹配")
    for actual, planned in zip(stages, expected):
        route = actual.get("waypoints_joint_rad")
        correct = planned["waypoints_joint_rad"]
        if (actual.get("number") != planned["number"]
                or actual.get("description") != planned["description"]
                or not math.isclose(actual.get("joint_distance_rad", float("nan")),
                                    planned["joint_distance_rad"], abs_tol=1e-9)
                or not isinstance(route, list)):
            raise ValueError("阶段说明、长度或关节路线格式错误")
        if len(route) != len(correct) or any(
            len(a) != 7 or max(abs(x - y) for x, y in zip(a, b)) > 1e-9
            for a, b in zip(route, correct)
        ):
            raise ValueError("阶段关节路线与当前规划算法不匹配")
    done = plan.get("completed_stages")
    if not isinstance(done, int) or not 0 <= done <= len(expected):
        raise ValueError("已完成阶段数量无效")
    results = plan.get("stage_results")
    if not isinstance(results, list) or len(results) != done or any(
        result.get("number") != index or result.get("completed") is not True
        for index, result in enumerate(results, 1)
    ):
        raise ValueError("阶段执行记录与完成数量不匹配")
    if plan.get("failed"):
        raise ValueError("计划已失败；需重新检查当前姿态并生成新计划")
    return expected


def save_plan(path, plan):
    """把每段执行结果写回计划文件，防止跳段或重复执行。"""
    path.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def wait_stage_target(robot, target, tolerance=STAGE_TARGET_TOLERANCE_RAD):
    """关节反馈须在阶段末端收敛；未收敛时禁止进入下一段。"""
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        current = joint_point(robot)
        error = max(abs(a - b) for a, b in zip(current, target))
        if error <= tolerance:
            return current, error
        time.sleep(0.1)
    raise RuntimeError(f"阶段已发完，但最终最大关节误差 {error:.4f} rad")


def main():
    """无计划参数时只读生成；带计划时只读复核；--run 只执行下一段。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--plan", type=Path, help="已有分段计划；省略时从实时姿态新建")
    parser.add_argument("--run", action="store_true", help="执行计划中的下一段，默认不运动")
    parser.add_argument("--speed-percent", type=int, choices=(5, 10), default=5)
    parser.add_argument("--strategy", choices=("original", "coupled-j7", "after-replay-sag"),
                        help="新建计划的路线策略；长回放急停后使用 after-replay-sag")
    args = parser.parse_args()
    if args.run and args.plan is None:
        parser.error("--run 必须指定已生成并复核的 --plan 文件")

    try:
        config = load_config(args.config)
        robot = AgxArmFactory.create_arm(create_agx_arm_config(
            robot=ArmModel.NERO, firmeware_version=NeroFW.V121, channel=config["channel"], local_loopback=True
        ))
        robot.connect()
        try:
            if wait_status(robot, fresh, timeout=3.0) is None:
                raise RuntimeError("控制器状态反馈缺失")
            healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
            check_drivers(robot)
            current = stable_joint_feedback(robot)

            if args.plan is None:
                strategy = args.strategy or "original"
                builder = route_builder(strategy)
                stages = builder(current, config["safe_start_joint_rad"], config)
                for stage in stages:
                    route = stage["waypoints_joint_rad"]
                    validate_route_workspace(robot, route, config)
                    stage["flange_xyz_envelope_m"] = flange_envelope(robot, route, config)
                plan = {
                    "schema_version": {"original": 1, "coupled-j7": 2,
                                       "after-replay-sag": 3}[strategy],
                    "strategy": strategy,
                    "created_at_utc": datetime.now(timezone.utc).isoformat(),
                    "mount": config["mount"],
                    "flange_workspace_m": config["flange_workspace_m"],
                    "start_joint_rad": current,
                    "target_joint_rad": config["safe_start_joint_rad"],
                    "stages": stages,
                    "completed_stages": 0,
                    "stage_results": [],
                    "failed": False,
                }
                PLAN_DIR.mkdir(parents=True, exist_ok=True)
                plan_path = PLAN_DIR / (
                    {"original": "nero_recover_s1_",
                     "coupled-j7": "nero_recover_s1_coupled_",
                     "after-replay-sag": "nero_recover_s1_after_replay_"}[strategy]
                    + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + ".json"
                )
                with plan_path.open("x", encoding="utf-8") as stream:
                    json.dump(plan, stream, ensure_ascii=False, indent=2)
                    stream.write("\n")
            else:
                plan_path = args.plan
                plan = json.loads(plan_path.read_text(encoding="utf-8"))
                validate_plan(plan, config)
                if args.strategy is not None and args.strategy != plan.get("strategy", "original"):
                    raise ValueError("命令指定的路线策略与计划文件不匹配")
                for stage in plan["stages"]:
                    route = stage["waypoints_joint_rad"]
                    validate_route_workspace(robot, route, config)
                    envelope = flange_envelope(robot, route, config)
                    if any(
                        any(abs(a - b) > 1e-6 for a, b in zip(
                            envelope[axis], stage["flange_xyz_envelope_m"][axis]
                        ))
                        for axis in ("x", "y", "z")
                    ):
                        raise ValueError(f"第 {stage['number']} 段法兰范围与当前正运动学结果不匹配")

            print(f"分段计划：{plan_path}")
            print("当前关节角 (rad)：", current)
            print("S1 目标关节角 (rad)：", plan["target_joint_rad"])
            print(f"已完成阶段：{plan['completed_stages']}/{len(plan['stages'])}")
            for stage in plan["stages"]:
                envelope = stage["flange_xyz_envelope_m"]
                print(f"  第 {stage['number']} 段：{stage['description']}；"
                      f"{len(stage['waypoints_joint_rad'])-1} 小步；"
                      f"关节距离 {stage['joint_distance_rad']:.3f} rad；"
                      f"法兰范围 {envelope}")

            if plan["completed_stages"] >= len(plan["stages"]):
                final_error = max(abs(a - b) for a, b in zip(current, plan["target_joint_rad"]))
                if final_error > STAGE_TARGET_TOLERANCE_RAD:
                    raise RuntimeError(
                        f"计划已完成，但当前姿态已偏离 S1 {final_error:.4f} rad"
                    )
                print(f"全部阶段已完成；实时位置仍接近 S1，最大关节误差 {final_error:.6f} rad。")
                return 0
            next_stage = plan["stages"][plan["completed_stages"]]
            route = next_stage["waypoints_joint_rad"]
            error = max(abs(a - b) for a, b in zip(current, route[0]))
            if error > START_TOLERANCE_RAD:
                raise RuntimeError(f"当前位置偏离下一段起点 {error:.4f} rad；不能继续旧计划")
            validate_route_workspace(robot, route, config)
            flange_envelope(robot, route, config)
            print(f"下一段：第 {next_stage['number']} 段；执行速度 {args.speed_percent}%。")
            if not args.run:
                print("只读规划/复核完成；未发送运动指令。")
                return 0

            aborted = False

            def on_signal(_signum, _frame):
                nonlocal aborted
                aborted = True

            previous_int = signal.signal(signal.SIGINT, on_signal)
            previous_term = signal.signal(signal.SIGTERM, on_signal)
            try:
                for remaining in range(5, 0, -1):
                    if aborted:
                        raise InterruptedError("已取消本段运动")
                    print(f"{remaining} 秒后执行第 {next_stage['number']} 段；按 Ctrl+C 取消。", flush=True)
                    time.sleep(1)
                if aborted:
                    raise InterruptedError("已取消本段运动")
                healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                check_drivers(robot)
                current = stable_joint_feedback(robot)
                error = max(abs(a - b) for a, b in zip(current, route[0]))
                if error > START_TOLERANCE_RAD:
                    raise RuntimeError("倒计时期间起点改变；未发送运动指令")

                result = {
                    "number": next_stage["number"],
                    "started_at_utc": datetime.now(timezone.utc).isoformat(),
                    "speed_percent": args.speed_percent,
                    "last_reached_step": 0,
                    "completed": False,
                }
                plan["stage_results"].append(result)
                save_plan(plan_path, plan)
                try:
                    # 运行时限只在小步到位后检查，不能因整段计时到期
                    # 把正常或缓慢的运动误判为故障急停。
                    deadline = time.monotonic() + max(90.0, 20.0 * (len(route) - 1))

                    def reached(index, _target):
                        result["last_reached_step"] = index

                    execute_route(
                        robot, config, route,
                        lambda: aborted,
                        on_waypoint=reached,
                        on_command=lambda index, target: result.update(
                            last_sent_step=index, last_sent_target_joint_rad=list(target)),
                        on_command_published=lambda index, _target: result.update(
                            last_sdk_publish_completed_step=index),
                        on_feedback=lambda joints: result.update(last_feedback_joint_rad=list(joints)),
                        on_status=lambda status: result.update(
                            last_motion_status=str(status.msg.motion_status)),
                        stop_after_waypoint=lambda index, _target: (
                            index < len(route) - 1 and time.monotonic() >= deadline),
                        target_tolerance_rad=(0.003 if plan.get("strategy") == "coupled-j7" else 0.005),
                        speed_percent=args.speed_percent,
                        waypoint_max_timeout_s=(15.0 if plan.get("strategy") == "after-replay-sag" else None),
                        pause_on_stationary_timeout=True,
                    )
                    final, final_error = wait_stage_target(
                        robot, route[-1],
                        tolerance=(0.003 if plan.get("strategy") == "coupled-j7" else STAGE_TARGET_TOLERANCE_RAD),
                    )
                    healthy_status(robot, {1}, CAN_IDLE_TEACH_STATUSES)
                    check_drivers(robot)
                    result["final_joint_rad"] = final
                    result["max_joint_error_rad"] = final_error
                    result["completed"] = True
                    plan["completed_stages"] += 1
                    print(f"第 {next_stage['number']} 段完成；最大关节误差 {final_error:.6f} rad。", flush=True)
                except Exception as exc:
                    result["error"] = str(exc)
                    if hasattr(exc, "reason"):
                        result["pause_reason"] = exc.reason
                    plan["failed"] = True
                    try:
                        result["joint_after_error_rad"] = joint_point(robot)
                    except Exception as feedback_exc:
                        result["joint_after_error_error"] = str(feedback_exc)
                    raise
                finally:
                    result["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
                    save_plan(plan_path, plan)
                    print(f"阶段记录：{plan_path}", flush=True)
            finally:
                signal.signal(signal.SIGINT, previous_int)
                signal.signal(signal.SIGTERM, previous_term)
        finally:
            robot.disconnect()
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError, can.CanError) as exc:
        print(f"回 S1 未完成：{exc}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
