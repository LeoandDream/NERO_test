"""从实时姿态重新规划到命名关节姿态；不读取或倒放示教轨迹。

目前只定义 S1。每段是一个控制器 move_j 目标，控制器自行做段内运动。
离线 FK 和关节盒只能筛掉明显不合适的候选；桌面、支撑和线缆仍须现场核对。
"""

import argparse
from datetime import datetime, timezone
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import sys

DEFAULT_CONFIG = Path("experiments/single_arm/config/nero_teach.json")
PLAN_DIR = Path("experiments/single_arm/lab/data/planned_returns")
RUN_DIR = Path("experiments/single_arm/lab/data/planned_return_runs")
START_TOLERANCE_RAD = 0.003
TARGET_TOLERANCE_RAD = 0.005
MAX_CONTROLLER_TARGET_DISTANCE_RAD = 1.0
# 仅对应 2026-09-27 受监护低位试验的窄起点；不是通用工作空间。
SITE_LOW_REFERENCE_Q = [
    0.2577153173494827, 0.017558012275062956, 0.07127924665144843,
    -0.15709708597200958, 0.003787364476827695,
    0.13192943815825137, 0.060859631017042275,
]
SITE_LOW_START_TOLERANCE_RAD = 0.01
SITE_LOW_FLOOR_X_M = 0.02
SITE_LOW_EXIT_J3_RAD = -0.30
SITE_LOW_EXIT_J1_RAD = 0.0
SITE_LOW_EXIT_J2_RAD = -0.30


def _load_config(path):
    from experiments.single_arm.teaching.teach_session import load_config
    return load_config(path)


def _validate_joints(joints, zero_radius):
    from experiments.single_arm.teaching.teach_session import validate_joints
    return validate_joints(joints, zero_radius)


def _validate_route_workspace(robot, route, config):
    from experiments.single_arm.teaching.teach_session import validate_route_workspace
    return validate_route_workspace(robot, route, config)


def flange_x_constraint(minimum_x_m, source):
    """仅为候选筛选表达一个显式法兰条件；来源文本不是执行批准。"""
    return _checked_constraint({
        "minimum_x_m": minimum_x_m, "frame": "robot_base",
        "point": "flange_center", "unit": "m",
        "scope": "single_arm_s1_return_candidate", "source": source,
    })


def _checked_constraint(condition):
    if condition is None:
        return None
    required = {"minimum_x_m", "frame", "point", "unit", "scope", "source"}
    if type(condition) is not dict or set(condition) != required:
        raise ValueError("法兰 X 候选条件须明确数值、基座坐标系、法兰中心、米、范围和来源")
    value = condition["minimum_x_m"]
    try:
        numeric_value = float(value) if type(value) in (int, float) else math.nan
    except OverflowError:
        numeric_value = math.nan
    if not math.isfinite(numeric_value):
        raise ValueError("法兰 X 候选条件的最小值须为有限米数；负数本身合法")
    if (condition["frame"], condition["point"], condition["unit"],
            condition["scope"]) != (
            "robot_base", "flange_center", "m", "single_arm_s1_return_candidate"):
        raise ValueError("法兰 X 候选条件的坐标系、作用点、单位或适用范围不符")
    if type(condition["source"]) is not str or not condition["source"].strip():
        raise ValueError("法兰 X 候选条件须有非空来源；来源文本不构成执行批准")
    return dict(condition, minimum_x_m=numeric_value, source=condition["source"].strip())


def require_return_execution_basis(config):
    """当前实验配置没有可核验的通用回 S1 路线许可。"""
    if config.get("planned_return_min_flange_x_m") is not None:
        raise ValueError("旧示教回位 X 数值只是历史假设，不能授予自动回位许可")
    raise ValueError("通用回 S1 的环境与路线依据尚未核定；候选可计算，自动执行/示教自动回位未就绪")


def _write_new(directory, prefix, data):
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def save_plan(plan):
    """独占新建计划文件，保留每次实时规划的来源。"""
    return _write_new(PLAN_DIR, "nero_planned_return_plan_", plan)


def _candidate_routes(start, target):
    """尝试直接到位或先调整肘部，再调整肩部与腕部。"""
    if max(abs(a-b) for a, b in zip(start, target)) <= 0.2:
        yield "direct", [list(start), list(target)]
    layouts = (
        ((3, 2), (1,), (0, 4, 5, 6)),
        ((3,), (2,), (1,), (0, 4, 5, 6)),
        ((3, 2, 6), (1,), (0, 4, 5)),
        ((3,), (1, 2), (0, 4, 5, 6)),
    )
    for layout in layouts:
        route = [list(start)]
        for group in layout:
            next_target = list(route[-1])
            for axis in group:
                next_target[axis] = target[axis]
            if max(abs(a-b) for a, b in zip(next_target, route[-1])) > 1e-5:
                route.append(next_target)
        if route[-1] == list(target):
            yield "-".join("".join(str(axis+1) for axis in group) for group in layout), route


def _subdivide_long_segments(route):
    """只拆超过单目标距离上限的段；每个中间点仍由控制器完整执行。"""
    result = [route[0]]
    for start, target in zip(route, route[1:]):
        parts = math.ceil(math.dist(start, target) /
                          MAX_CONTROLLER_TARGET_DISTANCE_RAD)
        for index in range(1, parts+1):
            fraction = index/parts
            result.append(list(target) if index == parts else
                          [a+(b-a)*fraction for a,b in zip(start,target)])
    return result


def _assess(robot, route, config, constraint):
    """逐段抽查关节盒和插值；旧专用路线仍传入各自的数值门槛。"""
    # _assess 还被 H0 等既有专用实验路线直接调用。它们的数值只用于原
    # 路线的有限采样检查；通用 plan_from_snapshot 始终经 _checked_constraint。
    if type(constraint) in (int, float):
        try:
            minimum_x_m = float(constraint)
        except OverflowError as exc:
            raise ValueError("专用路线法兰 X 条件须为有限数值") from exc
        if not math.isfinite(minimum_x_m):
            raise ValueError("专用路线法兰 X 条件须为有限数值")
    elif constraint is None:
        minimum_x_m = None
    else:
        minimum_x_m = _checked_constraint(constraint)["minimum_x_m"]
    ranges = []
    for start, target in zip(route, route[1:]):
        if math.dist(start, target) > 1.05:
            raise ValueError("单目标关节距离超过 1.05 rad")
        changed = [index for index in range(7) if abs(start[index]-target[index]) > 1e-6]
        xyz = []
        for flags in product((0, 1), repeat=len(changed)):
            point = list(start)
            for axis, flag in zip(changed, flags):
                if flag:
                    point[axis] = target[axis]
            _validate_joints(point, config["zero_exclusion_radius_rad"])
            xyz.append([float(v) for v in robot.fk(point)[:3]])
        for step in range(101):
            fraction = step / 100
            point = [a+(b-a)*fraction for a, b in zip(start, target)]
            _validate_joints(point, config["zero_exclusion_radius_rad"])
            xyz.append([float(v) for v in robot.fk(point)[:3]])
        bounds = {axis: [min(p[i] for p in xyz), max(p[i] for p in xyz)]
                  for i, axis in enumerate("xyz")}
        if not all(math.isfinite(v) for pair in bounds.values() for v in pair):
            raise ValueError("正运动学输出无效")
        if minimum_x_m is not None and bounds["x"][0] < minimum_x_m:
            raise ValueError(
                f"候选法兰 X 最小采样值 {bounds['x'][0]:.3f} m 低于显式条件 "
                f"{minimum_x_m:.3f} m；只说明该条件不符合")
        ranges.append(bounds)
    _validate_route_workspace(robot, route, config)
    return ranges


def site_low_stage(state):
    """Return the next supervised trial stage for one measured neighborhood."""
    if (state.get("control_mode") != 1 or state.get("arm_status") != 0 or
            state.get("teach_status") != 0 or
            state.get("ready_for_motion") is not True):
        return None
    q = state.get("joint_rad", [])
    pose = state.get("flange_pose_m_rad", [])
    if len(q) != 7 or len(pose) < 3:
        return None
    references = [list(SITE_LOW_REFERENCE_Q)]
    for axis, target in ((2, SITE_LOW_EXIT_J3_RAD),
                         (0, SITE_LOW_EXIT_J1_RAD)):
        next_q = list(references[-1])
        next_q[axis] = target
        references.append(next_q)
    pose_boxes = (
        ((0.025, 0.055), (-0.030, 0.005)),
        ((0.020, 0.045), (-0.040, -0.015)),
        ((0.015, 0.040), (-0.045, -0.025)),
    )
    for index, (reference, (x_box, y_box)) in enumerate(zip(
            references, pose_boxes)):
        if (max(abs(a-b) for a,b in zip(q, reference)) <=
                SITE_LOW_START_TOLERANCE_RAD and
                x_box[0] <= float(pose[0]) <= x_box[1] and
                y_box[0] <= float(pose[1]) <= y_box[1] and
                0.700 <= float(pose[2]) <= 0.735):
            return index
    return None


def is_site_low_start(state):
    return site_low_stage(state) is not None


def _plan_site_low_exit(robot, state, config, constraint):
    """仅对已记录的窄起点计算受监护低位试验前缀。"""
    stage = site_low_stage(state)
    if config["mount"] != "left" or stage is None:
        raise ValueError("非 H0 低位姿态不在本次现场恢复起点范围")
    start = list(state["joint_rad"])
    prefix = [start]
    for axis,target in ((2,SITE_LOW_EXIT_J3_RAD),
                        (0,SITE_LOW_EXIT_J1_RAD),
                        (1,SITE_LOW_EXIT_J2_RAD))[stage:]:
        next_q = list(prefix[-1])
        next_q[axis] = target
        prefix.append(next_q)
    prefix_constraint = flange_x_constraint(
        SITE_LOW_FLOOR_X_M, "2026-09-27 narrow low-pose trial prefix")
    ranges = _assess(robot, prefix, config, prefix_constraint)
    if constraint is not None:
        # 显式的候选条件也须覆盖低位前缀，不能只验证后段。
        _assess(robot, prefix, config, constraint)
    # In the two low-X retreat stages, each single-axis command must move the
    # predicted flange away from the human-side table edge in base -Y.
    for first, second in zip(prefix[:max(0,3-stage)],
                             prefix[1:max(0,3-stage)]):
        previous_y = float(robot.fk(first)[1])
        for index in range(1, 102):
            fraction = index / 101
            point = [a + fraction*(b-a) for a,b in zip(first, second)]
            y = float(robot.fk(point)[1])
            if y > previous_y + 0.0001:
                raise ValueError("低位首段没有持续沿 Y 负向离桌")
            previous_y = y
    if float(robot.fk(prefix[-2])[1]) > -0.034:
        raise ValueError("低位退让未达到该试验前缀的 Y 方向候选条件")
    simulated = dict(state, joint_rad=prefix[-1],
                     flange_pose_m_rad=list(robot.fk(prefix[-1])))
    suffix = plan_from_snapshot(robot, simulated, config, constraint)
    return {"strategy": "site_low_exit+" + suffix["strategy"],
            "start_joint_rad": start,
            "target_joint_rad": suffix["target_joint_rad"],
            "route_joint_rad": prefix + suffix["route_joint_rad"][1:],
            "joint_box_flange_ranges_m": ranges + suffix["joint_box_flange_ranges_m"],
            "flange_x_constraint": constraint,
            "flange_x_check": suffix["flange_x_check"],
            "site_prefix_check": "sampled_trial_condition_only",
            "environment_assessment": "not_evaluated",
            "execution_status": "candidate_only",
            "site_low_pose_trial": True,
            "site_low_stage": stage,
            "site_trial_first_target_only": True,
            "site_trial_next_target_joint_rad": prefix[1],
            "site_clearance_confirmed": False,
            "controller_targets": len(prefix)-1 + suffix["controller_targets"]}


def plan_from_snapshot(robot, state, config, flange_x_constraint=None,
                       *, allow_site_low_pose=False):
    """对当前七轴状态生成候选；法兰筛选不构成路线执行许可。"""
    constraint = _checked_constraint(flange_x_constraint)
    require_return_state(state)
    start = _validate_joints(state["joint_rad"], config["zero_exclusion_radius_rad"])
    target = _validate_joints(config["safe_start_joint_rad"],
                             config["zero_exclusion_radius_rad"])
    pose = state.get("flange_pose_m_rad")
    try:
        pose_valid = (type(pose) in (list, tuple) and len(pose) == 6 and
                      all(type(value) in (int, float) and math.isfinite(float(value))
                          for value in pose))
    except OverflowError:
        pose_valid = False
    if not pose_valid:
        raise ValueError("当前法兰反馈缺失或坐标不是有限数值")
    start_fk = robot.fk(start)
    try:
        fk_valid = (len(start_fk) >= 3 and
                    all(type(value) in (int, float) and math.isfinite(float(value))
                        for value in start_fk[:3]))
    except OverflowError:
        fk_valid = False
    if not fk_valid:
        raise ValueError("当前起点 FK 坐标无效")
    if math.dist(start_fk[:3], pose[:3]) > 0.02:
        raise ValueError("当前法兰反馈与 FK 不一致")
    if allow_site_low_pose and site_low_stage(state) is not None:
        return _plan_site_low_exit(robot, state, config, constraint)
    start_x_m = float(pose[0])
    if constraint is not None and start_x_m < constraint["minimum_x_m"]:
        raise ValueError(
            f"当前法兰 X={start_x_m:.3f} m 低于显式候选条件 "
            f"{constraint['minimum_x_m']:.3f} m；只拒绝该候选筛选，不判定碰撞"
        )
    if max(abs(a-b) for a, b in zip(start, target)) <= TARGET_TOLERANCE_RAD:
        return {"strategy": "already_at_s1", "start_joint_rad": start,
                "target_joint_rad": target, "route_joint_rad": [start],
                "joint_box_flange_ranges_m": [],
                "flange_x_constraint": constraint,
                "flange_x_check": "not_evaluated" if constraint is None else "start_point_only",
                "environment_assessment": "not_evaluated",
                "execution_status": "candidate_only",
                "controller_targets": 0}
    candidates = []
    rejected = []
    for name, coarse_route in _candidate_routes(start, target):
        route = _subdivide_long_segments(coarse_route)
        try:
            ranges = _assess(robot, route, config, constraint)
        except ValueError as exc:
            rejected.append(f"{name}: {exc}")
            continue
        # 只用目标数和稳定名称排序；X 方向没有通用安全含义。
        candidates.append((name, route, ranges))
    if not candidates:
        raise ValueError(
            "候选路线未通过已执行的有限检查；未发送运动指令：" +
            "；".join(rejected)
        )
    name, route, ranges = min(candidates, key=lambda item: (len(item[1]), item[0]))
    return {"strategy": name, "start_joint_rad": start,
            "target_joint_rad": target, "route_joint_rad": route,
            "joint_box_flange_ranges_m": ranges,
            "flange_x_constraint": constraint,
            "flange_x_check": "not_evaluated" if constraint is None else "sampled_pass",
            "environment_assessment": "not_evaluated",
            "execution_status": "candidate_only",
            "controller_targets": len(route)-1}


def require_return_state(state):
    """允许空闲示教模式规划，但把阻止回位的实时原因逐项指出。"""
    reasons = []
    if state["control_mode"] not in (1, 2):
        reasons.append(f"控制模式 {state['control_mode']} 不是 CAN/空闲示教模式")
    if state["arm_status"] != 0 or state["error_code"] != 0:
        reasons.append(f"整机状态 {state['arm_status']} 或错误码 0x{state['error_code']:04X} 异常")
    if state["teach_status"] not in (0, 2, 6):
        reasons.append(f"示教状态 {state['teach_status']} 尚未退出")
    if state["joint_variation_0_25s_rad"] > 0.002:
        reasons.append("0.25 秒最大关节变化 "
                       f"{state['joint_variation_0_25s_rad']:.6f} rad 超过 0.002 rad")
    if len(state["drivers"]) != 7:
        reasons.append("驱动反馈不是七轴")
    else:
        for index, driver in enumerate(state["drivers"], 1):
            if not driver["enabled"] or driver["undervoltage"] or driver["driver_error"]:
                reasons.append(f"关节 {index} 失能、欠压或驱动故障")
    if reasons:
        raise RuntimeError("回位实时预检未通过：" + "；".join(reasons))
    return state


def prepare(config_path=DEFAULT_CONFIG, flange_x_constraint=None, robot_factory=None,
            *, allow_site_low_pose=False):
    constraint = _checked_constraint(flange_x_constraint)
    config_path = Path(config_path)
    config = _load_config(config_path)
    if config["mount"] != "left":
        raise ValueError("当前现场路线只适用于左侧安装")
    from experiments.single_arm.lab.device import connected, read_state
    with connected(config, robot_factory) as robot:
        state = read_state(robot)
        route = plan_from_snapshot(robot, state, config, constraint,
                                   allow_site_low_pose=allow_site_low_pose)
    return {"schema_version": 2, "kind": "live_planned_return_to_s1",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "start_state": state, "target_name": "S1",
            "speed_percent": 2 if route.get("site_low_pose_trial") else 5,
            "source_recording": None, "reversed_recording": False,
            "environment_checked": False, "controller_actual_path_known": False,
            **route}


def run(plan_path, config_path=DEFAULT_CONFIG, robot_factory=None,
        countdown_s=5, *, site_clearance_confirmed=False,
        complete_site_route=False):
    """旧计划不可自动升级；新候选没有环境/路线执行许可时零控制拒绝。"""
    plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    if plan.get("schema_version") == 1 and plan.get("kind") == "live_planned_return_to_s1":
        raise ValueError("历史回位计划包含未获通用批准的 X 假设；不得按新语义直接执行")
    if (plan.get("schema_version") != 2 or
            plan.get("kind") != "live_planned_return_to_s1" or
            plan.get("execution_status") != "candidate_only"):
        raise ValueError("回位计划版本或候选状态不明；未发送控制指令")
    raise ValueError(
        "回 S1 候选尚缺经核定的环境与整条路线适用证据；"
        "现场确认或计划内自声明不能代替执行许可，未发送控制指令"
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--min-flange-x-m", type=float,
                        help="仅作候选筛选；须同时给出 --flange-x-source")
    parser.add_argument("--flange-x-source", help="候选 X 条件的来源；不是执行批准")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.run:
            if args.plan is None:
                parser.error("执行时需指定 --plan")
            run(args.plan, args.config)
        else:
            if (args.min_flange_x_m is None) != (args.flange_x_source is None):
                raise ValueError("候选法兰 X 数值与来源须同时提供")
            constraint = (None if args.min_flange_x_m is None else
                          flange_x_constraint(args.min_flange_x_m, args.flange_x_source))
            plan = prepare(args.config, constraint)
            path = save_plan(plan)
            print("只读回 S1 新规划：", path)
            print("策略：", plan["strategy"], "；控制器目标数：", plan["controller_targets"])
            print("当前七轴：", plan["start_joint_rad"])
            for index, box in enumerate(plan["joint_box_flange_ranges_m"], 1):
                print(f"目标 {index} 法兰关节盒预测 (m)：", box)
            print("候选状态：", plan["execution_status"],
                  "；环境评估：", plan["environment_assessment"])
            print("不使用示教倒序；未发送运动指令，也未授予执行许可。")
        return 0
    except (OSError, RuntimeError, ValueError, TimeoutError, InterruptedError) as exc:
        print(f"新规划回位未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
