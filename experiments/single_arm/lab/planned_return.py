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
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints, validate_route_workspace,
)


PLAN_DIR = Path("experiments/single_arm/lab/data/planned_returns")
RUN_DIR = Path("experiments/single_arm/lab/data/planned_return_runs")
START_TOLERANCE_RAD = 0.003
TARGET_TOLERANCE_RAD = 0.005
MAX_CONTROLLER_TARGET_DISTANCE_RAD = 1.0


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


def _assess(robot, route, config, min_flange_x_m):
    """逐段抽查关节盒和插值；盒覆盖控制器各轴不同步的可能姿态。"""
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
            validate_joints(point, config["zero_exclusion_radius_rad"])
            xyz.append([float(v) for v in robot.fk(point)[:3]])
        for step in range(101):
            fraction = step / 100
            point = [a+(b-a)*fraction for a, b in zip(start, target)]
            validate_joints(point, config["zero_exclusion_radius_rad"])
            xyz.append([float(v) for v in robot.fk(point)[:3]])
        bounds = {axis: [min(p[i] for p in xyz), max(p[i] for p in xyz)]
                  for i, axis in enumerate("xyz")}
        if not all(math.isfinite(v) for pair in bounds.values() for v in pair):
            raise ValueError("正运动学输出无效")
        if bounds["x"][0] < min_flange_x_m:
            raise ValueError("候选关节盒可能进入已知桌面侧")
        ranges.append(bounds)
    validate_route_workspace(robot, route, config)
    return ranges


def plan_from_snapshot(robot, state, config, min_flange_x_m=0.15):
    """对当前七轴状态生成新路线；返回结果不含任何录制轨迹点。"""
    require_return_state(state)
    start = validate_joints(state["joint_rad"], config["zero_exclusion_radius_rad"])
    target = validate_joints(config["safe_start_joint_rad"],
                             config["zero_exclusion_radius_rad"])
    if math.dist(robot.fk(start)[:3], state["flange_pose_m_rad"][:3]) > 0.02:
        raise ValueError("当前法兰反馈与 FK 不一致")
    if min_flange_x_m < 0:
        raise ValueError("本现场左侧装仅允许基座 X 非负的回位路线")
    start_x_m = float(state["flange_pose_m_rad"][0])
    if start_x_m < min_flange_x_m:
        raise ValueError(
            f"当前法兰 X={start_x_m:.3f} m 低于通用回 S1 门槛 "
            f"{min_flange_x_m:.3f} m；此入口不能从桌边低位启动。"
            "若确认为已标定 H0，请按当前使能状态使用 H0 专用启动入口；"
            "其他低位须现场重新规划，未发送运动指令"
        )
    if max(abs(a-b) for a, b in zip(start, target)) <= TARGET_TOLERANCE_RAD:
        if float(state["flange_pose_m_rad"][0]) < min_flange_x_m:
            raise ValueError("已在 S1 关节附近，但法兰 X 不符合现场门槛")
        return {"strategy": "already_at_s1", "start_joint_rad": start,
                "target_joint_rad": target, "route_joint_rad": [start],
                "joint_box_flange_ranges_m": [],
                "min_flange_x_m": min_flange_x_m,
                "controller_targets": 0}
    candidates = []
    for name, coarse_route in _candidate_routes(start, target):
        route = _subdivide_long_segments(coarse_route)
        try:
            ranges = _assess(robot, route, config, min_flange_x_m)
        except ValueError:
            continue
        # 优先离桌间隙更大，随后减少控制器目标数。此分数不代表碰撞证明。
        candidates.append((min(box["x"][0] for box in ranges), -len(route),
                           name, route, ranges))
    if not candidates:
        raise ValueError(
            "当前姿态的候选路线均未通过关节限位或法兰 X 门槛；"
            "未发送运动指令。请保持当前位置，检查起点与现场环境，"
            "不要降低门槛或重跑旧计划"
        )
    _, _, name, route, ranges = max(candidates)
    return {"strategy": name, "start_joint_rad": start,
            "target_joint_rad": target, "route_joint_rad": route,
            "joint_box_flange_ranges_m": ranges,
            "min_flange_x_m": min_flange_x_m,
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


def prepare(config_path=DEFAULT_CONFIG, min_flange_x_m=0.15, robot_factory=None):
    config_path = Path(config_path)
    config = load_config(config_path)
    if config["mount"] != "left":
        raise ValueError("当前现场路线只适用于左侧安装")
    with connected(config, robot_factory) as robot:
        state = read_state(robot)
        route = plan_from_snapshot(robot, state, config, min_flange_x_m)
    return {"schema_version": 1, "kind": "live_planned_return_to_s1",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "start_state": state, "target_name": "S1", "speed_percent": 5,
            "source_recording": None, "reversed_recording": False,
            "environment_checked": False, "controller_actual_path_known": False,
            **route}


def run(plan_path, config_path=DEFAULT_CONFIG, robot_factory=None,
        countdown_s=5):
    plan_path = Path(plan_path)
    config_path = Path(config_path)
    config = load_config(config_path)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if (plan.get("schema_version") != 1 or
            plan.get("kind") != "live_planned_return_to_s1" or
            plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest() or
            plan.get("target_joint_rad") != config["safe_start_joint_rad"] or
            plan.get("speed_percent") != 5 or
            plan.get("source_recording") is not None or
            plan.get("reversed_recording") is not False):
        raise ValueError("回位计划版本、目标或配置不匹配")
    report = {"schema_version": 1, "kind": "live_planned_return_to_s1",
              "plan": str(plan_path), "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "controller_targets": plan["controller_targets"]}
    report_path = _write_new(RUN_DIR, "nero_planned_return_run_", report)
    interrupted = False
    previous_handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            state = read_state(robot)
            current = plan_from_snapshot(robot, state, config, plan["min_flange_x_m"])
            if max(abs(a-b) for a, b in zip(state["joint_rad"],
                                             plan["start_joint_rad"])) > START_TOLERANCE_RAD:
                raise RuntimeError("实时起点已变化；未发送运动指令")
            if (current["strategy"] != plan["strategy"] or
                    current["controller_targets"] != plan["controller_targets"] or
                    any(max(abs(a-b) for a, b in zip(x, y)) > 0.005
                        for x, y in zip(current["route_joint_rad"],
                                        plan["route_joint_rad"]))):
                raise RuntimeError("实时重新规划与保存路线不符；未发送运动指令")
            previous_handlers = (signal.signal(signal.SIGINT, stop),
                                 signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后沿新规划回 S1；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            if plan["controller_targets"]:
                report["motion_attempted"] = True
                execute_route(robot, config, plan["route_joint_rad"],
                              lambda: interrupted, speed_percent=5,
                              target_tolerance_rad=TARGET_TOLERANCE_RAD,
                              waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                              pause_on_stationary_timeout=True,
                              path_mode="joint_box", joint_box_margin_rad=0.03)
            else:
                report["motion_attempted"] = False
            final = read_state(robot)
            error = max(abs(a-b) for a, b in zip(final["joint_rad"],
                                                   config["safe_start_joint_rad"]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if not final["ready_for_motion"] or error > TARGET_TOLERANCE_RAD:
                raise RuntimeError("到位后独立状态或 S1 关节误差不合格")
            report["completed"] = True
    except Exception as exc:
        report.update(error=str(exc), error_type=type(exc).__name__)
        raise RuntimeError(f"新规划回 S1 未完成：{exc}；报告 {report_path}") from exc
    finally:
        if previous_handlers is not None:
            signal.signal(signal.SIGINT, previous_handlers[0])
            signal.signal(signal.SIGTERM, previous_handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
        print("回位报告：", report_path, flush=True)
    return report_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--min-flange-x-m", type=float, default=0.15)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.run:
            if args.plan is None:
                parser.error("执行时需指定 --plan")
            run(args.plan, args.config)
        else:
            plan = prepare(args.config, args.min_flange_x_m)
            path = save_plan(plan)
            print("只读回 S1 新规划：", path)
            print("策略：", plan["strategy"], "；控制器目标数：", plan["controller_targets"])
            print("当前七轴：", plan["start_joint_rad"])
            for index, box in enumerate(plan["joint_box_flange_ranges_m"], 1):
                print(f"目标 {index} 法兰关节盒预测 (m)：", box)
            print("不使用示教倒序；未发送运动指令。")
        return 0
    except (OSError, RuntimeError, ValueError, TimeoutError, InterruptedError) as exc:
        print(f"新规划回位未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
