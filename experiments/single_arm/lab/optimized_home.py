"""Search a shorter sparse move_j route between S1 and the supported home candidate.

The search keeps the empirically required high, negative-Y detour and a
single-J1 table-edge approach/release. It minimizes unnecessary joint travel
among candidate elbow bends while auditing joint limits, asynchronous joint
corners, and a site-specific flange corridor. Physical obstacles and cables
are not represented; this is a monitored trial and it never disables joints.
"""

import argparse
from datetime import datetime, timezone
from itertools import product
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.planned_return import _assess, _subdivide_long_segments, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


HOME_Q = [0.23, 0.31084313978019007, -0.028082347664588763,
          -0.05707226654021458, 1.6599302916942469,
          0.13151055913777274, 0.592539281052075]
REPORT_DIR = Path("experiments/single_arm/lab/data/optimized_home")
LOW_X_M = 0.10
# The successful older route remained at or below -0.0325 m whenever its
# flange was within X<0.10 m, except the final light-contact leg. Requiring
# -0.034 m keeps at least that modeled margin in the newly generated route.
MAX_LOW_X_Y_M = -0.034
J1_OUTWARD = (0.40, 0.45, 0.50, 0.55, 0.60)
J4_BENDS = (-0.25, -0.30, -0.35, -0.40)
# With J3 held near H0, the shoulder sweep reaches Y about +0.05 m while
# flange X is below 0.10 m. The bend is a table-edge detour, not a gratuitous
# return. The reduced-j3 strategy trades earlier J5/J7 motion for less bend.
J3_BENDS = (-0.70, -0.80, -0.90, -1.00)
ROUTE_STRATEGIES = ("validated", "reduced-j3")
DEFAULT_STRATEGY = "reduced-j3"
# The validated route's joint-box minimum flange Z is about 0.560 m.
# A new candidate must not descend below this modeled envelope.
REDUCED_J3_MIN_FLANGE_Z_M = 0.559


def _low_x_worst_y(robot, route_without_contact):
    worst = -math.inf
    for start, target in zip(route_without_contact, route_without_contact[1:]):
        changed = [i for i in range(7) if abs(start[i]-target[i]) > 1e-6]
        samples = []
        for flags in product((0, 1), repeat=len(changed)):
            q = list(start)
            for axis, flag in zip(changed, flags):
                if flag:
                    q[axis] = target[axis]
            samples.append(q)
        for step in range(101):
            fraction = step / 100
            samples.append([a+fraction*(b-a) for a,b in zip(start,target)])
        for q in samples:
            x, y = robot.fk(q)[:2]
            if x < LOW_X_M:
                worst = max(worst, y)
    return worst


def _nominal_home_route(robot, config):
    s1 = list(config["safe_start_joint_rad"])
    high_y = list(s1)
    high_y[0] = -0.2
    valid = []
    for j1, j4, j3 in product(J1_OUTWARD, J4_BENDS, J3_BENDS):
        outward = list(HOME_Q)
        outward[0] = j1
        bent = list(outward)
        bent[2], bent[3] = j3, j4
        shoulder_at_s1 = list(bent)
        shoulder_at_s1[1] = s1[1]
        route = _subdivide_long_segments([
            s1, high_y, shoulder_at_s1, bent, outward, list(HOME_Q),
        ])
        try:
            _assess(robot, route, config, -0.19)
        except ValueError:
            continue
        worst = _low_x_worst_y(robot, route[:-1])
        if worst > MAX_LOW_X_Y_M:
            continue
        if (robot.fk(route[1])[0] < 0.3 or robot.fk(route[1])[1] > -0.08 or
                max(abs(a-b) for a,b in zip(route[-2][1:], route[-1][1:])) > 1e-9):
            continue
        distance = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
        cost = distance + 0.05*(len(route)-1)
        valid.append((cost, distance, len(route)-1, j1, j4, j3, worst, route))
    if not valid:
        raise ValueError("未找到满足桌边模型余量的稀疏回家路线")
    best = min(valid, key=lambda item: item[:6])
    return {"cost": best[0], "joint_path_length_rad": best[1],
            "targets": best[2], "j1_outward_rad": best[3],
            "j4_bend_rad": best[4], "j3_bend_rad": best[5],
            "worst_low_x_y_m": best[6], "valid_candidates": len(valid),
            "home_route_joint_rad": best[7]}


def _reduced_j3_home_route(robot, config):
    """Search a lower J3 excursion while retaining the validated corridor."""
    s1 = list(config["safe_start_joint_rad"])
    high_y = list(s1)
    high_y[0] = -0.2
    valid = []
    for j1, j4, j3, j5 in product(
            (0.40, 0.45, 0.50), (-0.30, -0.35, -0.40),
            (-0.60, -0.70, -0.80), (0.50, 1.00)):
        outward = list(HOME_Q)
        outward[0] = j1
        bent = list(outward)
        bent[2], bent[3], bent[4], bent[6] = j3, j4, j5, s1[6]
        shoulder_at_s1 = list(bent)
        shoulder_at_s1[1] = s1[1]
        route = _subdivide_long_segments([
            s1, high_y, shoulder_at_s1, bent, outward, list(HOME_Q),
        ])
        try:
            boxes = _assess(robot, route, config, -0.19)
        except ValueError:
            continue
        min_z = min(box["z"][0] for box in boxes)
        if min_z < REDUCED_J3_MIN_FLANGE_Z_M:
            continue
        worst = _low_x_worst_y(robot, route[:-1])
        if worst > MAX_LOW_X_Y_M:
            continue
        if (robot.fk(route[1])[0] < 0.3 or robot.fk(route[1])[1] > -0.08 or
                max(abs(a-b) for a,b in zip(route[-2][1:],route[-1][1:])) > 1e-9):
            continue
        distance = sum(math.dist(a,b) for a,b in zip(route,route[1:]))
        cost = distance + 0.05*(len(route)-1)
        valid.append((cost, distance, len(route)-1, j1, j4, j3,
                      j5, worst, min_z, route))
    if not valid:
        raise ValueError("未找到同时减少 J3 转动并维持桌边和高度余量的路线")
    best = min(valid, key=lambda item: item[:7])
    return {"cost": best[0], "joint_path_length_rad": best[1],
            "targets": best[2], "j1_outward_rad": best[3],
            "j4_bend_rad": best[4], "j3_bend_rad": best[5],
            "j5_early_rad": best[6], "j7_early_rad": s1[6],
            "worst_low_x_y_m": best[7], "min_flange_z_m": best[8],
            "valid_candidates": len(valid), "home_route_joint_rad": best[9]}


def plan(robot, state, config, direction, *, strategy=DEFAULT_STRATEGY):
    require_ready(state)
    if (config["mount"] != "left" or direction not in ("start", "home") or
            strategy not in ROUTE_STRATEGIES):
        raise ValueError("仅适用于左侧安装的启动/回家")
    expected = HOME_Q if direction == "start" else config["safe_start_joint_rad"]
    actual = list(state["joint_rad"])
    if max(abs(a-b) for a,b in zip(actual,expected)) > 0.01:
        raise ValueError("当前姿态不在本次路线的命名起点")
    if math.dist(robot.fk(actual)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    chosen = (_nominal_home_route(robot, config) if strategy == "validated"
              else _reduced_j3_home_route(robot, config))
    nominal = chosen["home_route_joint_rad"]
    route = list(reversed(nominal)) if direction == "start" else list(nominal)
    route[0] = actual
    if direction == "start":
        # Preserve the live non-J1 axes for the physical edge-release step.
        # Encoder noise around the calibrated target must not command a
        # multi-axis motion while the flange is touching the table.
        outward_j1 = route[1][0]
        route[1] = list(actual)
        route[1][0] = outward_j1
    boxes = _assess(robot, route, config, -0.19)
    if direction == "start":
        before, after = robot.fk(route[0]), robot.fk(route[1])
        if (after[1] > before[1] - 0.015 or
                after[0] < before[0] + 0.004 or
                max(abs(a-b) for a,b in zip(route[0][1:],route[1][1:])) > 1e-9):
            raise ValueError("起步未先以单 J1 沿 Y 负向离桌")
        worst = _low_x_worst_y(robot, list(reversed(route))[:-1])
    else:
        if robot.fk(route[1])[1] > -0.08:
            raise ValueError("高位首段未沿 Y 负向绕开桌边")
        worst = _low_x_worst_y(robot, route[:-1])
    if worst > MAX_LOW_X_Y_M:
        raise ValueError("实时路线的桌边余量未通过")
    return {"direction": direction, "strategy": strategy,
            "route_joint_rad": route,
            "joint_box_flange_ranges_m": boxes,
            "search": {k:v for k,v in chosen.items() if k != "home_route_joint_rad"},
            "automatic_disable": False}


def plan_resume(robot, state, config, source):
    """Resume only a paused, unchanged home route at its last reached target."""
    require_ready(state)
    if (config["mount"] != "left" or
            source.get("kind") != "optimized_sparse_s1_h0_trial" or
            source.get("direction") != "home" or
            source.get("strategy") not in ROUTE_STRATEGIES or
            source.get("completed") or source.get("emergency_stop_requested") or
            source.get("error_type") != "RoutePaused" or
            "机械臂仍停在上一小步附近" not in source.get("error", "")):
        raise ValueError("仅可续行未急停、确认停在上一目标的 H0 回零报告")
    chosen = (_nominal_home_route(robot, config)
              if source["strategy"] == "validated"
              else _reduced_j3_home_route(robot, config))
    nominal = chosen["home_route_joint_rad"]
    recorded = source.get("full_route_joint_rad",
                          source.get("route_joint_rad", []))
    local_route = source.get("route_joint_rad", [])
    completed = source.get("completed_targets", [])
    previous_offset = source.get("resumed_after_target", 0)
    if not isinstance(previous_offset, int) or previous_offset < 0:
        raise ValueError("原报告的续行索引无效")
    count = previous_offset + len(completed)

    def same_target(a, b, tolerance=1e-6):
        return (isinstance(a, list) and len(a) == len(b) == 7 and
                max(abs(x-y) for x,y in zip(a,b)) <= tolerance)

    if (len(recorded) != len(nominal) or
            len(local_route) != len(nominal)-previous_offset or
            not 0 <= count < len(nominal)-1 or
            any(not same_target(recorded[i], nominal[i])
                for i in range(1, len(nominal))) or
            any(not same_target(local_route[i], recorded[previous_offset+i])
                for i in range(1, len(local_route))) or
            any(item.get("index") != i or
                not same_target(item.get("joint_rad"), local_route[i])
                for i, item in enumerate(completed, 1))):
        raise ValueError("原报告的目标序列与当前标定不一致，不能续行")
    actual = list(state["joint_rad"])
    if (not same_target(actual, recorded[count], 0.003) or
            math.dist(robot.fk(actual)[:3], state["flange_pose_m_rad"][:3]) > 0.005):
        raise ValueError("实时姿态不在原报告最后到位目标，不能续行")
    route = [actual, *recorded[count+1:]]
    boxes = _assess(robot, route, config, -0.19)
    if _low_x_worst_y(robot, route[:-1]) > MAX_LOW_X_Y_M:
        raise ValueError("剩余路线的桌边余量未通过")
    return {"direction": "home", "strategy": source["strategy"],
            "route_joint_rad": route, "joint_box_flange_ranges_m": boxes,
            "search": {k:v for k,v in chosen.items() if k != "home_route_joint_rad"},
            "full_route_joint_rad": recorded,
            "resumed_after_target": count, "automatic_disable": False}


def _read_preflight_state(robot, report, *, label, expected_joint=None):
    """Recheck only a transient stability reading, never a controller fault."""
    first_joint = None
    for attempt in range(3):
        state = read_state(robot)
        report.setdefault("preflight_state_attempts", []).append({
            "label": label, "attempt": attempt + 1,
            "joint_rad": state["joint_rad"],
            "variation_rad": state["joint_variation_0_25s_rad"],
            "reasons": state["reasons"],
        })
        reference = expected_joint if expected_joint is not None else first_joint
        if reference is not None and max(abs(a-b) for a,b in zip(
                reference, state["joint_rad"])) > 0.002:
            raise RuntimeError("预检复测期间姿态变化超过 0.002 rad；未发送运动指令")
        if state["ready_for_motion"]:
            return state
        if state["reasons"] != ["七轴反馈尚未停稳"]:
            return state
        if first_joint is None:
            first_joint = list(state["joint_rad"])
        if attempt < 2:
            time.sleep(0.15)
    return state


def run(direction, *, execute=False, countdown_s=5,
        config_path=DEFAULT_CONFIG, robot_factory=None,
        strategy=DEFAULT_STRATEGY, resume_report=None):
    config = load_config(config_path)
    source = None
    if resume_report is not None:
        source = json.loads(Path(resume_report).read_text(encoding="utf-8"))
        if direction != "home" or source.get("strategy") != strategy:
            raise ValueError("续行报告必须与 home 方向和路线策略一致")
    report = {"kind": "optimized_sparse_s1_h0_trial", "direction": direction,
              "strategy": strategy,
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "execution_requested": execute, "completed": False,
              "completed_targets": [], "emergency_stop_requested": False,
              "precommand_feedback_retries": [],
              "stationary_retries": [],
              "resume_source_report": str(resume_report) if source is not None else None,
              "automatic_disable": False}
    path = _write_new(REPORT_DIR, "nero_optimized_home_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            state = _read_preflight_state(robot, report, label="initial")
            planned = (plan_resume(robot, state, config, source) if source is not None
                       else plan(robot, state, config, direction, strategy=strategy))
            report.update(start_state=state, preflight_passed=True, **planned)
            route = planned["route_joint_rad"]
            remaining_distance = sum(math.dist(a, b) for a, b in
                                     zip(route, route[1:]))
            print(f"{direction}：程序搜索得到 {len(route)-1} 个 move_j 目标；"
                  f"本次剩余关节行程 {remaining_distance:.3f} rad。",
                  flush=True)
            if source is not None:
                print(f"从原报告已完成的第 {planned['resumed_after_target']} 个目标续行。",
                      flush=True)
            print("搜索参数/低位 Y 最靠桌值：",
                  {k: planned["search"][k] for k in (
                      "j1_outward_rad", "j4_bend_rad", "j3_bend_rad",
                      "worst_low_x_y_m")}, flush=True)
            if strategy == "reduced-j3":
                print("减少 J3 候选：J5/J7 提前到",
                      planned["search"]["j5_early_rad"],
                      planned["search"]["j7_early_rad"],
                      "rad；预测最小法兰 Z",
                      planned["search"]["min_flange_z_m"], "m。",
                      flush=True)
            if not execute:
                print("只读预检完成；未发送运动或失能指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后执行缩短路线；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live_state = _read_preflight_state(
                robot, report, label="after_countdown", expected_joint=route[0])
            live = (plan_resume(robot, live_state, config, source) if source is not None
                    else plan(robot, live_state, config, direction,
                              strategy=strategy))
            if max(abs(a-b) for a,b in zip(route[0],
                                            live["route_joint_rad"][0])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            route = live["route_joint_rad"]
            report.update(route_joint_rad=route,
                          joint_box_flange_ranges_m=live["joint_box_flange_ranges_m"])
            for index in range(1, len(route)):
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                speed = 2 if ((direction == "start" and index == 1) or
                              (direction == "home" and index == len(route)-1)) else 5
                for retry in range(2):
                    command_attempted = False

                    def mark_command(_i, _target):
                        nonlocal command_attempted
                        command_attempted = True

                    try:
                        execute_route(
                            robot, config, route[index-1:index+1],
                            lambda: interrupted, speed_percent=speed,
                            target_tolerance_rad=0.005, waypoint_timeout_s=20,
                            waypoint_max_timeout_s=120,
                            pause_on_stationary_timeout=True,
                            path_mode="joint_box", joint_box_margin_rad=0.03,
                            on_command=mark_command,
                            on_waypoint=lambda _i, target: report["completed_targets"].append(
                                {"index": index, "joint_rad": target,
                                 "speed_percent": speed}),
                            on_emergency_stop=lambda reason: report.update(
                                emergency_stop_requested=True,
                                emergency_stop_reason=reason),
                            stationary_retry_limit=1 if speed == 5 else 0,
                            on_stationary_retry=lambda _i, attempt: report[
                                "stationary_retries"].append({
                                    "target_index": index + planned.get(
                                        "resumed_after_target", 0),
                                    "attempt": attempt}),
                        )
                        break
                    except RuntimeError as exc:
                        if (retry or command_attempted or interrupted or
                                str(exc) != "关节角反馈缺失或过期"):
                            raise
                        stable = require_ready(read_state(robot))
                        if max(abs(a-b) for a,b in zip(
                                stable["joint_rad"], route[index-1])) > 0.003:
                            raise RuntimeError("反馈恢复后姿态已变化；停止路线") from exc
                        report["precommand_feedback_retries"].append(index)
                        print(f"第 {index} 目标发送前反馈短暂过期；姿态未变，重试一次。",
                              flush=True)
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                encoding="utf-8")
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a,b in zip(final["joint_rad"], route[-1]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("目标关节误差超限")
            report["completed"] = True
            print(f"缩短路线 {direction} 完成；{len(route)-1}/{len(route)-1} 目标到位；"
                  f"最大关节误差 {error:.6f} rad；保持使能。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"缩短路线未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("缩短路线报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--direction", choices=("start", "home"), required=True)
    parser.add_argument("--strategy", choices=ROUTE_STRATEGIES,
                        default=DEFAULT_STRATEGY)
    parser.add_argument("--resume-report", type=Path,
                        help="仅续行原报告中确认停在上一目标、未急停的 H0 回零")
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        run(args.direction, execute=args.run, strategy=args.strategy,
            resume_report=args.resume_report)
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
