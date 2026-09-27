"""Site-specific S1/H0 move_j trial with an outward base-Y detour.

The historical H0 target is retained, while its blocked X-first route is not
used. Stage 1 stops before table contact so the operator can inspect the gap.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints,
)


HOME_PATH = Path("experiments/single_arm/lab/config/supported_home.json")
REPORT_DIR = Path("experiments/single_arm/lab/data/two_point_y_detour")
OUTER_J1_RAD = 0.6
PRECONTACT_J1_RAD = 0.45
INTERMEDIATE_J2_RAD = 0.2
STAGES = ("to-h0-clearance", "to-h0-contact", "to-s1")
# The seven-target trial reached its endpoint without contact, but the operator
# rejected its ordering: the Y detour happened after the arm had descended.
# Retire the whole route, including its reverse, until a new high-clearance
# detour has been planned and checked against the actual table edge.
ROUTE_HOLD = True


def _forward(s1, h0):
    route = [list(s1)]
    for fraction in (0.5, 1.0):
        point = list(s1)
        for axis in (4, 6):
            point[axis] = s1[axis] + fraction * (h0[axis] - s1[axis])
        route.append(point)
    body = list(route[-1])
    for axis in (0, 2, 3, 5):
        body[axis] = h0[axis]
    route.append(body)
    mid = list(body)
    mid[1] = INTERMEDIATE_J2_RAD
    route.append(mid)
    outward = list(mid)
    outward[0] = OUTER_J1_RAD
    route.append(outward)
    low_outer = list(outward)
    low_outer[1] = h0[1]
    route.append(low_outer)
    precontact = list(low_outer)
    precontact[0] = PRECONTACT_J1_RAD
    route.append(precontact)
    route.append(list(h0))
    return route


def plan(robot, state, config, stage):
    require_ready(state)
    if config["mount"] != "left":
        raise ValueError("此现场路线仅适用于左侧安装")
    home_bytes = HOME_PATH.read_bytes()
    home = json.loads(home_bytes)
    if (home.get("name") != "H0" or home.get("route_blocked") is not True
            or home.get("config_sha256") != hashlib.sha256(
                Path(DEFAULT_CONFIG).read_bytes()).hexdigest()):
        raise ValueError("旧 H0 目标或其路线锁定状态与现场记录不符")
    s1 = validate_joints(config["safe_start_joint_rad"],
                         config["zero_exclusion_radius_rad"])
    h0 = validate_joints(home["joint_rad"], config["zero_exclusion_radius_rad"])
    forward = _forward(s1, h0)
    actual = list(state["joint_rad"])
    if math.dist(robot.fk(actual)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    if stage == "to-h0-clearance":
        expected = s1
        route = [actual, *forward[1:8]]
        floors = (0.38, 0.39, 0.35, -0.07, -0.07, -0.11, -0.12)
        batches = ((0, 4, 5), (4, 7, 2))
    elif stage == "to-h0-contact":
        expected = forward[7]
        route = [actual, h0]
        floors = (-0.13,)
        batches = ((0, 1, 2),)
    elif stage == "to-s1":
        expected = h0
        route = [actual, forward[6], forward[5], forward[4],
                 forward[3], forward[2], forward[1], s1]
        floors = (-0.13, -0.11, -0.07, -0.07, 0.35, 0.39, 0.38)
        batches = ((0, 1, 2), (1, 7, 5))
    else:
        raise ValueError("未知两点试验阶段")
    if max(abs(a - b) for a, b in zip(actual, expected)) > 0.005:
        raise ValueError("实时姿态不在本阶段起点附近")
    boxes = _assess(robot, route, config, min(floors))
    if any(box["x"][0] < floor for box, floor in zip(boxes, floors)):
        raise ValueError("候选关节盒越过本阶段离桌边界")
    return {"stage": stage, "route_joint_rad": route,
            "joint_box_flange_ranges_m": boxes, "speed_batches": batches,
            "home_config_sha256": hashlib.sha256(home_bytes).hexdigest()}


def run(stage, *, execute=False, config_path=DEFAULT_CONFIG, countdown_s=5):
    if ROUTE_HOLD:
        raise RuntimeError("现场否决了本路线的顺序：应先在高处沿 Y 负向绕开桌边，再下降；全部阶段已停用")
    config_path = Path(config_path)
    config = load_config(config_path)
    report = {"kind": "site_specific_s1_h0_y_detour_move_j",
              "stage": stage, "created_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "completed_targets": []}
    path = _write_new(REPORT_DIR, "nero_two_point_y_detour_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state = read_state(robot)
            planned = plan(robot, state, config, stage)
            report.update(start_state=state, **planned)
            route = planned["route_joint_rad"]
            print(f"{stage}：{len(route)-1} 个 move_j 目标；法兰起终点 XYZ：",
                  state["flange_pose_m_rad"][:3], robot.fk(route[-1])[:3], flush=True)
            print("各段预测法兰范围：", planned["joint_box_flange_ranges_m"], flush=True)
            if not execute:
                print("只读预检完成；未发送运动指令。", flush=True)
                return path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后执行 {stage}；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live = read_state(robot)
            live_plan = plan(robot, live, config, stage)
            if max(abs(a-b) for a, b in zip(
                    route[0], live_plan["route_joint_rad"][0])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            route[0] = live_plan["route_joint_rad"][0]
            for start, stop_index, speed in planned["speed_batches"]:
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                execute_route(
                    robot, config, route[start:stop_index+1],
                    lambda: interrupted, speed_percent=speed,
                    target_tolerance_rad=0.005, waypoint_timeout_s=20,
                    waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                    path_mode="joint_box", joint_box_margin_rad=0.03,
                    on_waypoint=lambda index, target: report["completed_targets"].append({
                        "index": start + index, "joint_rad": target,
                        "speed_percent": speed}),
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason))
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                encoding="utf-8")
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], route[-1]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("目标关节误差超限")
            report["completed"] = True
            print(f"{stage} 完成；{len(route)-1}/{len(route)-1} 目标到位，"
                  f"最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"两点试验未完成：{exc}；报告 {path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                        encoding="utf-8")
        print("两点试验报告：", path, flush=True)
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGES, required=True)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        run(args.stage, execute=args.run)
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
