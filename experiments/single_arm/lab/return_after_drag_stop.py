"""本次拖动回位故障后，从已确认的低位回 S1；默认只读。

只适用于 2026-09-26 本次姿态。先通过 J2 沿基座 X 正向离桌，
再到 S1 的 J2 角，最后用两个关节目标收拢腕部。全程仅四个
move_j 目标，由控制器处理每个目标之间的平滑运动。
"""

import argparse
from datetime import datetime, timezone
import hashlib
from itertools import product
import json
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints, validate_route_workspace,
)


OUTPUT_DIR = Path("experiments/single_arm/lab/data/drag_stop_returns")
REFERENCE = [0.080634, 0.115192, -0.323130, -0.166312,
             1.529432, -0.663347, 0.015341]


def _save(data, prefix):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUTPUT_DIR / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def _route(start, s1, config):
    start = validate_joints(start, config["zero_exclusion_radius_rad"])
    lift = list(start)
    lift[1] -= 0.05
    bend = list(lift)
    bend[1] = s1[1]
    wrist_middle = [(a + b) / 2 for a, b in zip(bend, s1)]
    return [validate_joints(point, config["zero_exclusion_radius_rad"])
            for point in (start, lift, bend, wrist_middle, s1)]


def _box_ranges(robot, route):
    ranges = []
    for first, last in zip(route, route[1:]):
        xyz = []
        for corner in product((0, 1), repeat=7):
            joints = [last[i] if corner[i] else first[i] for i in range(7)]
            xyz.append([float(x) for x in robot.fk(joints)[:3]])
        ranges.append({axis: [min(p[i] for p in xyz), max(p[i] for p in xyz)]
                       for i, axis in enumerate("xyz")})
    if ranges[0]["x"][0] < -0.015 or ranges[0]["x"][1] > 0.035:
        raise ValueError("离桌首段超出已复核 X 范围")
    if any(box["x"][0] < 0.015 for box in ranges[1:]):
        raise ValueError("后续阶段可能回到桌面低位")
    if any(box["y"][0] < -0.06 or box["y"][1] > 0.05 or
           box["z"][0] < 0.52 or box["z"][1] > 0.72 for box in ranges):
        raise ValueError("预测法兰范围超出本次现场复核窗")
    return ranges


def _prepare_live(robot, config):
    state = require_ready(read_state(robot))
    if config["mount"] != "left" or max(abs(a-b) for a, b in zip(
            state["joint_rad"], REFERENCE)) > 0.02:
        raise ValueError("当前姿态或安装方式与本次回位起点不符")
    route = _route(state["joint_rad"], config["safe_start_joint_rad"], config)
    validate_route_workspace(robot, route, config)
    boxes = _box_ranges(robot, route)
    return state, route, boxes


def prepare(config_path=DEFAULT_CONFIG):
    config_path = Path(config_path)
    config = load_config(config_path)
    with connected(config) as robot:
        state, route, boxes = _prepare_live(robot, config)
    return {"schema_version": 1, "kind": "one_time_drag_stop_return",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "start_joint_rad": state["joint_rad"], "route_joint_rad": route,
            "joint_box_flange_ranges_m": boxes, "speed_percent": 5,
            "controller_targets": 4,
            "site_note": "左侧安装、裸法兰。现场检查桌面、腕部、线缆和支撑。"}


def run(plan_path, config_path=DEFAULT_CONFIG):
    plan_path = Path(plan_path)
    config_path = Path(config_path)
    config = load_config(config_path)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if (plan.get("kind") != "one_time_drag_stop_return" or
            plan.get("schema_version") != 1 or plan.get("speed_percent") != 5 or
            plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest() or
            plan.get("route_joint_rad") != _route(plan["start_joint_rad"],
                                                   config["safe_start_joint_rad"], config)):
        raise ValueError("回位计划与本次配置或路线不符")
    report = {"schema_version": 1, "kind": "one_time_drag_stop_return",
              "plan": str(plan_path), "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False}
    report_path = _save(report, "nero_drag_stop_return_run_")
    interrupted = False
    previous_handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state, _live_route, boxes = _prepare_live(robot, config)
            if max(abs(a-b) for a, b in zip(state["joint_rad"],
                                             plan["start_joint_rad"])) > 0.002:
                raise RuntimeError("实时起点偏离计划；未发送运动指令")
            for live, planned in zip(boxes, plan["joint_box_flange_ranges_m"]):
                if any(max(abs(a-b) for a, b in zip(live[axis], planned[axis])) > 0.005
                       for axis in "xyz"):
                    raise RuntimeError("实时几何与计划不同；未发送运动指令")
            previous_handlers = (signal.signal(signal.SIGINT, stop),
                                 signal.signal(signal.SIGTERM, stop))
            for remaining in range(5, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消")
                print(f"{remaining} 秒后回 S1；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            report["motion_started"] = True
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                   encoding="utf-8")
            execute_route(robot, config, plan["route_joint_rad"], lambda: interrupted,
                          speed_percent=5, target_tolerance_rad=0.005,
                          waypoint_timeout_s=20, waypoint_max_timeout_s=180,
                          pause_on_stationary_timeout=True,
                          path_mode="joint_box", joint_box_margin_rad=0.03)
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"],
                                                   config["safe_start_joint_rad"]))
            report.update(final_status=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("S1 关节误差超过 0.005 rad")
            report["completed"] = True
    except Exception as exc:
        report.update(error=str(exc), error_type=type(exc).__name__)
        raise RuntimeError(f"回 S1 未完成：{exc}；报告 {report_path}") from exc
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
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.run:
            if args.plan is None:
                parser.error("执行时需要 --plan")
            run(args.plan, args.config)
        else:
            plan = prepare(args.config)
            path = _save(plan, "nero_drag_stop_return_plan_")
            print("只读回位计划：", path)
            print("四个控制器目标和法兰关节盒预测 (m)：")
            for index, box in enumerate(plan["joint_box_flange_ranges_m"], 1):
                print(index, box)
            print("速度 5%；未发送运动指令。")
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError) as exc:
        print(f"本次回位未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
