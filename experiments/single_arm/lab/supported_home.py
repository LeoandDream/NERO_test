"""Sparse move_j route between S1 and the observed table-supported H0 pose.

H0 is a site-specific candidate. This module never disables the arm. The
recorded drag samples bind the target pose but are not replayed as commands.
"""

from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import time

from experiments.single_arm.lab.device import connected, read_state
from experiments.single_arm.lab.home_demo import _post_stop_reasons
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route, joint_point
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints,
)


HOME_FILE = Path("experiments/single_arm/lab/config/supported_home.json")
PLAN_DIR = Path("experiments/single_arm/lab/data/supported_home_plans")
RUN_DIR = Path("experiments/single_arm/lab/data/supported_home_runs")
POSE_TOLERANCE_RAD = 0.005


def load_home(config_path=DEFAULT_CONFIG, home_path=HOME_FILE):
    config_path, home_path = Path(config_path), Path(home_path)
    config = load_config(config_path)
    home = json.loads(home_path.read_text(encoding="utf-8"))
    if home.get("route_blocked"):
        raise RuntimeError("H0 回零路线已停用：" + home.get(
            "route_block_reason", "需重新核对目标和桌边接触"))
    if (home.get("schema_version") != 1 or home.get("name") != "H0"
            or home.get("robot_model") != "NERO"
            or home.get("mount") != config["mount"]
            or home.get("tool_offset_m") != [0, 0, 0]
            or home.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or home.get("disable_verified") is not False):
        raise ValueError("回零位配置与当前安装、设备或示教配置不匹配")
    target = validate_joints(home["joint_rad"], config["zero_exclusion_radius_rad"])
    source_path = Path(home["source_report"])
    source_bytes = source_path.read_bytes()
    source = json.loads(source_bytes)
    assessment = json.loads(Path(home["source_assessment"]).read_text(encoding="utf-8"))
    if (assessment.get("recording_accepted") is not True
            or assessment.get("source_report_sha256") != hashlib.sha256(source_bytes).hexdigest()
            or assessment.get("source_report") != str(source_path)
            or max(abs(a-b) for a, b in zip(target,
                      source["post_stop_state"]["joint_rad"])) > 1e-9):
        raise ValueError("回零位与原始拖动及离线复核证据不一致")
    return config, home, target


def _forward_route(s1, h0):
    """Adjust wrist clear of the table, then approach the support by J2."""
    route = [list(s1)]
    for fraction in (0.5, 1.0):
        point = list(route[0])
        for axis in (4, 6):
            point[axis] = s1[axis] + fraction * (h0[axis] - s1[axis])
        route.append(point)
    body = list(route[-1])
    for axis in (0, 2, 3, 5):
        body[axis] = h0[axis]
    route.append(body)
    approach = list(body)
    approach[1] = h0[1] - 0.04
    route.extend((approach, list(h0)))
    return route


def _route_for(source_name, current, s1, h0):
    forward = _forward_route(s1, h0)
    route = forward if source_name == "S1" else list(reversed(forward))
    route = [list(point) for point in route]
    route[0] = list(current)
    return route


def _from_snapshot(robot, state, config, home, target_name):
    if target_name not in ("S1", "H0"):
        raise ValueError("回零路线目标只能是 S1 或 H0")
    reasons = _post_stop_reasons(state)
    reasons.extend(item for item in state.get("reasons", [])
                   if item != "控制器未处于 CAN 控制模式")
    if reasons:
        raise RuntimeError("当前状态不允许规划回零路线：" + "；".join(dict.fromkeys(reasons)))
    s1 = config["safe_start_joint_rad"]
    h0 = home["joint_rad"]
    current = validate_joints(state["joint_rad"], config["zero_exclusion_radius_rad"])
    matches = [name for name, target in (("S1", s1), ("H0", h0))
               if max(abs(a-b) for a, b in zip(current, target)) <= POSE_TOLERANCE_RAD]
    if len(matches) != 1:
        raise ValueError("实时姿态不在已记录的 S1 或 H0 附近；拒绝套用这条路线")
    source_name = matches[0]
    if source_name == target_name:
        return {"source_name": source_name, "target_name": target_name,
                "route_joint_rad": [current], "controller_targets": 0,
                "joint_box_flange_ranges_m": [], "speed_percent_by_target": []}
    route = _route_for(source_name, current, s1, h0)
    if any(math.dist(a, b) > 1.0 for a, b in zip(route, route[1:])):
        raise ValueError("回零路线单目标关节距离超过 1.0 rad")
    boxes = _assess(robot, route, config, -0.135)
    # The model keeps the flange clear for wrist/body alignment, then uses a
    # separate 22 mm support approach. Only flange FK is modeled here.
    bounds = ([0.33, 0.33, 0.33, -0.11, -0.135] if source_name == "S1"
              else [-0.135, -0.11, 0.33, 0.33, 0.33])
    if any(box["x"][0] < lower for box, lower in zip(boxes, bounds)):
        raise ValueError("回零路线关节盒预测未保持预定的离桌间隙")
    return {"source_name": source_name, "target_name": target_name,
            "route_joint_rad": route, "controller_targets": len(route)-1,
            "joint_box_flange_ranges_m": boxes,
            "speed_percent_by_target": ([5, 5, 5, 5, 2] if source_name == "S1"
                                        else [2, 5, 5, 5, 5])}


def prepare(target_name="H0", *, config_path=DEFAULT_CONFIG,
            home_path=HOME_FILE, robot_factory=None):
    config_path, home_path = Path(config_path), Path(home_path)
    config, home, _ = load_home(config_path, home_path)
    with connected(config, robot_factory) as robot:
        state = read_state(robot)
        route = _from_snapshot(robot, state, config, home, target_name)
    plan = {"schema_version": 1, "kind": "supported_home_move_j",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "home_config_sha256": hashlib.sha256(home_path.read_bytes()).hexdigest(),
            "start_state": state, "source_recording": home["source_report"],
            "reversed_recording": False, "automatic_disable": False,
            "controller_actual_path_known": False,
            "site_support": home["description"], **route}
    path = _write_new(PLAN_DIR, "nero_supported_home_plan_", plan)
    return {"plan_path": str(path), "plan": plan}


def run(plan_path, *, config_path=DEFAULT_CONFIG, home_path=HOME_FILE,
        robot_factory=None, countdown_s=5):
    plan_path = Path(plan_path)
    config_path, home_path = Path(config_path), Path(home_path)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    config, home, _ = load_home(config_path, home_path)
    if (plan.get("schema_version") != 1 or plan.get("kind") != "supported_home_move_j"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("home_config_sha256") != hashlib.sha256(home_path.read_bytes()).hexdigest()
            or plan.get("source_recording") != home["source_report"]
            or plan.get("reversed_recording") is not False
            or plan.get("automatic_disable") is not False):
        raise ValueError("回零计划版本、来源或配置不匹配")
    report = {"schema_version": 1, "kind": "supported_home_move_j_run",
              "plan_path": str(plan_path),
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "completed_targets": [],
              "automatic_disable": False, "emergency_stop_requested": False}
    report_path = _write_new(RUN_DIR, "nero_supported_home_run_", report)
    interrupted = False
    previous_handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            state = read_state(robot)
            current = _from_snapshot(robot, state, config, home, plan["target_name"])
            actual_route, planned_route = current["route_joint_rad"], plan["route_joint_rad"]
            if (current["source_name"] != plan["source_name"]
                    or current["controller_targets"] != plan["controller_targets"]
                    or len(actual_route) != len(planned_route)
                    or max(abs(a-b) for a, b in zip(state["joint_rad"],
                                                     plan["start_state"]["joint_rad"])) > 0.003
                    or any(max(abs(a-b) for a, b in zip(actual, planned)) > 0.003
                           for actual, planned in zip(actual_route, planned_route))
                    or current["speed_percent_by_target"] != plan["speed_percent_by_target"]):
                raise RuntimeError("实时起点或重新规划的关节目标与计划不符；未发送运动指令")
            report["preflight_state"] = state
            if not current["controller_targets"]:
                report["final_state"] = state
                report["completed"] = True
                return report_path
            previous_handlers = (signal.signal(signal.SIGINT, stop),
                                 signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后向 {plan['target_name']} 发送新规划的 move_j；"
                      "按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            route = actual_route
            split = 4 if plan["target_name"] == "H0" else 1
            batches = ((route[:split+1], plan["speed_percent_by_target"][0], 0),
                       (route[split:], plan["speed_percent_by_target"][split], split))
            for points, speed, offset in batches:
                if interrupted:
                    raise InterruptedError("尚未发送下一段目标")
                execute_route(
                    robot, config, points, lambda: interrupted,
                    speed_percent=speed, target_tolerance_rad=POSE_TOLERANCE_RAD,
                    waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                    pause_on_stationary_timeout=True, path_mode="joint_box",
                    joint_box_margin_rad=0.03,
                    on_waypoint=lambda index, target: report["completed_targets"].append({
                        "index": offset+index, "target_joint_rad": target,
                        "feedback_joint_rad": joint_point(robot), "speed_percent": speed}),
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason))
            final = read_state(robot)
            expected = home["joint_rad"] if plan["target_name"] == "H0" else config["safe_start_joint_rad"]
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], expected))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if not final["ready_for_motion"] or error > POSE_TOLERANCE_RAD:
                raise RuntimeError("到位后独立状态或七轴误差不合格")
            report["completed"] = True
    except Exception as exc:
        report.update(error=str(exc), error_type=type(exc).__name__)
        raise RuntimeError(f"回零路线未完成：{exc}；报告 {report_path}") from exc
    finally:
        if previous_handlers is not None:
            signal.signal(signal.SIGINT, previous_handlers[0])
            signal.signal(signal.SIGTERM, previous_handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
        print("回零路线报告：", report_path, flush=True)
    return report_path
