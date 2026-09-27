"""仅用于本次立方体拖动后的实时姿态回 S1；默认只读规划。

先让 J2 朝 S1 方向移动，使左侧安装的法兰沿基座 X 正向离桌；
再以小关节盒目标同步收拢其余轴。每一步都重新检查反馈，失败不续跑。
旧计划与未知姿态一律拒绝。本程序不会复位或使能。
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


PLAN_DIR = Path("experiments/single_arm/lab/data/cube_drag_returns")
STEP_RAD = 0.025
REFERENCE_JOINT_RAD = [0.0806, -0.5111, -0.3231, -0.4039,
                       1.5293, -0.6632, 0.0153]


def _now():
    return datetime.now(timezone.utc).isoformat()


def _save(data, prefix):
    PLAN_DIR.mkdir(parents=True, exist_ok=True)
    path = PLAN_DIR / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def _route(start, target, config):
    """两段、一条连续执行路线；单次逐轴目标差不超过 0.025 rad。"""
    middle = list(start)
    middle[1] = target[1]
    stages = [list(start), middle, list(target)]
    route = [list(start)]
    for first, last in zip(stages, stages[1:]):
        count = max(1, math.ceil(max(abs(b-a) for a, b in zip(first, last)) / STEP_RAD))
        for step in range(1, count+1):
            fraction = step/count
            route.append(validate_joints(
                [a+fraction*(b-a) for a, b in zip(first, last)],
                config["zero_exclusion_radius_rad"],
            ))
    return route, [len(route) - 1 - max(1, math.ceil(max(
        abs(b-a) for a, b in zip(middle, target)) / STEP_RAD)), len(route)-1]


def _envelope(robot, route):
    """逐小步八角关节盒 FK；控制器各轴可在相邻目标间不同步。"""
    points = []
    for first, last in zip(route, route[1:]):
        for corners in product((0, 1), repeat=7):
            joints = [last[i] if corners[i] else first[i] for i in range(7)]
            points.append([float(value) for value in robot.fk(joints)[:3]])
    return {axis: [min(point[i] for point in points),
                   max(point[i] for point in points)]
            for i, axis in enumerate("xyz")}


def _preflight(robot, config):
    state = read_state(robot)
    if (state["control_mode"] not in (1, 2) or state["arm_status"] != 0
            or state["teach_status"] not in (0, 2, 6) or state["error_code"] != 0
            or state["joint_variation_0_25s_rad"] > 0.001
            or len(state["drivers"]) != 7
            or any(not item["enabled"] or item["undervoltage"] or item["driver_error"]
                   for item in state["drivers"])):
        raise RuntimeError("控制器状态、示教停止、停稳或七轴驱动未通过回位预检")
    q = validate_joints(state["joint_rad"], config["zero_exclusion_radius_rad"])
    if max(abs(a-b) for a, b in zip(q, REFERENCE_JOINT_RAD)) > 0.04:
        raise ValueError("不在本次拖动后的已检查姿态附近；拒绝套用回位路线")
    target = validate_joints(config["safe_start_joint_rad"],
                             config["zero_exclusion_radius_rad"])
    route, stage_ends = _route(q, target, config)
    validate_route_workspace(robot, route, config)
    envelope = _envelope(robot, route)
    if (envelope["x"][0] < 0.35 or envelope["x"][1] > 0.42
            or envelope["y"][0] < -0.04 or envelope["y"][1] > 0.02
            or envelope["z"][0] < 0.50 or envelope["z"][1] > 0.60):
        raise ValueError("本次预测关节小盒法兰范围超出现场复核窗")
    if math.dist(robot.fk(q)[:3], state["flange_pose_m_rad"][:3]) > 0.02:
        raise RuntimeError("当前法兰反馈与正运动学不一致")
    return state, route, stage_ends, envelope


def prepare(config_path=DEFAULT_CONFIG):
    config_path = Path(config_path)
    config = load_config(config_path)
    if config["mount"] != "left":
        raise ValueError("只适用于当前左侧安装")
    with connected(config, None) as robot:
        state, route, ends, envelope = _preflight(robot, config)
    return {"schema_version": 1, "kind": "one_time_cube_drag_return_to_s1",
            "created_at_utc": _now(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "start_joint_rad": state["joint_rad"],
            "target_joint_rad": config["safe_start_joint_rad"],
            "route_joint_rad": route, "stage_end_indices": ends,
            "flange_joint_box_envelope_m": envelope,
            "speed_percent": 5,
            "site_note": "左侧装、裸法兰；现场需确认桌面、腕部、支撑和线缆全程间隙。",
            "not_general_start_initializer": True}


def run(plan_path, config_path=DEFAULT_CONFIG, countdown_s=5):
    path = Path(plan_path)
    plan = json.loads(path.read_text(encoding="utf-8"))
    config_path = Path(config_path)
    config = load_config(config_path)
    if (plan.get("kind") != "one_time_cube_drag_return_to_s1"
            or plan.get("schema_version") != 1
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("speed_percent") != 5
            or plan.get("target_joint_rad") != config["safe_start_joint_rad"]):
        raise ValueError("本次回位计划与配置不符")
    expected, ends = _route(plan["start_joint_rad"],
                            config["safe_start_joint_rad"], config)
    if plan.get("route_joint_rad") != expected or plan.get("stage_end_indices") != ends:
        raise ValueError("回位路线与本次固定算法不符")
    report = {"schema_version": 1, "kind": "one_time_cube_drag_return_to_s1",
              "started_at_utc": _now(), "plan": str(path), "completed": False,
              "last_published_index": 0, "feedback_sample_count": 0}
    report_path = _save(report, "nero_cube_drag_return_run_")
    interrupted = False
    previous = None

    def stop(_number, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, None) as robot:
            state, _live_route, _live_stage_ends, envelope = _preflight(robot, config)
            if (max(abs(a-b) for a, b in zip(state["joint_rad"], plan["start_joint_rad"])) > 0.002
                    or max(abs(a-b) for axis in "xyz" for a, b in zip(
                        envelope[axis], plan["flange_joint_box_envelope_m"][axis]
                    )) > 0.005):
                raise RuntimeError("实时起点或离线几何已变化；未发送运动命令")
            previous = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消")
                print(f"{remaining} 秒后从当前姿态回 S1；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)

            # 记录文件和额外 FK 不挂在发布后的回调上。回调一旦抛错，
            # 底层执行器会在已发目标后触发电子急停；本次事故正由此引起。
            report["motion_started"] = True
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                   encoding="utf-8")
            execute_route(robot, config, expected, lambda: interrupted,
                          speed_percent=5, target_tolerance_rad=0.005,
                          waypoint_timeout_s=5, waypoint_max_timeout_s=15,
                          pause_on_stationary_timeout=True,
                          path_mode="joint_box", joint_box_margin_rad=0.02)
            final = read_state(robot)
            error = max(abs(a-b) for a, b in zip(final["joint_rad"],
                                                   config["safe_start_joint_rad"]))
            report.update(final_status=final, final_max_joint_error_rad=error)
            if not final["ready_for_motion"] or error > 0.005:
                raise RuntimeError("回位后状态或关节误差未通过独立复核")
            report["completed"] = True
    except Exception as exc:
        report.update(error=str(exc), error_type=type(exc).__name__)
        raise RuntimeError(f"回 S1 未完成：{exc}；报告 {report_path}") from exc
    finally:
        if previous is not None:
            signal.signal(signal.SIGINT, previous[0])
            signal.signal(signal.SIGTERM, previous[1])
        report["finished_at_utc"] = _now()
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
                parser.error("执行时必须给本次只读生成的 --plan")
            run(args.plan, args.config)
        else:
            plan = prepare(args.config)
            path = _save(plan, "nero_cube_drag_return_plan_")
            print("只读回 S1 计划：", path)
            print("当前/目标七轴 (rad)：", plan["start_joint_rad"], plan["target_joint_rad"])
            print("小步数和两段结束索引：", len(plan["route_joint_rad"])-1,
                  plan["stage_end_indices"])
            print("每步关节盒法兰预测范围 (m)：", plan["flange_joint_box_envelope_m"])
            print("速度 5%；未发送运动指令。")
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError) as exc:
        print(f"本次回位未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
