"""从本次六面采样中断后的低位姿态，单轴 J2 离桌；不自动回 S1。

只适用于左侧安装、裸法兰且现场已确认基座 X 正向离桌的布局。
计划由实时反馈生成，运行时重算；其他姿态必须另行规划。
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
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints, validate_route_workspace,
)


PLAN_DIR = Path("experiments/single_arm/lab/data/low_pose_escapes")
J2_TARGET_RAD = -0.2


def _save(data, prefix):
    PLAN_DIR.mkdir(parents=True, exist_ok=True)
    path = PLAN_DIR / (prefix + datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%S%fZ") + ".json")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return path


def plan_from_state(robot, state, config):
    """仅当姿态接近本次低位时，预测 J2 单轴目标的离桌趋势。"""
    require_ready(state)
    start = validate_joints(state["joint_rad"],
                            config["zero_exclusion_radius_rad"])
    x = float(state["flange_pose_m_rad"][0])
    if not (.18 <= start[1] <= .27 and -.05 <= start[2] <= .10
            and -.25 <= start[3] <= -.10 and -.10 <= x <= -.04):
        raise ValueError("不是本次六面中断后的低位姿态；拒绝使用离桌路线")
    if math.dist(robot.fk(start)[:3], state["flange_pose_m_rad"][:3]) > .02:
        raise ValueError("实时法兰反馈与正运动学不一致")
    target = list(start)
    target[1] = J2_TARGET_RAD
    validate_joints(target, config["zero_exclusion_radius_rad"])
    validate_route_workspace(robot, [start, target], config)
    points = []
    for index in range(101):
        joints = list(start)
        joints[1] += (J2_TARGET_RAD - start[1])*index/100
        validate_joints(joints, config["zero_exclusion_radius_rad"])
        xyz = [float(value) for value in robot.fk(joints)[:3]]
        if not all(math.isfinite(value) for value in xyz):
            raise ValueError("离线法兰反馈无效")
        if points and xyz[0] < points[-1][0]-.001:
            raise ValueError("预测路线未持续沿 X 正向离桌")
        points.append(xyz)
    if points[-1][0] < .15 or min(point[0] for point in points) < -.10:
        raise ValueError("单轴目标未达到离桌门槛或经过更低区域")
    return {"start_joint_rad": start, "target_joint_rad": target,
            "predicted_flange_xyz_ranges_m": {
                axis: [min(p[i] for p in points), max(p[i] for p in points)]
                for i, axis in enumerate("xyz")},
            "predicted_start_xyz_m": points[0],
            "predicted_target_xyz_m": points[-1]}


def prepare(config_path=DEFAULT_CONFIG, robot_factory=None):
    config_path = Path(config_path)
    config = load_config(config_path)
    if config["mount"] != "left":
        raise ValueError("只适用于当前左侧安装")
    with connected(config, robot_factory) as robot:
        state = read_state(robot)
        route = plan_from_state(robot, state, config)
    return {"schema_version": 1, "kind": "face_failure_low_pose_escape",
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
            "speed_percent": 5, "automatic_return": False,
            "site_assumption": "Left mount, bare flange; base X positive leaves the table; inspect all links and cables.",
            **route}


def run(plan_path, config_path=DEFAULT_CONFIG, robot_factory=None,
        countdown_s=5):
    plan_path = Path(plan_path)
    config_path = Path(config_path)
    config = load_config(config_path)
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if (plan.get("schema_version") != 1
            or plan.get("kind") != "face_failure_low_pose_escape"
            or plan.get("config_sha256") != hashlib.sha256(config_path.read_bytes()).hexdigest()
            or plan.get("speed_percent") != 5
            or plan.get("target_joint_rad", [None,None])[1] != J2_TARGET_RAD):
        raise ValueError("离桌计划版本、目标或配置不匹配")
    report = {"schema_version": 1, "kind": "face_failure_low_pose_escape",
              "plan": str(plan_path), "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "command_published": False}
    report_path = _save(report, "nero_low_pose_escape_run_")
    interrupted = False
    previous = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            state = read_state(robot)
            current = plan_from_state(robot, state, config)
            if (max(abs(a-b) for a,b in zip(current["start_joint_rad"],
                                               plan["start_joint_rad"])) > .003
                    or max(abs(a-b) for a,b in zip(current["target_joint_rad"],
                                                   plan["target_joint_rad"])) > .003):
                raise RuntimeError("实时起点已变化；未发送运动指令")
            previous = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后单轴 J2 离桌；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            state = read_state(robot)
            current = plan_from_state(robot, state, config)
            if max(abs(a-b) for a,b in zip(current["start_joint_rad"],
                                           plan["start_joint_rad"])) > .003:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            execute_route(robot, config,
                          [current["start_joint_rad"], current["target_joint_rad"]],
                          lambda: interrupted, speed_percent=5,
                          target_tolerance_rad=.005, waypoint_timeout_s=20,
                          waypoint_max_timeout_s=120,
                          pause_on_stationary_timeout=True,
                          path_mode="joint_box", joint_box_margin_rad=.03,
                          on_command_published=lambda *_: report.update(command_published=True))
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a,b in zip(final["joint_rad"],
                                                current["target_joint_rad"]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > .005 or final["flange_pose_m_rad"][0] < .15:
                raise RuntimeError("离桌到位后误差或法兰 X 不合格")
            report["completed"] = True
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"低位离桌未完成：{exc}；报告 {report_path}") from exc
    finally:
        if previous is not None:
            signal.signal(signal.SIGINT, previous[0])
            signal.signal(signal.SIGTERM, previous[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
        print("离桌报告：", report_path, flush=True)
    return report_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--run", action="store_true",
                        help="同一命令内读取实时姿态、预检并执行一次 move_j")
    args = parser.parse_args(argv)
    try:
        if args.plan is None:
            plan = prepare(args.config)
            path = _save(plan, "nero_low_pose_escape_plan_")
        else:
            path = args.plan
            plan = json.loads(path.read_text(encoding="utf-8"))
        print("本次离桌计划：", path, flush=True)
        print("实时起点七轴：", plan["start_joint_rad"], flush=True)
        print("单次 move_j 目标：", plan["target_joint_rad"], flush=True)
        print("预测法兰起点/终点 XYZ (m)：",
              plan["predicted_start_xyz_m"], plan["predicted_target_xyz_m"], flush=True)
        print("预测 XYZ 范围：", plan["predicted_flange_xyz_ranges_m"], flush=True)
        if args.run:
            print("倒计时后执行单轴离桌；不自动回 S1。", flush=True)
            run(path, args.config)
        else:
            print("只读预检完成；未发送运动指令。", flush=True)
        return 0
    except (OSError, ValueError, RuntimeError, TimeoutError, InterruptedError) as exc:
        print(f"低位离桌未完成：{exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
