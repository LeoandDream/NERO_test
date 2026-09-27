"""Site trial for S1 <-> table-supported H0 with an early negative-Y detour.

The sparse joint targets are planned from measured poses, not a replay of the
hand-guided recording. Each invocation checks the live state before motion.
The old quick-park route remains blocked. Reaching H0 never disables the arm.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
from itertools import product
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.planned_return import (
    _assess, _subdivide_long_segments, _write_new,
)
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints,
)


HOME_PATH = Path("experiments/single_arm/lab/config/supported_home.json")
SOURCE_PATH = Path(
    "experiments/single_arm/lab/data/home_demos/recordings/"
    "nero_poses_lab_20260926T093258489363Z.csv"
)
SOURCE_SHA256 = "2d633392ea49f64f9ccdd4b69b7155363de718eefeb3401deb5a80e0cf61cc82"
REPORT_DIR = Path("experiments/single_arm/lab/data/high_y_home")
STAGES = ("s1-to-outer", "outer-to-h0", "h0-to-s1")
START_TOLERANCE_RAD = 0.005
# The controller reached H0, but the operator reported contact or an incorrect
# support point. No stage may be executed again until the physical cause is
# identified and a replacement route is validated.
EXECUTION_HOLD = True


def _source_points(robot, config):
    if hashlib.sha256(SOURCE_PATH.read_bytes()).hexdigest() != SOURCE_SHA256:
        raise ValueError("H0 手拖实测源已变化")
    with SOURCE_PATH.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    points = {}
    for name, second in (("high", 4.0), ("outer_mid", 5.5),
                         ("outer_turn", 6.5), ("outer", 10.0),
                         ("precontact", 20.0)):
        row = min(rows, key=lambda item: abs(float(item["elapsed_s"]) - second))
        if abs(float(row["elapsed_s"]) - second) > 0.051:
            raise ValueError(f"手拖源缺少 {second:g} 秒附近的反馈")
        joints = validate_joints(
            [float(row[f"joint_{axis}_rad"]) for axis in range(1, 8)],
            config["zero_exclusion_radius_rad"],
        )
        measured = [float(row[f"{axis}_m"]) for axis in "xyz"]
        if math.dist(robot.fk(joints)[:3], measured) > 0.005:
            raise ValueError(f"手拖源 {name} 的法兰反馈与模型不符")
        points[name] = joints
    return points


def _routes(robot, config, home):
    s1 = validate_joints(config["safe_start_joint_rad"],
                         config["zero_exclusion_radius_rad"])
    h0 = validate_joints(home["joint_rad"],
                         config["zero_exclusion_radius_rad"])
    samples = _source_points(robot, config)
    high_outward = list(s1)
    high_outward[0] = -0.2  # base Y approximately -0.096 m while X is still high
    high = list(samples["high"])
    high[0] = 0.4  # keep the high-pose flange on the outward Y side
    outer_mid = list(samples["outer_mid"])
    outer_mid[0] = 0.7
    outer = samples["outer"]
    y_release = list(h0)
    y_release[0] = 0.5  # first departure movement is negative base Y
    return {
        "s1-to-outer": _subdivide_long_segments([
            s1, high_outward, high, outer_mid, samples["outer_turn"], outer,
        ]),
        "outer-to-h0": [outer, samples["precontact"], h0],
        "h0-to-s1": _subdivide_long_segments([
            h0, y_release, outer, samples["outer_turn"], outer_mid,
            high, high_outward, s1,
        ]),
    }


def _check_outer_corridor(robot, route):
    """Check paired X/Y samples; independent axis ranges lose correlation."""
    for start, target in zip(route, route[1:]):
        changed = [axis for axis in range(7)
                   if abs(start[axis] - target[axis]) > 1e-6]
        points = []
        for flags in product((0, 1), repeat=len(changed)):
            joints = list(start)
            for axis, flag in zip(changed, flags):
                if flag:
                    joints[axis] = target[axis]
            points.append(joints)
        for index in range(101):
            fraction = index / 100
            points.append([a + fraction*(b-a) for a, b in zip(start, target)])
        for joints in points:
            xyz = robot.fk(joints)
            if xyz[0] < 0.10 and xyz[1] > -0.025:
                raise ValueError("低位段可能靠近朝人一侧桌边")


def plan(robot, state, config, stage):
    require_ready(state)
    if config["mount"] != "left" or stage not in STAGES:
        raise ValueError("只允许左侧安装的 S1/H0 现场试验阶段")
    home_bytes = HOME_PATH.read_bytes()
    home = json.loads(home_bytes)
    if (home.get("name") != "H0" or home.get("route_blocked") is not True
            or home.get("disable_verified") is not False
            or home.get("config_sha256") != hashlib.sha256(
                Path(DEFAULT_CONFIG).read_bytes()).hexdigest()):
        raise ValueError("H0 配置或原路线锁定状态已变化")
    routes = _routes(robot, config, home)
    expected = routes[stage][0]
    actual = validate_joints(state["joint_rad"],
                             config["zero_exclusion_radius_rad"])
    if max(abs(a-b) for a, b in zip(actual, expected)) > START_TOLERANCE_RAD:
        raise ValueError("当前姿态不在该阶段起点；未发送运动指令")
    if math.dist(robot.fk(actual)[:3], state["flange_pose_m_rad"][:3]) > 0.005:
        raise ValueError("实时法兰反馈与模型不符")
    route = [actual, *routes[stage][1:]]
    boxes = _assess(robot, route, config, -0.19)
    if stage == "s1-to-outer":
        if (robot.fk(route[1])[1] > -0.08 or robot.fk(route[1])[0] < 0.3):
            raise ValueError("未在高位先沿 Y 负向绕开桌边")
        # Check joint-box corners and the diagonal together, retaining the
        # X/Y correlation. This does not model the physical tabletop, cables,
        # or the controller's actual path.
        _check_outer_corridor(robot, route)
    elif stage == "outer-to-h0":
        if boxes[0]["y"][1] > -0.03 or boxes[1]["y"][1] > -0.02:
            raise ValueError("贴边段不满足 Y 向缓慢接近条件")
    else:
        first = robot.fk(route[0])
        second = robot.fk(route[1])
        if second[1] > first[1] - 0.025 or boxes[0]["y"][1] > first[1] + 0.002:
            raise ValueError("从 H0 出发的第一目标未沿 Y 负向离开桌边")
        _check_outer_corridor(robot, route[1:])
    speeds = ([5] * (len(route)-1) if stage == "s1-to-outer" else
              [2, 2] if stage == "outer-to-h0" else
              [2] + [5] * (len(route)-2))
    return {"stage": stage, "route_joint_rad": route,
            "joint_box_flange_ranges_m": boxes,
            "speed_percent_by_target": speeds,
            "home_config_sha256": hashlib.sha256(home_bytes).hexdigest(),
            "source_sha256": SOURCE_SHA256,
            "automatic_disable": False}


def run(stage, *, execute=False, config_path=DEFAULT_CONFIG, countdown_s=5,
        robot_factory=None):
    if execute and EXECUTION_HOLD:
        raise RuntimeError("H0 贴边实机验收未通过；新路线全部锁定，不发送运动指令")
    config = load_config(Path(config_path))
    report = {"kind": "high_y_s1_h0_move_j_trial", "stage": stage,
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "completed_targets": [], "emergency_stop_requested": False}
    report_path = _write_new(REPORT_DIR, "nero_high_y_home_", report)
    interrupted = False
    handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config, robot_factory) as robot:
            state = read_state(robot)
            planned = plan(robot, state, config, stage)
            report.update(start_state=state, preflight_passed=True,
                          execution_requested=execute, **planned)
            route = planned["route_joint_rad"]
            print(f"{stage}：{len(route)-1} 个 move_j 目标；起终点法兰 XYZ：",
                  state["flange_pose_m_rad"][:3], robot.fk(route[-1])[:3], flush=True)
            print("逐段关节盒法兰范围：", planned["joint_box_flange_ranges_m"], flush=True)
            if not execute:
                print("只读预检完成；未发送运动指令。", flush=True)
                return report_path
            handlers = (signal.signal(signal.SIGINT, stop),
                        signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后执行 {stage}；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live = plan(robot, read_state(robot), config, stage)
            if max(abs(a-b) for a, b in zip(
                    route[0], live["route_joint_rad"][0])) > 0.002:
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            route[0] = live["route_joint_rad"][0]
            for index, speed in enumerate(planned["speed_percent_by_target"], 1):
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                execute_route(
                    robot, config, route[index-1:index+1],
                    lambda: interrupted, speed_percent=speed,
                    target_tolerance_rad=0.005, waypoint_timeout_s=20,
                    waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                    path_mode="joint_box", joint_box_margin_rad=0.03,
                    on_waypoint=lambda _index, target: report["completed_targets"].append({
                        "index": index, "joint_rad": target, "speed_percent": speed}),
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason),
                )
                report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                       encoding="utf-8")
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], route[-1]))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("终点关节误差超限")
            report["completed"] = True
            print(f"{stage} 完成；{len(route)-1}/{len(route)-1} 目标到位；"
                  f"最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"高位 Y 绕行未完成：{exc}；报告 {report_path}") from exc
    finally:
        if handlers is not None:
            signal.signal(signal.SIGINT, handlers[0])
            signal.signal(signal.SIGTERM, handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
        print("高位 Y 绕行报告：", report_path, flush=True)
    return report_path


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
