"""One-time 5 mm base-Y move_j direction probe from the H0 precontact pose.

This does not enter drag mode, contact the support, or disable the arm.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.control_interfaces.ik_preview import solve_nearby_pose
from experiments.single_arm.lab.device import connected, read_state, require_ready
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import DEFAULT_CONFIG, load_config


SOURCE_Q = [0.19952604008799177, 0.3085916650451174,
            -0.028082347664588763, -0.1539903999034597,
            1.6761967603228343, 0.13178981181809182, 0.5925218277595549]
REPORT_DIR = Path("experiments/single_arm/lab/data/h0_y_probes")


def plan(robot, state, config, delta_y_m):
    if delta_y_m != -0.005:
        raise ValueError("本次只允许从当前姿态探测基座 Y 负向 5 mm")
    require_ready(state)
    start = state["joint_rad"]
    if max(abs(a-b) for a, b in zip(start, SOURCE_Q)) > 0.005:
        raise ValueError("当前姿态已偏离本次桌边前位置；拒绝套用探测")
    flange = state["flange_pose_m_rad"]
    if math.dist(robot.fk(start)[:3], flange[:3]) > 0.005:
        raise ValueError("法兰反馈与模型不符")
    target_pose = list(flange)
    target_pose[1] += delta_y_m
    target = solve_nearby_pose(
        robot, start, target_pose, config["zero_exclusion_radius_rad"])
    if max(abs(a-b) for a, b in zip(start, target)) > 0.04:
        raise ValueError("5 mm 探测所需关节变化超限")
    predicted = robot.fk(target)
    if (abs(predicted[0]-flange[0]) > 0.001
            or abs(predicted[1]-target_pose[1]) > 0.0003
            or abs(predicted[2]-flange[2]) > 0.001):
        raise ValueError("模型没有得到指定的 Y 向小位移")
    boxes = _assess(robot, [start, target], config, -0.12)
    if boxes[0]["x"][0] < -0.12 or boxes[0]["x"][1] > -0.10:
        raise ValueError("探测可能偏离当前离桌位置")
    return start, target, boxes


def run(*, config_path=DEFAULT_CONFIG, countdown_s=5):
    config = load_config(Path(config_path))
    if config["mount"] != "left":
        raise ValueError("本次方向探测仅适用于左侧安装")
    report = {"kind": "one_time_h0_y_direction_probe_move_j",
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "delta_y_m": -0.005, "completed": False,
              "automatic_return": False, "automatic_disable": False}
    report_path = _write_new(REPORT_DIR, "nero_h0_y_probe_", report)
    interrupted = False
    old_handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state = read_state(robot)
            start, target, boxes = plan(robot, state, config, -0.005)
            report.update(start_state=state, target_joint_rad=target,
                          joint_box_flange_ranges_m=boxes)
            print("Y 负向 5 mm 探测；起点/预测目标法兰 XYZ：",
                  state["flange_pose_m_rad"][:3], robot.fk(target)[:3], flush=True)
            print("预测范围：", boxes, flush=True)
            old_handlers = (signal.signal(signal.SIGINT, stop),
                            signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后探测；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live = read_state(robot)
            live_start, live_target, _ = plan(robot, live, config, -0.005)
            if (max(abs(a-b) for a, b in zip(start, live_start)) > 0.003
                    or max(abs(a-b) for a, b in zip(target, live_target)) > 0.003):
                raise RuntimeError("倒计时期间姿态变化；未发送运动指令")
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")
            execute_route(
                robot, config, [live_start, live_target],
                lambda: interrupted, speed_percent=2,
                target_tolerance_rad=0.005, waypoint_timeout_s=20,
                waypoint_max_timeout_s=120, pause_on_stationary_timeout=True,
                path_mode="joint_box", joint_box_margin_rad=0.03,
                on_emergency_stop=lambda reason: report.update(
                    emergency_stop_requested=True, emergency_stop_reason=reason))
            final = require_ready(read_state(robot))
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], target))
            report.update(final_state=final, final_max_joint_error_rad=error)
            if error > 0.005:
                raise RuntimeError("方向探测到位误差超限")
            report["completed"] = True
            print(f"Y 向探测到位；最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report.update(error_type=type(exc).__name__, error=str(exc))
        raise RuntimeError(f"Y 向探测未完成：{exc}；报告 {report_path}") from exc
    finally:
        if old_handlers is not None:
            signal.signal(signal.SIGINT, old_handlers[0])
            signal.signal(signal.SIGTERM, old_handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
        print("方向探测报告：", report_path, flush=True)
    return report_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="store_true", help="按实时状态复核后执行一次")
    args = parser.parse_args(argv)
    if not args.run:
        parser.error("本次只提供单命令实时预检与执行；须显式加 --run")
    try:
        run()
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
