"""One-off move_j recovery after the 2026-09-26 unexpected drag motion.

The first stage joins a hand-guided, observed clearance pose. The separate
contact stage approaches H0 by J4, as in that recording. This is not the
blocked generic S1/H0 route and never disables the arm.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import sys
import time

from experiments.single_arm.lab.device import connected, read_state
from experiments.single_arm.lab.home_demo import _post_stop_reasons
from experiments.single_arm.lab.planned_return import _assess, _write_new
from experiments.single_arm.start_transfer.go_to_start import execute_route
from experiments.single_arm.teaching.teach_session import (
    DEFAULT_CONFIG, load_config, validate_joints,
)


H0_FILE = Path("experiments/single_arm/lab/config/supported_home.json")
REPORT_DIR = Path("experiments/single_arm/lab/data/h0_jump_recoveries")
# The operator rejected the clearance direction after stage 1; block the
# unexecuted contact stage and any replay of stage 1 before connecting CAN.
ROUTE_BLOCKED = True
START_Q = [0.4295255289158045, 0.18811158677994885,
           0.00008726646259971648, 0.11383037381507018,
           1.5599229255549718, 0.1318945315732115, 1.370624514883667]
# 26.05 s in the 09:32 hand-guided route, before its contact with the desk.
# At this posture the operator reported no contact; the final 18 mm was
# reached in that recording mainly by changing J4 with J2 held near 0.31.
HAND_GUIDED_CSV = Path(
    "experiments/single_arm/lab/data/home_demos/recordings/"
    "nero_poses_lab_20260926T093258489363Z.csv")
HAND_GUIDED_ROW = 520
TOLERANCE_RAD = 0.005


def _observed_precontact():
    import csv
    with HAND_GUIDED_CSV.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) <= HAND_GUIDED_ROW:
        raise ValueError("原始安全示范缺少预接触样本")
    row = rows[HAND_GUIDED_ROW]
    q = [float(row[f"joint_{index}_rad"]) for index in range(1, 8)]
    xyz = [float(row[key]) for key in ("x_m", "y_m", "z_m")]
    if (abs(float(row["elapsed_s"]) - 26.05) > 0.02
            or math.dist(xyz, [-0.111, -0.022, 0.707]) > 0.005
            or row["arm_status"] != "NORMAL"):
        raise ValueError("原始示范的预接触样本已变化")
    return q


def _state_ok(state):
    reasons = _post_stop_reasons(state)
    if reasons:
        raise RuntimeError("当前状态不允许回零：" + "；".join(reasons))
    if state["control_mode"] not in (1, 2):
        raise RuntimeError("控制器模式异常")


def build_route(robot, state, config, stage):
    """Reject any start not matching this incident or the observed precontact."""
    _state_ok(state)
    current = validate_joints(state["joint_rad"], config["zero_exclusion_radius_rad"])
    if math.dist(robot.fk(current)[:3], state["flange_pose_m_rad"][:3]) > 0.02:
        raise ValueError("实时法兰反馈与模型不一致")
    pre = validate_joints(_observed_precontact(), config["zero_exclusion_radius_rad"])
    h0 = validate_joints(json.loads(H0_FILE.read_text(encoding="utf-8"))["joint_rad"],
                         config["zero_exclusion_radius_rad"])
    if stage == "clearance":
        if max(abs(a-b) for a, b in zip(current, START_Q)) > 0.01:
            raise ValueError("当前已偏离本次异动后的姿态；不套用一次性路线")
        first = list(current)
        first[1] = -0.1
        second = list(first)
        for axis in (0, 2, 4, 5, 6):
            second[axis] = pre[axis]
        third = list(second)
        third[3] = pre[3]
        route = [current, first, second, third, pre]
        lower_x = [-0.11, 0.035, 0.035, -0.12]
    elif stage == "contact":
        if max(abs(a-b) for a, b in zip(current, pre)) > TOLERANCE_RAD:
            raise ValueError("机械臂未停在手拖实测的桌边前位置；拒绝贴边")
        route = [current, h0]
        lower_x = [-0.13]
    else:
        raise ValueError("阶段只能是 clearance 或 contact")
    boxes = _assess(robot, route, config, -0.13)
    if any(box["x"][0] < bound for box, bound in zip(boxes, lower_x)):
        raise ValueError("候选路线没有保持预期的桌边间隙")
    return route, boxes


def run(stage, *, config_path=DEFAULT_CONFIG, countdown_s=5):
    if ROUTE_BLOCKED:
        raise RuntimeError("现场确认回零方向错误：应核对基座 Y 外移；本次 X 向路线及贴边段已停用")
    config = load_config(Path(config_path))
    if config["mount"] != "left":
        raise ValueError("一次性回零路线只适用于现场左侧安装")
    report = {"kind": "h0_after_drag_jump_move_j", "stage": stage,
              "started_at_utc": datetime.now(timezone.utc).isoformat(),
              "completed": False, "automatic_disable": False,
              "hand_guided_source": str(HAND_GUIDED_CSV),
              "completed_targets": []}
    report_path = _write_new(REPORT_DIR, "nero_h0_jump_recovery_", report)
    interrupted = False
    old_handlers = None

    def stop(_signum, _frame):
        nonlocal interrupted
        interrupted = True

    try:
        with connected(config) as robot:
            state = read_state(robot)
            route, boxes = build_route(robot, state, config, stage)
            report["start_state"] = state
            report["route_joint_rad"] = route
            report["joint_box_flange_ranges_m"] = boxes
            report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                   encoding="utf-8")
            print(f"{stage}：{len(route)-1} 个 move_j 目标；实时法兰 XYZ ",
                  state["flange_pose_m_rad"][:3], flush=True)
            print("目标法兰 XYZ：", robot.fk(route[-1])[:3], flush=True)
            print("各段预测法兰范围：", boxes, flush=True)
            old_handlers = (signal.signal(signal.SIGINT, stop),
                            signal.signal(signal.SIGTERM, stop))
            for remaining in range(countdown_s, 0, -1):
                if interrupted:
                    raise InterruptedError("倒计时取消；未发送运动指令")
                print(f"{remaining} 秒后执行 {stage}；按 Ctrl+C 取消。", flush=True)
                time.sleep(1)
            live = read_state(robot)
            live_route, _ = build_route(robot, live, config, stage)
            if max(abs(a-b) for a, b in zip(live_route[0], route[0])) > 0.003:
                raise RuntimeError("倒计时期间实际姿态变化；未发送运动指令")
            if interrupted:
                raise InterruptedError("倒计时取消；未发送运动指令")

            batches = ([(route[:4], 5, 0), (route[3:], 2, 3)]
                       if stage == "clearance" else [(route, 2, 0)])
            for points, speed, offset in batches:
                if interrupted:
                    raise InterruptedError("尚未发送下一目标")
                execute_route(
                    robot, config, points, lambda: interrupted,
                    speed_percent=speed, target_tolerance_rad=TOLERANCE_RAD,
                    waypoint_timeout_s=20, waypoint_max_timeout_s=120,
                    pause_on_stationary_timeout=True, path_mode="joint_box",
                    joint_box_margin_rad=0.03,
                    on_emergency_stop=lambda reason: report.update(
                        emergency_stop_requested=True, emergency_stop_reason=reason))
                report["completed_targets"].extend({
                    "index": offset + index, "target_joint_rad": target}
                    for index, target in enumerate(points[1:], 1))
                report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                                       encoding="utf-8")
            final = read_state(robot)
            error = max(abs(a-b) for a, b in zip(final["joint_rad"], route[-1]))
            report["final_state"] = final
            report["final_max_joint_error_rad"] = error
            if final["arm_status"] != 0 or error > TOLERANCE_RAD:
                raise RuntimeError("到位状态或关节误差不合格")
            report["completed"] = True
            print(f"{stage} 完成；最大关节误差 {error:.6f} rad。", flush=True)
    except Exception as exc:
        report["error_type"] = type(exc).__name__
        report["error"] = str(exc)
        raise RuntimeError(f"一次性 H0 回零未完成：{exc}；报告 {report_path}") from exc
    finally:
        if old_handlers is not None:
            signal.signal(signal.SIGINT, old_handlers[0])
            signal.signal(signal.SIGTERM, old_handlers[1])
        report["finished_at_utc"] = datetime.now(timezone.utc).isoformat()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n",
                               encoding="utf-8")
        print("回零报告：", report_path, flush=True)
    return report_path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("clearance", "contact"), required=True)
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args(argv)
    if not args.run:
        parser.error("此一次性入口只在现场确认后运行；需显式加 --run")
    try:
        run(args.stage)
    except (OSError, ValueError, RuntimeError, InterruptedError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
